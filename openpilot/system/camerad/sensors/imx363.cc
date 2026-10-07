#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <unistd.h>

#include "system/camerad/sensors/sensor.h"

namespace {

// Sony IMX analog gain: gain = 1024 / (1024 - code)
// Code range: 0 to 960, step 64
const float sensor_analog_gains_IMX363[] = {
    1.000,   // code 0
    1.067,   // code 64
    1.143,   // code 128
    1.231,   // code 192
    1.333,   // code 256
    1.455,   // code 320
    1.600,   // code 384
    1.778,   // code 448
    2.000,   // code 512
    2.286,   // code 576
    2.667,   // code 640
    3.200,   // code 704
    4.000,   // code 768
    5.333,   // code 832
    8.000,   // code 896
    16.000,  // code 960
};

__attribute__((unused))
const uint32_t imx363_analog_gains_reg[] = {
    0x000,  // 1.000x
    0x040,  // 1.067x
    0x080,  // 1.143x
    0x0C0,  // 1.231x
    0x100,  // 1.333x
    0x140,  // 1.455x
    0x180,  // 1.600x
    0x1C0,  // 1.778x
    0x200,  // 2.000x
    0x240,  // 2.286x
    0x280,  // 2.667x
    0x2C0,  // 3.200x
    0x300,  // 4.000x
    0x340,  // 5.333x
    0x380,  // 8.000x
    0x3C0,  // 16.000x
};

}  // namespace

IMX363::IMX363() {
  image_sensor = cereal::FrameData::ImageSensor::UNKNOWN;  // TODO: add IMX363 to cereal enum
  bayer_pattern = CAM_ISP_PATTERN_BAYER_RGRGRG;  // IMX363 = RGGB
  pixel_size_mm = 0.0014;  // 1.4 um pixel pitch
  data_word = false;

  frame_width = 3840;
  frame_height = 2160;
  frame_stride = (frame_width * 10 / 8);  // 4800 bytes for RAW10
  out_scale = 2;  // GPU downsamples 3840→1920, 2160→1080

  extra_height = 0;
  frame_offset = 0;

  start_reg_array.assign(std::begin(start_reg_array_imx363), std::end(start_reg_array_imx363));
  init_reg_array.assign(std::begin(init_array_imx363), std::end(init_array_imx363));

  // BAYER_XSHIFT=1 moves the analog crop one column right (X 96..3935 -> 97..3936), which
  // flips the horizontal Bayer phase (RGGB <-> GRBG) without changing output size. The IFE
  // demux config in ife.h is hardcoded for every sensor, so if it does not match this
  // sensor'''s phase the demosaic mis-assigns channels and every other output column comes
  // out with a different level. Diagnostic switch for that.
  if (getenv("BAYER_XSHIFT") != nullptr) {
    for (auto &r : init_reg_array) {
      if (r.reg_addr == 0x0345) r.reg_data = 0x61;  // X_START lo: 96 -> 97
      if (r.reg_addr == 0x0349) r.reg_data = 0x60;  // X_END   lo: 3935 -> 3936
    }
  }

  probe_reg_addr = 0x0016;       // Chip ID register (standard Sony IMX)
  probe_expected_data = 0x0363;  // IMX363 chip ID
  bits_per_pixel = 10;
  mipi_format = CAM_FORMAT_MIPI_RAW_10;
  frame_data_type = 0x2B;       // MIPI CSI-2 RAW10 data type
  mclk_frequency = 24000000;    // 24 MHz

  readout_time_ns = 15000000;   // ~15ms estimate

  ev_scale = 1.0;
  dc_gain_factor = 1;
  dc_gain_min_weight = 1;
  dc_gain_max_weight = 1;
  dc_gain_on_grey = 0.9;
  dc_gain_off_grey = 1.0;

  exposure_time_min = 2;
  exposure_time_max = 3124;  // FLL(3140) - 16 (full-res 3840x2160 30fps)

  analog_gain_min_idx = 0;
  analog_gain_rec_idx = 0;    // 1x
  // Use the first four real sensor gain stops (1x..8x).  The 8x stop is paired
  // with the matching factory ABF profile below; do not expose the 16x stop
  // until temporal denoise is available, otherwise dark scenes turn into a
  // bright but unstable/noisy image.
  analog_gain_max_idx = 14;
  analog_gain_cost_delta = -1;
  analog_gain_cost_low = 0.4;
  analog_gain_cost_high = 6.4;

  for (int i = 0; i <= analog_gain_max_idx; i++) {
    sensor_analog_gains[i] = sensor_analog_gains_IMX363[i];
  }

  // IMX363_BINNED=1: run the sensor in 2x2 binning at its native 2016x1512
  // 4:3 output so the IFE scaler is 1:1 and no sensor pixels are discarded.
  // The mode-2 table is appended to (not substituted for) the base init list: both are
  // random-write register lists applied in order, so mode 2's binning/crop/timing values
  // simply override the full-resolution ones set earlier, while the global analog/vendor
  // init from the base list is preserved.
  // Keep the stable binned default, but permit an on-device full-resolution
  // A/B without restarting the parent manager (which owns its environment).
  // This is deliberately a file switch, not a new default: full resolution
  // must prove both sharper and free of the former column artifact first.
  const bool binned = getenv("IMX363_BINNED") != nullptr &&
                      access("/data/imx363_fullres_test", F_OK) != 0;
  if (binned) {
    init_reg_array.insert(init_reg_array.end(),
                          std::begin(imx363_mode2_2016x1512_30fps),
                          std::end(imx363_mode2_2016x1512_30fps));
    frame_width = 2016;
    frame_height = 1512;
    frame_stride = (frame_width * 10 / 8);   // 2520 bytes, RAW10
    out_scale = 1;                            // sensor already did the 2x downscale
    pixel_size_mm = 0.0028;                   // 1.4um pixels binned 2x2
    exposure_time_max = 5008;                 // FLL(5024) - 16
    // Real-time low-light A/B: modeld consumes road frames at 20 Hz, so a
    // 20fps sensor cadence permits 50ms integration rather than 33ms. This
    // gains 0.58EV before any noisy gain amplification. Disabled by default
    // until cadence, motion blur and model timing are verified on-device.
    if (getenv("IMX363_20FPS") != nullptr) {
      init_reg_array.push_back({0x0340, 0x1D}); // FLL = 7536
      init_reg_array.push_back({0x0341, 0x70});
      exposure_time_max = 7520;                // FLL - 16

      // A 50 ms integration at 20 Hz is acceptable while stationary but causes
      // conspicuous directional smear in a moving vehicle at night.  This
      // opt-in profile exchanges one stop of integration for the IMX363's
      // final real analog-gain stop: 3760 lines * 16x ~= 7520 lines * 8x.
      // Keep digital gain at 1x and leave the normal profile available simply
      // by removing this environment setting.
      if (getenv("IMX363_MOTION_SAFE_NIGHT") != nullptr) {
        exposure_time_max = 3760;  // ~25 ms; cadence remains 20 Hz
        analog_gain_max_idx = 15;  // real 16x (register code 0x3c0)
        sensor_analog_gains[15] = sensor_analog_gains_IMX363[15];
      }
    }
  }

  min_ev = exposure_time_min * sensor_analog_gains[analog_gain_min_idx];
  max_ev = exposure_time_max * dc_gain_factor * sensor_analog_gains[analog_gain_max_idx];
  target_grey_factor = 0.002;
  // Runtime AE tuning hook for the V4L2 route.  Factory HAL/CamX runs a much richer AEC
  // stack than openpilot's simple grey-fraction controller; keeping this tunable lets us
  // raise the low-light target without rebuilding while comparing against HAL-like output.
  if (const char *value = getenv("IMX363_TARGET_GREY_FACTOR")) {
    char *end = nullptr;
    const float parsed = std::strtof(value, &end);
    if (end != value && *end == '\0') {
      // 0.0001 keeps the low-light target near 0.4.  The prior 0.002 floor
      // unnecessarily held the binned IMX363 about 16% below that target in
      // ordinary indoor scenes despite having ample exposure headroom.
      target_grey_factor = std::clamp(parsed, 0.0001f, 0.08f);
    }
  }

  black_level = 64;  // typical for 10-bit Sony IMX

  // Color correction matrix — plain D65 from Chromatix (Semco IMX363), row sums ≈ 1.0.
  // 2026-08-30: reverted the "AWB gains baked into CCM" variant (R=2.39/G=1.0/B=1.39,
  // row 0 summed to ~3.4 instead of ~1.0) — that broke the color-preserving property of
  // the matrix and is the leading suspect for the "画质很差" complaint that got V4L2
  // shelved. comma's own OX03C10/OS04C10 drivers in this same tree both use a plain,
  // unmodified, row-sum≈1 CCM with WB left neutral at the 0x6fc register (see ife.h) —
  // this mirrors that same convention instead of inventing a WB-in-CCM scheme.
  color_correct_matrix = {
    0x000000e2, 0x00000fa5, 0x00000ff9,  // [+1.7631, -0.7115, -0.0516]
    0x00000fe9, 0x000000c1, 0x00000fd6,  // [-0.1782, +1.5095, -0.3313]
    0x00000008, 0x00000fa0, 0x000000d9,  // [+0.0600, -0.7530, +1.6930]
  };

  // Gamma LUT from Chromatix factory calibration (258→65 downsampled)
  gamma_lut_rgb = {
    0, 36, 84, 132, 180, 231, 275, 319, 359, 398, 434, 463, 490,
    514, 536, 556, 575, 594, 613, 631, 648, 664, 678, 692, 705, 717,
    729, 741, 752, 763, 773, 784, 794, 803, 813, 822, 831, 840, 848,
    856, 865, 872, 880, 888, 895, 903, 911, 917, 924, 931, 937, 943,
    950, 956, 963, 968, 975, 980, 987, 992, 999, 1004, 1010, 1016, 1022,
  };
  // Chromatix Gamma15 contains several AEC/DRC-specific curves.  The default
  // above is the original bright/low-light curve.  Profile 2 is the factory's
  // conservative curve (entry 950): it preserves highlight headroom in strong
  // sun without inventing a non-factory tone curve.  Keep it opt-in until the
  // AEC trigger tree is fully decoded; this makes an on-device A/B reversible.
  if (const char *profile = getenv("IMX363_GAMMA_PROFILE");
      profile != nullptr && profile[0] == '2' && profile[1] == '\0') {
    gamma_lut_rgb = {
      0, 23, 52, 80, 106, 132, 158, 183, 207, 229, 251, 273, 294,
      314, 334, 354, 372, 391, 409, 426, 444, 461, 478, 494, 510, 526,
      542, 558, 573, 588, 602, 617, 631, 646, 660, 673, 687, 700, 714,
      726, 740, 752, 765, 778, 791, 803, 815, 827, 839, 851, 863, 875,
      887, 898, 910, 921, 933, 944, 955, 966, 978, 988, 1000, 1010, 1021,
    };
  }
  prepare_gamma_lut();

  // IFE ABF34 bank 0, generated from Xiaomi's own Semco IMX363
  // chromatix gain-1.0..1.8 AEC region with CamX's GenerateNoiseStandardLUT.
  // This is deliberately labelled accurately: the V4L2 path still needs the
  // corresponding gain-indexed per-frame DMI update for low-light operation.
  // Each word is {delta[17:9], base[8:0]}.
  abf34_noise_lut = {
    0x000001ff, 0x000001ff, 0x000001ff, 0x000001ff, 0x000001ff, 0x000001ff, 0x000001ff, 0x000001ff,
    0x000001ff, 0x000005ff, 0x00002ffd, 0x00002be6, 0x000025d1, 0x000023bf, 0x00001dae, 0x00001da0,
    0x00001992, 0x00001786, 0x0000157b, 0x00001371, 0x00001368, 0x0000115f, 0x00001157, 0x00000f4f,
    0x00000d48, 0x00000d42, 0x00000d3c, 0x00000d36, 0x00000b30, 0x00000b2b, 0x00000b26, 0x00000b21,
    0x0000091c, 0x00000918, 0x00000914, 0x00000910, 0x0000070c, 0x00000909, 0x00000705, 0x00000902,
    0x000006fe, 0x000006fb, 0x000006f8, 0x000004f5, 0x000006f3, 0x000006f0, 0x000004ed, 0x000006eb,
    0x000000e8, 0x000000e8, 0x000000e8, 0x000000e8, 0x000000e8, 0x000000e8, 0x000000e8, 0x000000e8,
    0x000000e8, 0x000000e8, 0x000000e8, 0x000000e8, 0x000000e8, 0x000000e8, 0x000000e8, 0x000000e8,
  };
  // Semco Chromatix AEC 1.98..3.60x region, packed by CamX ABF34 exactly.
  // Each region is stored as an independent LUT so V4L2 can track the vendor
  // noise model rather than applying the 1x filter across all mid-light gains.
  abf34_noise_lut_profiles = {abf34_noise_lut, {
    0x1ff,0x1ff,0x1ff,0x07ff,0x59fc,0x47d0,0x39ad,0x2f91,
    0x277a,0x2167,0x1f57,0x1b48,0x173b,0x1730,0x1325,0x111c,
    0x1114,0x0f0c,0x0f05,0x0cfe,0x0cf8,0x0af2,0x0aed,0x0ae8,
    0x08e3,0x08df,0x08db,0x08d7,0x08d3,0x06cf,0x06cc,0x06c9,
    0x06c6,0x06c3,0x06c0,0x04bd,0x06bb,0x04b8,0x04b6,0x06b4,
    0x04b1,0x04af,0x04ad,0x04ab,0x04a9,0x02a7,0x04a6,0x00a4,
    0x00a4,0x00a4,0x00a4,0x00a4,0x00a4,0x00a4,0x00a4,0x00a4,
    0x00a4,0x00a4,0x00a4,0x00a4,0x00a4,0x00a4,0x00a4,0x00a4,
  }, {
  // Semco Chromatix AEC 3.98..7.60x region, packed by CamX ABF34 exactly.
    0x1ff,0x1ff,0x37ff,0x67e4,0x4db1,0x3b8b,0x2f6e,0x2957,
    0x2343,0x1d32,0x1924,0x1718,0x150d,0x1303,0x10fa,0x0ef2,
    0x0eeb,0x0ce4,0x0ade,0x0cd9,0x08d3,0x0acf,0x08ca,0x08c6,
    0x08c2,0x08be,0x06ba,0x06b7,0x06b4,0x06b1,0x06ae,0x04ab,
    0x06a9,0x04a6,0x04a4,0x06a2,0x049f,0x049d,0x049b,0x0499,
    0x0297,0x0496,0x0494,0x0492,0x0290,0x048f,0x028d,0x008c,
    0x008c,0x008c,0x008c,0x008c,0x008c,0x008c,0x008c,0x008c,
    0x008c,0x008c,0x008c,0x008c,0x008c,0x008c,0x008c,0x008c,
  }, {
  // Semco Chromatix AEC 9.98..15.98x region.  This is selected for the
  // sensor's 8x stop: it intentionally errs on the stronger factory denoise
  // side while we do not have CamX temporal NR in the V4L2 route.
    0x01ff,0x39ff,0x83e3,0x59a2,0x4176,0x3556,0x293c,0x2328,
    0x1d17,0x1b09,0x14fc,0x14f2,0x10e8,0x10e0,0x0ed8,0x0cd1,
    0x0ccb,0x0ac5,0x0ac0,0x08bb,0x0ab7,0x06b2,0x08af,0x08ab,
    0x06a7,0x06a4,0x06a1,0x069e,0x049b,0x0699,0x0496,0x0494,
    0x0492,0x0690,0x048d,0x028b,0x048a,0x0488,0x0486,0x0284,
    0x0483,0x0281,0x0480,0x027e,0x047d,0x027b,0x027a,0x0079,
    0x0079,0x0079,0x0079,0x0079,0x0079,0x0079,0x0079,0x0079,
    0x0079,0x0079,0x0079,0x0079,0x0079,0x0079,0x0079,0x0079,
  }};

  // Linearization LUT — IMX363 linear sensor, knee points from Chromatix
  linearization_lut = {
    0x02000000, 0x02000000, 0x02000000, 0x02000000,  // seg 0
    0x0200003f, 0x0200003f, 0x0200003f, 0x0200003f,  // seg 1
    0x020007bf, 0x020007bf, 0x020007bf, 0x020007bf,  // seg 2
    0x02000f3f, 0x02000f3f, 0x02000f3f, 0x02000f3f,  // seg 3
    0x020016bf, 0x020016bf, 0x020016bf, 0x020016bf,  // seg 4
    0x02001e3f, 0x02001e3f, 0x02001e3f, 0x02001e3f,  // seg 5
    0x00003fff, 0x00003fff, 0x00003fff, 0x00003fff,  // seg 6
    0x00003fff, 0x00003fff, 0x00003fff, 0x00003fff,  // seg 7
    0x00003fff, 0x00003fff, 0x00003fff, 0x00003fff,  // seg 8
  };
  linearization_pts = {0x003f07bf, 0x0f3f16bf, 0x1e3f25bf, 0x2d3f34bf};

  // Vignetting LUT (GRR bank) — LSC from Chromatix Semco IMX363
  // 13×17 mesh, packed {gr_gain[25:13], r_gain[12:0]} UQ3.10
  vignetting_lut = {
    0x01de52b2, 0x01928fb3, 0x0156ed5a, 0x012e2bd6, 0x01164aec, 0x0102ca18, 0x00f42971, 0x00eac905, 0x00e728db, 0x00e908f3, 0x00f06948, 0x00fce9dc, 0x010f8aa9, 0x01264b8e, 0x014a0ce5, 0x0181af0b, 0x01c911be,
    0x01a67084, 0x01612dbc, 0x0130abe8, 0x01128abe, 0x00f9c9b4, 0x00e568cc, 0x00d7a823, 0x00cfa7bd, 0x00cca793, 0x00ce67ac, 0x00d50804, 0x00e0a896, 0x00f2c968, 0x010b0a70, 0x01274b88, 0x01532d30, 0x01986fc4,
    0x017aeec5, 0x013dec6c, 0x0118eb06, 0x00fb69cb, 0x00e188a5, 0x00cee7c0, 0x00c08708, 0x00b7468b, 0x00b3a65b, 0x00b5c67b, 0x00bdc6e4, 0x00ca878c, 0x00db485d, 0x00f30972, 0x0110aaab, 0x01330bfb, 0x016e4e1d,
    0x015bad91, 0x01280b98, 0x01076a51, 0x00e808f6, 0x00d007d2, 0x00bc26d5, 0x00aba603, 0x00a1457f, 0x009de550, 0x00a06571, 0x00a8c5de, 0x00b7469a, 0x00ca278c, 0x00e0089b, 0x00fdc9e5, 0x011ecb34, 0x01500cf4,
    0x0146acce, 0x011b2b1f, 0x00fa29c2, 0x00dae85e, 0x00c2c730, 0x00ad0619, 0x009bc542, 0x0091c4c4, 0x008e0495, 0x0090c4b6, 0x0099c526, 0x00a8a5e3, 0x00bce6e8, 0x00d3a809, 0x00f0294e, 0x01120aba, 0x013d0c45,
    0x013ccc72, 0x01138ad4, 0x00f1c96b, 0x00d36805, 0x00ba86c7, 0x00a3e5a8, 0x009284d3, 0x0087e454, 0x0083c425, 0x0086a446, 0x009044b6, 0x009fc575, 0x00b4c67f, 0x00cc87b2, 0x00e808f7, 0x010a2a67, 0x01336bee,
    0x0139ac58, 0x01116abc, 0x00efa951, 0x00d147eb, 0x00b846a9, 0x00a10586, 0x009004b7, 0x00850436, 0x0080e407, 0x0083e428, 0x008d8498, 0x009d4557, 0x00b26663, 0x00ca2797, 0x00e5a8de, 0x01082a53, 0x0130ebd2,
    0x013dac7a, 0x01146ada, 0x00f30976, 0x00d46812, 0x00bbe6da, 0x00a4e5b9, 0x0093e4e6, 0x0089a46a, 0x0085643a, 0x0088245a, 0x0091a4ca, 0x00a1058a, 0x00b56691, 0x00cd27c0, 0x00e88904, 0x010aea72, 0x0133ebf1,
    0x014a8ced, 0x011c8b27, 0x00fc49d8, 0x00dd2878, 0x00c5474f, 0x00afe63e, 0x009ea569, 0x0094c4eb, 0x009104bd, 0x0093c4df, 0x009c454e, 0x00aac609, 0x00bec70a, 0x00d4e822, 0x00f1a968, 0x0112cac3, 0x013f2c5b,
    0x0160edc8, 0x012b6bb7, 0x010aaa70, 0x00ebc91e, 0x00d347fa, 0x00bfe708, 0x00b08640, 0x00a665bf, 0x00a2c592, 0x00a505b1, 0x00acc61d, 0x00baa6d6, 0x00cc47bb, 0x00e268c4, 0x00ffea07, 0x01202b46, 0x0153ad1f,
    0x01868f4b, 0x0144ecb0, 0x011ceb28, 0x01006a03, 0x00e6e8e4, 0x00d3c801, 0x00c64752, 0x00bd66e0, 0x00ba06b5, 0x00bbe6d2, 0x00c30735, 0x00ce47cf, 0x00df289d, 0x00f6c9ad, 0x0112aacf, 0x0136ac26, 0x01740e58,
    0x01bcb190, 0x016d2e3d, 0x01386c2e, 0x0118caf6, 0x01012a03, 0x00ecc922, 0x00df087e, 0x00d6a817, 0x00d347f2, 0x00d4a808, 0x00dac858, 0x00e648e9, 0x00f8c9bc, 0x010eeaa6, 0x012acbba, 0x01588d77, 0x01a2103d,
    0x0206d44b, 0x01aa3085, 0x0166cdd5, 0x013aec2b, 0x0120ab35, 0x010eaa76, 0x00ffe9d7, 0x00f62973, 0x00f28948, 0x00f3a95d, 0x00fb89b5, 0x0107ca3f, 0x0118caf7, 0x012e8bca, 0x0155ad3b, 0x018f6f8e, 0x01dc129c,
  };

  // GBB bank -- {gb_gain[25:13], b_gain[12:0]} UQ3.10, same 13x17 mesh, same
  // Chromatix source as the GRR table above. Without this the ISP would apply the R/Gr
  // gains to B/Gb: R needs 5.073x in the corners but B only 4.096x, i.e. blue would be
  // over-corrected by ~24% at the edges (a purple cast growing toward the corners).
  vignetting_lut_gbb = {
    0x01db8eee, 0x01906c6e, 0x0154aa96, 0x012ba94b, 0x0113a894, 0x00ffa7fa, 0x00f0a788, 0x00e7073f, 0x00e36720, 0x00e5e724, 0x00ed4751, 0x00fb27a9, 0x010e2833, 0x012588d3, 0x014989e7, 0x01814b7b, 0x01c9ada8,
    0x01a6cd48, 0x01604aeb, 0x012f4968, 0x0110c873, 0x00f7a7b2, 0x00e2e70d, 0x00d4e6a0, 0x00cc8661, 0x00c98646, 0x00cbc64a, 0x00d2e670, 0x00df66c2, 0x00f1e74e, 0x010a6805, 0x012708dc, 0x01534a1d, 0x01990c47,
    0x017b8bfc, 0x013e09e6, 0x011888bd, 0x00fa67ce, 0x00e026ff, 0x00cd2667, 0x00be85f0, 0x00b4a5a2, 0x00b0e580, 0x00b3a588, 0x00bc85bd, 0x00c9a61d, 0x00dac69e, 0x00f34755, 0x0111283e, 0x0134293f, 0x016fab1d,
    0x015d8b01, 0x0129293d, 0x0107883a, 0x00e80741, 0x00cf6679, 0x00bb05d4, 0x00aa454f, 0x009f84f8, 0x009c04d9, 0x009ee4e4, 0x00a7e51d, 0x00b7058b, 0x00ca4623, 0x00e0a6d0, 0x00fee7be, 0x012088b4, 0x0151ea4e,
    0x0149aa62, 0x011d48e3, 0x00fba7dd, 0x00dba6e2, 0x00c3461d, 0x00ace56a, 0x009b24db, 0x0090a489, 0x008cc465, 0x008fe474, 0x0099a4b4, 0x00a92525, 0x00be05ca, 0x00d54683, 0x00f2276b, 0x0114e86e, 0x014069d6,
    0x01406a1f, 0x0116a8b0, 0x00f467a8, 0x00d566ad, 0x00bbc5e0, 0x00a4c524, 0x0092e499, 0x0087a441, 0x0083441e, 0x0086a431, 0x0090e479, 0x00a104f2, 0x00b6859d, 0x00cee660, 0x00eae741, 0x010de847, 0x013769a0,
    0x013e4a0d, 0x011528a1, 0x00f2c798, 0x00d3c6a2, 0x00ba25d2, 0x00a2450e, 0x0090e485, 0x0085442b, 0x0080e409, 0x00844420, 0x008ea46b, 0x009f04ea, 0x00b4c598, 0x00cd265e, 0x00e9673f, 0x010c4845, 0x0135a997,
    0x0142aa2e, 0x0118a8bb, 0x00f687b1, 0x00d746b8, 0x00be05ed, 0x00a6a52e, 0x0094e4a3, 0x008a044e, 0x0085842b, 0x0088a441, 0x0092e490, 0x00a3050c, 0x00b825b9, 0x00d0667e, 0x00ec6761, 0x010f6861, 0x013929c0,
    0x014f0a8d, 0x012088fd, 0x00ffc7f9, 0x00dfc6fb, 0x00c76635, 0x00b14582, 0x009fa4f6, 0x009504a5, 0x00912488, 0x0094449d, 0x009d64e0, 0x00aca558, 0x00c16603, 0x00d826b9, 0x00f5a7a6, 0x0117a8a3, 0x0144aa1a,
    0x01664b47, 0x012f2968, 0x010dc867, 0x00ee476b, 0x00d506a1, 0x00c18604, 0x00b16582, 0x00a6a52e, 0x00a30513, 0x00a58526, 0x00ae2567, 0x00bce5db, 0x00cf266a, 0x00e5a71d, 0x0103a813, 0x0124a906, 0x01596ac2,
    0x018cac71, 0x0148ca34, 0x012008f3, 0x0103080c, 0x00e9073f, 0x00d566a5, 0x00c78638, 0x00be05f2, 0x00ba85d5, 0x00bca5e2, 0x00c4a61a, 0x00d08678, 0x00e1e703, 0x00fa67c0, 0x0116689d, 0x013b89ab, 0x0179ebbc,
    0x01c38e17, 0x01714b72, 0x013bc9c9, 0x011b88c3, 0x0103880b, 0x00eec769, 0x00e046fb, 0x00d766bc, 0x00d3e6a6, 0x00d5c6ac, 0x00dc66d9, 0x00e90733, 0x00fbe7c7, 0x0112a875, 0x012f2959, 0x015e0ad1, 0x01a88d00,
    0x02107062, 0x01afad37, 0x016b4b19, 0x013e09b8, 0x012328f2, 0x0110c864, 0x010187f9, 0x00f727b6, 0x00f3479b, 0x00f50798, 0x00fd87da, 0x010a882f, 0x011c88ae, 0x01332957, 0x015aaa83, 0x01962c9e, 0x01e4af91,
  };
}

std::vector<i2c_random_wr_payload> IMX363::getExposureRegisters(int exposure_time, int new_exp_g, bool dc_gain_enabled) const {
  // Analog gain (0x0204/0x0205) — Sony IMX363: gain = 1024/(1024-code), code 0x000..0x3C0.
  // 2026-08-31 晚: 之前"模拟增益无效"是误判——旧代码把 4.0× 基础 + 模拟增益表值全乘进
  // DIG_GAIN(0x020E/0x020F), 夜间 AE 推到 4×4=16× 数字增益, 数字增益把传感器噪声放大
  // 16 倍 = 夜间噪点多、清晰度下降的直接原因。恢复真实模拟增益: 模拟增益随 AE 索引走
  // imx363_analog_gains_reg[], DIG_GAIN 固定 1× (不再 4× 基础)。模拟增益不放大读出噪声,
  // 只有数字增益才放大, 夜间噪点应大幅下降。
  //
  // A/B 对照开关: IMX363_LEGACY_DIGGAIN=1 强制走旧"全数字增益"路径, 用于同场景噪点对比。
  if (getenv("IMX363_LEGACY_DIGGAIN") != nullptr) {
    float dig_gain_factor = 4.0f * sensor_analog_gains_IMX363[new_exp_g];
    uint16_t dig_gain_code = (uint16_t)std::min((int)(dig_gain_factor * 256.0f + 0.5f), 0x0FFF);
    return {
      {0x0104, 0x01},
      {0x0202, (uint16_t)(exposure_time >> 8)},
      {0x0203, (uint16_t)(exposure_time & 0xFF)},
      {0x0204, 0x00}, {0x0205, 0x00},  // ANA_GAIN = 1x (legacy: fixed)
      {0x020E, (uint16_t)(dig_gain_code >> 8)}, {0x020F, (uint16_t)(dig_gain_code & 0xFF)},  // DIG_GAIN = 4x*table
      {0x0104, 0x00},
    };
  }

  uint16_t analog_gain_code = imx363_analog_gains_reg[new_exp_g];

  // A bounded intermediate exposure-compensation mode. It deliberately stays
  // opt-in while we tune the AE hand-off: use real analog gain first, then add
  // a capped digital lift. This is much quieter than the legacy 4x*table route,
  // which can reach 16x digital gain, but still gives enough headroom for dim scenes
  // where exposure time is already pinned at the binned-mode frame length.
  float digital_gain = 1.0f;
  if (const char *value = getenv("IMX363_DIGITAL_GAIN")) {
    char *end = nullptr;
    const float parsed = std::strtof(value, &end);
    if (end != value && *end == '\0') {
      digital_gain = std::clamp(parsed, 1.0f, 4.0f);
    }
  }
  const uint16_t digital_gain_code = (uint16_t)std::lround(digital_gain * 256.0f);

  // IMX363 has independent GR/R/B/GB digital-gain registers.  The vendor
  // mainline driver programs all four together; leaving R/B/GB untouched
  // makes their value depend on the sensor's previous power-on state.  Keep
  // them equal here: colour balance belongs to the calibrated IFE WB stage,
  // while this guarantees a neutral, reproducible sensor baseline.
  const uint16_t digital_gain_hi = digital_gain_code >> 8;
  const uint16_t digital_gain_lo = digital_gain_code & 0xFF;

  return {
    {0x0104, 0x01},  // GROUPED_PARAMETER_HOLD = ON
    {0x0202, (uint16_t)(exposure_time >> 8)},
    {0x0203, (uint16_t)(exposure_time & 0xFF)},
    {0x0204, (uint16_t)(analog_gain_code >> 8)}, {0x0205, (uint16_t)(analog_gain_code & 0xFF)},  // ANA_GAIN = real analog gain
    {0x020E, digital_gain_hi}, {0x020F, digital_gain_lo},  // GR
    {0x0210, digital_gain_hi}, {0x0211, digital_gain_lo},  // R
    {0x0212, digital_gain_hi}, {0x0213, digital_gain_lo},  // B
    {0x0214, digital_gain_hi}, {0x0215, digital_gain_lo},  // GB
    {0x0104, 0x00},  // GROUPED_PARAMETER_HOLD = OFF (apply)
  };
}

int IMX363::getSlaveAddress(int port) const {
  assert(port >= 0 && port <= 2);
  // IMX363 rear main camera on Xiaomi Mi 8: I2C write address 0x34
  return (int[]){0x34, 0x34, 0x34}[port];
}

float IMX363::getExposureScore(float desired_ev, int exp_t, int exp_g_idx, float exp_gain, int gain_idx) const {
  float score = std::abs(desired_ev - (exp_t * exp_gain));
  float m = exp_g_idx > analog_gain_rec_idx ? analog_gain_cost_high : analog_gain_cost_low;
  score += std::abs(exp_g_idx - (int)analog_gain_rec_idx) * m;
  score += ((1 - analog_gain_cost_delta) +
            analog_gain_cost_delta * (exp_g_idx - analog_gain_min_idx) / (analog_gain_max_idx - analog_gain_min_idx)) *
           std::abs(exp_g_idx - gain_idx) * 3.0;
  return score;
}
