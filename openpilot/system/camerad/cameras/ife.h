#pragma once

#include <cstdio>

#include "cdm.h"

#include "system/camerad/cameras/hw.h"
#include "system/camerad/sensors/sensor.h"

int build_common_ife_bps(uint8_t *dst, const CameraConfig cam, const SensorInfo *s, std::vector<uint32_t> &patches, bool ife) {
  uint8_t *start = dst;

  /*
    Common between IFE and BPS.
  */

  // IFE -> BPS addresses
  /*
  std::map<uint32_t, uint32_t> addrs = {
    {0xf30, 0x3468},
  };
  */

  // YUV
  dst += write_cont(dst, ife ? 0xf30 : 0x3468, {
    0x00680208,
    0x00000108,
    0x00400000,
    0x03ff0000,
    0x01c01ed8,
    0x00001f68,
    0x02000000,
    0x03ff0000,
    0x1fb81e88,
    0x000001c0,
    0x02000000,
    0x03ff0000,
  });

  return dst - start;
}

int build_update(uint8_t *dst, const CameraConfig cam, const SensorInfo *s, std::vector<uint32_t> &patches,
                 int abf_profile = 0, bool update_abf = false, bool abf_enabled = true) {
  uint8_t *start = dst;
  if (update_abf && !s->abf34_noise_lut_profiles.empty()) {
    uint64_t addr;
    // ABF34 bank 0 is safe to update per request; the buffer address is
    // patched by SpectraCamera to the selected factory AEC profile.
    dst += write_dmi(dst, &addr, 64*sizeof(uint32_t), 0xc24, 12);
    patches.push_back(addr - (uint64_t)start);
  }

  // init sequence
  dst += write_random(dst, {
    0x2c, 0xffffffff,
    0x30, 0xffffffff,
    0x34, 0xffffffff,
    0x38, 0xffffffff,
    0x3c, 0xffffffff,
  });

  // demux cfg
  dst += write_cont(dst, 0x560, {
    0x00000001,
    0x04440444,
    0x04450445,
    0x04440444,
    0x04450445,
    0x000000ca,
    0x0000009c,
  });

  // white balance -- see /data/wb_gains.txt ("lo hi w1", Q7, 1.0 = neutral).
  // Left neutral the IFE applies no WB at all and the D65 CCM (whose rows sum to ~1.0,
  // i.e. it expects white-balanced input) turns neutral surfaces magenta.
  // Re-read periodically so gains can be swept on a live camerad during tuning.
  {
    // Empirically calibrated on the live 1920x1080 IFE output using only
    // well-exposed near-neutral pixels: R/G=1.008, B/G=0.999 after the IFE
    // CCM. Mapping is lo=G, hi=B, w1=R; keep the file override for changing
    // illuminants and per-module tuning.
    static float wb_lo = 1.0f, wb_hi = 1.6610f, wb_w1 = 1.7400f;
    static int wb_tick = 0;
    if ((wb_tick++ % 30) == 0) {
      if (FILE *f = fopen("/data/wb_gains.txt", "r")) {
        float a, b, c;
        if (fscanf(f, "%f %f %f", &a, &b, &c) == 3 &&
            a > 0.0f && a < 8.0f && b > 0.0f && b < 8.0f && c > 0.0f && c < 8.0f) {
          wb_lo = a; wb_hi = b; wb_w1 = c;
        }
        fclose(f);
      }
    }
    auto q7 = [](float g) -> uint32_t { return (uint32_t)(g * 128.0f + 0.5f) & 0xFFFF; };
    dst += write_cont(dst, 0x6fc, {
      (q7(wb_hi) << 16) | q7(wb_lo),
      q7(wb_w1),
      0x00000000,
      0x00000000,
    });
  }

  // module config/enables (e.g. enable debayer, white balance, etc.)
  dst += write_cont(dst, 0x40, {
    // Factory CamX IFEABF34 writes lens module bit 7. The base bits are the
    // already working V4L2 path; ABF34 register/LUT programming is below.
    // ABF34 is useful once the IMX363 leaves its clean 1x daylight region,
    // but it visibly softens distant road detail at 1x. The caller changes
    // only this enable bit; the selected factory noise-profile DMI remains
    // allocated and is restored automatically as analogue gain rises.
    (0x00000c86 & (abf_enabled ? ~0u : ~0x80u)) |
      ((uint32_t)(cam.vignetting_correction) << 8),
  });
  dst += write_cont(dst, 0x44, {
    0x00000000,
  });
  dst += write_cont(dst, 0x48, {
    (1 << 3) | (1 << 1),
  });
  dst += write_cont(dst, 0x4c, {
    0x00000019,
  });
  dst += write_cont(dst, 0xf00, {
    0x00000000,
  });

  // cropping
  dst += write_cont(dst, 0xe0c, {
    0x00000e00,
  });
  dst += write_cont(dst, 0xe2c, {
    0x00000e00,
  });

  // black level scale + offset
  dst += write_cont(dst, 0x6b0, {
    ((uint32_t)(1 << 11) << 0xf) | (s->black_level << (14 - s->bits_per_pixel)),
    0x0,
    0x0,
  });

  return dst - start;
}


int build_initial_config(uint8_t *dst, const CameraConfig cam, const SensorInfo *s, std::vector<uint32_t> &patches, uint32_t out_width, uint32_t out_height) {
  uint8_t *start = dst;

  // start with the every frame config
  dst += build_update(dst, cam, s, patches);

  uint64_t addr;

  // setup
  dst += write_cont(dst, 0x478, {
    0x00000004,
    0x004000c0,
  });
  dst += write_cont(dst, 0x488, {
    0x00000000,
    0x00000000,
    0x00000f0f,
  });
  dst += write_cont(dst, 0x49c, {
    0x00000001,
  });
  dst += write_cont(dst, 0xce4, {
    0x00000000,
    0x00000000,
  });

  // linearization
  dst += write_cont(dst, 0x4dc, {
    0x00000000,
  });
  dst += write_cont(dst, 0x4e0, s->linearization_pts);
  dst += write_cont(dst, 0x4f0, s->linearization_pts);
  dst += write_cont(dst, 0x500, s->linearization_pts);
  dst += write_cont(dst, 0x510, s->linearization_pts);
  // TODO: this is DMI64 in the dump, does that matter?
  dst += write_dmi(dst, &addr, s->linearization_lut.size()*sizeof(uint32_t), 0xc24, 9);
  patches.push_back(addr - (uint64_t)start);

  // vignetting correction
  dst += write_cont(dst, 0x6bc, {
    0x0b3c0000,
    0x00670067,
    0xd3b1300c,
    0x13b1300c,
  });
  dst += write_cont(dst, 0x6d8, {
    0xec4e4000,
    0x0100c003,
  });
  dst += write_dmi(dst, &addr, s->vignetting_lut.size()*sizeof(uint32_t), 0xc24, 14); // GRR
  patches.push_back(addr - (uint64_t)start);
  dst += write_dmi(dst, &addr, s->vignetting_lut.size()*sizeof(uint32_t), 0xc24, 15); // GBB
  patches.push_back(addr - (uint64_t)start);

  // debayer
  dst += write_cont(dst, 0x6f8, {
    0x00000100,
  });
  dst += write_cont(dst, 0x71c, {
    0x00008000,
    0x08000066,
  });

  // color correction
  dst += write_cont(dst, 0x760, s->color_correct_matrix);

  // gamma
  dst += write_cont(dst, 0x798, {
    0x00000000,
  });
  dst += write_dmi(dst, &addr, s->gamma_lut_rgb.size()*sizeof(uint32_t), 0xc24, 26);  // G
  patches.push_back(addr - (uint64_t)start);

  dst += write_dmi(dst, &addr, s->gamma_lut_rgb.size()*sizeof(uint32_t), 0xc24, 28);  // B
  patches.push_back(addr - (uint64_t)start);
  dst += write_dmi(dst, &addr, s->gamma_lut_rgb.size()*sizeof(uint32_t), 0xc24, 30);  // R
  patches.push_back(addr - (uint64_t)start);

  // IMX363 factory IFE ABF34 register locations and field layout recovered
  // from Xiaomi CamX. FILTER_EN is enabled with legal neutral RNR/noise-
  // preserve fields; the DMI payload comes from the IMX363 Chromatix table.
  dst += write_cont(dst, 0x5e8, {
    0x06363349,
  });
  dst += write_cont(dst, 0x5f4, {
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x3ffb0000, 0x00000000, 0x08000400, 0x0fff0c00,
    0x000000ff, 0x000000ff, 0x000000ff, 0x000000ff,
    0x000000ff, 0x000000ff, 0x000000ff, 0x000000ff,
    0x00000000,
    0x00030201, // BPC: fmax=1, fmin=2, offset=3
    0x000040ee, // BPC: BLS=64, max/min shift=14
    0x00010000, 0x00000000, 0x00000000,
  });
  dst += write_dmi(dst, &addr, 64*sizeof(uint32_t), 0xc24, 12); // ABF34 bank 0
  patches.push_back(addr - (uint64_t)start);

  // output size/scaling
  dst += write_cont(dst, 0xa3c, {
    0x00000003,
    ((out_width - 1) << 16) | (s->frame_width - 1),
    0x30036666,
    0x00000000,
    0x00000000,
    s->frame_width - 1,
    ((out_height - 1) << 16) | (s->frame_height - 1),
    0x30036666,
    0x00000000,
    0x00000000,
    s->frame_height - 1,
  });
  dst += write_cont(dst, 0xa68, {
    0x00000003,
    ((out_width / 2 - 1) << 16) | (s->frame_width - 1),
    0x3006cccc,
    0x00000000,
    0x00000000,
    s->frame_width - 1,
    ((out_height / 2 - 1) << 16) | (s->frame_height - 1),
    0x3006cccc,
    0x00000000,
    0x00000000,
    s->frame_height - 1,
  });

  // cropping
  dst += write_cont(dst, 0xe10, {
    out_height - 1,
    out_width - 1,
  });
  dst += write_cont(dst, 0xe30, {
    out_height / 2 - 1,
    out_width - 1,
  });
  dst += write_cont(dst, 0xe18, {
    0x0ff00000,
    0x00000016,
  });
  dst += write_cont(dst, 0xe38, {
    0x0ff00000,
    0x00000017,
  });

  dst += build_common_ife_bps(dst, cam, s, patches, true);

  return dst - start;
}
