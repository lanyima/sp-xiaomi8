#include "system/camerad/cameras/camera_common.h"

#include <cassert>
#include <cstring>
#include <string>
#include <cstdio>

#include "common/clutil.h"
#include "common/swaglog.h"
#include "system/camerad/cameras/nv12_info.h"
#include "system/camerad/cameras/spectra.h"
#include "media/cam_req_mgr.h"

// GPU kernel: RAW10 Bayer → NV12 with auto AWB + CCM color correction
// - Full 10-bit precision readout
// - r_total = combined DPAF + WB correction for R (auto-computed per frame)
// - b_gain = WB correction for B (auto-computed per frame)
// - Green uses DPAF-compensated Gr + Gb from same Bayer block
// - Mild D65 CCM (30% blend with identity)
// - Matched to Python diagnostic pipeline (verified best quality)

static const char raw10_to_nv12_cl_src[] = R"(

// Read a 10-bit RAW pixel at Bayer position (x, y) with bounds clamping
inline int raw10_pixel(__global const uchar *raw, int raw_stride, int x, int y,
                       int max_x, int max_y) {
    // Clamp to valid range preserving Bayer parity
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= max_x) x = max_x - 2 + (x & 1);  // preserve even/odd
    if (y >= max_y) y = max_y - 2 + (y & 1);
    int row_off = y * raw_stride;
    int group = x >> 2;
    int pos = x & 3;
    int msb = (int)raw[row_off + group * 5 + pos];
    int lsb = ((int)raw[row_off + group * 5 + 4] >> (pos * 2)) & 3;
    return (msb << 2) | lsb;
}

__kernel void raw10_to_nv12(
    __global const uchar *raw,
    __global uchar *yuv,
    const int raw_stride,
    const int yuv_stride,
    const int out_width,
    const int out_height,
    const int raw_y_offset,
    const int uv_offset,
    const float r_total,
    const float b_gain,
    const float dpaf_ratio
) {
    const int bx = get_global_id(0);
    const int by = get_global_id(1);
    if (bx >= out_width / 2 || by >= out_height / 2) return;

    const int ox = bx * 2;
    const int oy = by * 2;

    const int BL = 64;
    const float NORM = 1.0f / 959.0f;

    // Raw dimensions for bounds clamping
    const int raw_W = out_width * 2;
    const int raw_H = out_height * 2;

    // CCM for Y path: 30% D65 (clean edges)
    const float ccm00 =  1.2290f, ccm01 = -0.2135f, ccm02 = -0.0155f;
    const float ccm10 = -0.0535f, ccm11 =  1.1529f, ccm12 = -0.0994f;
    const float ccm20 =  0.0180f, ccm21 = -0.2259f, ccm22 =  1.2079f;

    // CCM for UV path: 70% D65 (vivid color separation)
    const float cc00 =  1.5341f, cc01 = -0.4983f, cc02 = -0.0362f;
    const float cc10 = -0.1247f, cc11 =  1.3569f, cc12 = -0.2319f;
    const float cc20 =  0.0420f, cc21 = -0.5271f, cc22 =  1.4851f;

    int Y[4];
    int sumR_c = 0, sumG_c = 0, sumB_c = 0;  // color path sums for UV

    for (int dy = 0; dy < 2; dy++) {
        for (int dx = 0; dx < 2; dx++) {
            int cx = (ox + dx) * 2;           // Bayer x (even)
            int cy = (oy + dy) * 2 + raw_y_offset;  // Bayer y (even)

            // === R: direct sample from this Bayer block ===
            int R_raw = raw10_pixel(raw, raw_stride, cx, cy, raw_W, raw_H + raw_y_offset);
            float R_lin = max((float)(R_raw - BL), 0.0f) * NORM;

            // === G: same-block Gr+Gb with DPAF compensation ===
            int Gr = raw10_pixel(raw, raw_stride, cx+1, cy,   raw_W, raw_H + raw_y_offset);
            int Gb = raw10_pixel(raw, raw_stride, cx,   cy+1, raw_W, raw_H + raw_y_offset);
            float Gr_f = max((float)(Gr - BL), 0.0f) * dpaf_ratio;
            float Gb_f = max((float)(Gb - BL), 0.0f);
            float G_lin = (Gr_f + Gb_f) * 0.5f * NORM;

            // === B: direct sample ===
            int B_raw = raw10_pixel(raw, raw_stride, cx+1, cy+1, raw_W, raw_H + raw_y_offset);
            float B_lin = max((float)(B_raw - BL), 0.0f) * NORM;

            // Apply AWB gains
            R_lin *= r_total;
            B_lin *= b_gain;

            // === Y PATH: mild CCM + gamma + S-curve (clean edges) ===
            float Ry = ccm00 * R_lin + ccm01 * G_lin + ccm02 * B_lin;
            float Gy = ccm10 * R_lin + ccm11 * G_lin + ccm12 * B_lin;
            float By = ccm20 * R_lin + ccm21 * G_lin + ccm22 * B_lin;
            Ry = native_powr(clamp(Ry, 1e-6f, 1.0f), 1.0f / 2.2f);
            Gy = native_powr(clamp(Gy, 1e-6f, 1.0f), 1.0f / 2.2f);
            By = native_powr(clamp(By, 1e-6f, 1.0f), 1.0f / 2.2f);
            Ry = Ry * Ry * (3.0f - 2.0f * Ry);
            Gy = Gy * Gy * (3.0f - 2.0f * Gy);
            By = By * By * (3.0f - 2.0f * By);
            int Ri_y = (int)(Ry * 255.0f);
            int Gi_y = (int)(Gy * 255.0f);
            int Bi_y = (int)(By * 255.0f);
            Y[dy * 2 + dx] = clamp((77 * Ri_y + 150 * Gi_y + 29 * Bi_y) >> 8, 0, 255);

            // === UV PATH: strong CCM + gamma + S-curve + saturation (vivid color) ===
            float Rc = cc00 * R_lin + cc01 * G_lin + cc02 * B_lin;
            float Gc = cc10 * R_lin + cc11 * G_lin + cc12 * B_lin;
            float Bc = cc20 * R_lin + cc21 * G_lin + cc22 * B_lin;
            Rc = native_powr(clamp(Rc, 1e-6f, 1.0f), 1.0f / 2.2f);
            Gc = native_powr(clamp(Gc, 1e-6f, 1.0f), 1.0f / 2.2f);
            Bc = native_powr(clamp(Bc, 1e-6f, 1.0f), 1.0f / 2.2f);
            Rc = Rc * Rc * (3.0f - 2.0f * Rc);
            Gc = Gc * Gc * (3.0f - 2.0f * Gc);
            Bc = Bc * Bc * (3.0f - 2.0f * Bc);
            // 1.3x saturation boost
            float luma_c = 0.299f * Rc + 0.587f * Gc + 0.114f * Bc;
            Rc = clamp(luma_c + 1.3f * (Rc - luma_c), 0.0f, 1.0f);
            Gc = clamp(luma_c + 1.3f * (Gc - luma_c), 0.0f, 1.0f);
            Bc = clamp(luma_c + 1.3f * (Bc - luma_c), 0.0f, 1.0f);
            sumR_c += (int)(Rc * 255.0f);
            sumG_c += (int)(Gc * 255.0f);
            sumB_c += (int)(Bc * 255.0f);
        }
    }

    yuv[oy * yuv_stride + ox]             = (uchar)Y[0];
    yuv[oy * yuv_stride + ox + 1]         = (uchar)Y[1];
    yuv[(oy + 1) * yuv_stride + ox]       = (uchar)Y[2];
    yuv[(oy + 1) * yuv_stride + ox + 1]   = (uchar)Y[3];

    int avgR = sumR_c / 4, avgG = sumG_c / 4, avgB = sumB_c / 4;
    int U = clamp(((-43 * avgR - 85 * avgG + 128 * avgB) >> 8) + 128, 0, 255);
    int V = clamp(((128 * avgR - 107 * avgG - 21 * avgB) >> 8) + 128, 0, 255);
    yuv[uv_offset + by * yuv_stride + ox]     = (uchar)U;
    yuv[uv_offset + by * yuv_stride + ox + 1] = (uchar)V;
}
)";


void CameraBuf::init(cl_device_id device_id, cl_context context, SpectraCamera *cam, VisionIpcServer * v, int frame_cnt, VisionStreamType type) {
  vipc_server = v;
  stream_type = type;
  frame_buf_count = frame_cnt;

  const SensorInfo *sensor = cam->sensor.get();

  // RAW frames from ISP
  if (cam->cc.output_type != ISP_IFE_PROCESSED) {
    camera_bufs_raw = std::make_unique<VisionBuf[]>(frame_buf_count);

    const int raw_frame_size = (sensor->frame_height + sensor->extra_height) * sensor->frame_stride;
    for (int i = 0; i < frame_buf_count; i++) {
      camera_bufs_raw[i].allocate(raw_frame_size);
      // xiaomi8: VisionBuf has no init_cl in new API; CL buffers created on-demand below
    }
    LOGD("allocated %d CL buffers", frame_buf_count);
  }

  // Keep the advertised VisionIPC dimensions and backing allocation in lockstep.
  // The old encoder-derived constants happened to cover 1928x1208, but allocate
  // only 4,804,608 bytes for a 2016x1512 stream. modeld correctly computes the
  // 2016x1512 Venus size (5,787,648 bytes), so the mismatch caused a delayed
  // GPU/CPU out-of-bounds read and modeld SIGSEGV. Use the same Venus helper as
  // SpectraCamera::camera_open() for every geometry.
  const auto [nv12_stride, nv12_y_height, nv12_uv_height, nv12_size] =
    get_nv12_info(out_img_width, out_img_height);
  assert(nv12_stride == cam->stride);
  assert(nv12_y_height == cam->y_height);
  assert(nv12_uv_height == cam->uv_height);

  vipc_server->create_buffers_with_sizes(stream_type, VIPC_BUFFER_COUNT, out_img_width, out_img_height, nv12_size, cam->stride, cam->uv_offset);
  LOGD("created %d YUV vipc buffers with size %dx%d", VIPC_BUFFER_COUNT, cam->stride, cam->y_height);

  // Initialize GPU demosaic kernel (RAW10 Bayer → NV12)
  if (camera_bufs_raw) {
    raw_width = sensor->frame_width;
    raw_height = sensor->frame_height;
    raw_stride = sensor->frame_stride;

    cl_device = device_id;
    cl_ctx = context;
    const cl_queue_properties qprops[] = {0};
    cl_queue = CL_CHECK_ERR(clCreateCommandQueueWithProperties(cl_ctx, cl_device, qprops, &err));
    cl_prg = cl_program_from_source(cl_ctx, cl_device, raw10_to_nv12_cl_src, nullptr);
    cl_krnl = CL_CHECK_ERR(clCreateKernel(cl_prg, "raw10_to_nv12", &err));
    LOGE("GPU demosaic kernel compiled OK (raw %dx%d stride=%d -> out %dx%d)",
         raw_width, raw_height, raw_stride, out_img_width, out_img_height);
  }
}

CameraBuf::~CameraBuf() {
  if (cl_krnl) clReleaseKernel(cl_krnl);
  if (cl_prg) clReleaseProgram(cl_prg);
  if (cl_queue) clReleaseCommandQueue(cl_queue);
  if (camera_bufs_raw != nullptr) {
    for (int i = 0; i < frame_buf_count; i++) {
      camera_bufs_raw[i].free();
    }
  }
}

void CameraBuf::sendFrameToVipc() {
  assert(cur_buf_idx >=0 && cur_buf_idx < frame_buf_count);

  if (camera_bufs_raw) {
    cur_camera_buf = &camera_bufs_raw[cur_buf_idx];
  }

  cur_yuv_buf = vipc_server->get_buffer(stream_type, cur_buf_idx);

  // RAW10 Bayer → NV12 conversion via GPU (OpenCL)
  if (cur_camera_buf && cur_yuv_buf && cur_camera_buf->addr && cur_yuv_buf->addr && cl_krnl) {
    static int raw_frame_count = 0;
    static float s_dpaf_ratio = 1.126f;  // DPAF Gb/Gr ratio, updated per-frame

    int width = cur_yuv_buf->width;
    int height = cur_yuv_buf->height;
    int rs = this->raw_stride;  // Use stored raw stride (4800 for 3840-wide RAW10)
    int yuv_stride = cur_yuv_buf->stride;
    int total_raw_lines = (int)(cur_camera_buf->len / rs);
    int raw_y_offset = (total_raw_lines > raw_height) ? (total_raw_lines - raw_height) / 2 : 0;
    int uv_offset = (int)cur_yuv_buf->uv_offset;

    // Use CL buffers (ION zero-copy)
    // xiaomi8: new VisionBuf has no buf_cl; wrap host pointer on the fly
    cl_int _wrap_err = 0;
    cl_mem raw_cl = clCreateBuffer(cl_ctx, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY,
                                   cur_camera_buf->len, cur_camera_buf->addr, &_wrap_err);
    cl_mem yuv_cl = clCreateBuffer(cl_ctx, CL_MEM_USE_HOST_PTR | CL_MEM_WRITE_ONLY,
                                   cur_yuv_buf->len, cur_yuv_buf->addr, &_wrap_err);

    if (raw_cl && yuv_cl) {
      // === Grey World Auto White Balance ===
      // Use DPAF-compensated green average for proper WB ratios
      {
        uint8_t *raw_ptr = (uint8_t *)cur_camera_buf->addr;
        const int AWB_BL = 16;
        const int GRID = 12;
        float sum_r = 0, sum_gr = 0, sum_gb = 0, sum_b = 0;
        int awb_count = 0;
        int y0 = raw_height / 6, y1 = raw_height * 5 / 6;
        int x0 = raw_width / 6, x1 = raw_width * 5 / 6;
        for (int gi = 0; gi < GRID; gi++) {
          int sy = (y0 + (y1 - y0) * gi / (GRID - 1)) & ~1;
          for (int gj = 0; gj < GRID; gj++) {
            int sx = (x0 + (x1 - x0) * gj / (GRID - 1)) & ~1;
            int roff0 = (sy + raw_y_offset) * rs;
            int roff1 = (sy + 1 + raw_y_offset) * rs;
            int px0 = (sx / 4) * 5 + (sx % 4);
            int px1 = ((sx + 1) / 4) * 5 + ((sx + 1) % 4);
            int r  = (int)raw_ptr[roff0 + px0] - AWB_BL; if (r < 0) r = 0;
            int gr = (int)raw_ptr[roff0 + px1] - AWB_BL; if (gr < 0) gr = 0;
            int gb = (int)raw_ptr[roff1 + px0] - AWB_BL; if (gb < 0) gb = 0;
            int b  = (int)raw_ptr[roff1 + px1] - AWB_BL; if (b < 0) b = 0;
            if (gb > 15 && gb < 230 && r > 5) {
              sum_r += r; sum_gr += gr; sum_gb += gb; sum_b += b;
              awb_count++;
            }
          }
        }
        if (awb_count > 20) {
          float avg_r  = sum_r / awb_count;
          float avg_gr_v = sum_gr / awb_count;
          float avg_gb = sum_gb / awb_count;
          float avg_b  = sum_b / awb_count;
          // Compute DPAF ratio (Gb/Gr) for green channel compensation
          float dpaf = (avg_gr_v > 1.0f) ? avg_gb / avg_gr_v : 1.126f;
          if (dpaf < 1.0f) dpaf = 1.0f; if (dpaf > 2.0f) dpaf = 2.0f;
          s_dpaf_ratio = s_dpaf_ratio * 0.95f + dpaf * 0.05f;
          // Use DPAF-compensated green average for WB ratios
          float g_avg = (avg_gr_v * s_dpaf_ratio + avg_gb) * 0.5f;
          float rt = (avg_r > 1.0f) ? g_avg / avg_r : 1.16f;
          float bg = (avg_b > 1.0f) ? g_avg / avg_b : 0.89f;
          if (rt < 0.5f) rt = 0.5f; if (rt > 5.0f) rt = 5.0f;
          if (bg < 0.2f) bg = 0.2f; if (bg > 5.0f) bg = 5.0f;
          // Always update AWB (no freeze period)
          float alpha = (awb_frame_count < 5) ? 0.20f : 0.05f;
          awb_r_total = awb_r_total * (1.0f - alpha) + rt * alpha;
          awb_b_gain = awb_b_gain * (1.0f - alpha) + bg * alpha;
          awb_frame_count++;
        }
      }

      float kr = awb_r_total, kb = awb_b_gain;

      CL_CHECK(clSetKernelArg(cl_krnl, 0, sizeof(cl_mem), &raw_cl));
      CL_CHECK(clSetKernelArg(cl_krnl, 1, sizeof(cl_mem), &yuv_cl));
      CL_CHECK(clSetKernelArg(cl_krnl, 2, sizeof(int), &rs));
      CL_CHECK(clSetKernelArg(cl_krnl, 3, sizeof(int), &yuv_stride));
      CL_CHECK(clSetKernelArg(cl_krnl, 4, sizeof(int), &width));
      CL_CHECK(clSetKernelArg(cl_krnl, 5, sizeof(int), &height));
      CL_CHECK(clSetKernelArg(cl_krnl, 6, sizeof(int), &raw_y_offset));
      CL_CHECK(clSetKernelArg(cl_krnl, 7, sizeof(int), &uv_offset));
      CL_CHECK(clSetKernelArg(cl_krnl, 8, sizeof(float), &kr));
      CL_CHECK(clSetKernelArg(cl_krnl, 9, sizeof(float), &kb));
      float dpaf_r = s_dpaf_ratio;
      CL_CHECK(clSetKernelArg(cl_krnl, 10, sizeof(float), &dpaf_r));

      size_t global_work_size[2] = {(size_t)(width / 2), (size_t)(height / 2)};
      CL_CHECK(clEnqueueNDRangeKernel(cl_queue, cl_krnl, 2, NULL, global_work_size, NULL, 0, NULL, NULL));
      CL_CHECK(clFinish(cl_queue));
      clReleaseMemObject(raw_cl);
      clReleaseMemObject(yuv_cl);

      if (raw_frame_count < 5 || raw_frame_count % 300 == 0) {
        LOGE("AWB frame %d: r_total=%.3f b_gain=%.3f dpaf=%.3f (samples=%d)",
             raw_frame_count, kr, kb, dpaf_r, awb_frame_count);
      }
    } else {
      // Fallback: simple CPU grayscale if no CL buffers
      uint8_t *raw = (uint8_t *)cur_camera_buf->addr;
      uint8_t *yuv = (uint8_t *)cur_yuv_buf->addr;
      for (int y = 0; y < height; y++) {
        uint8_t *raw_row = raw + (y * 2 + raw_y_offset) * rs;
        uint8_t *yuv_row = yuv + y * yuv_stride;
        for (int x = 0; x < width; x++) {
          int rx = x * 2;
          yuv_row[x] = raw_row[(rx / 4) * 5 + (rx % 4)];
        }
      }
      memset(yuv + uv_offset, 128, yuv_stride * height / 2);
    }

    if (raw_frame_count % 300 == 0) {
      LOGE("GPU demosaic: frame %d, raw %dx%d stride=%d -> out %dx%d yuv_stride=%d",
           raw_frame_count, raw_width, raw_height, rs, width, height, yuv_stride);
    }
    raw_frame_count++;
  }

  VisionIpcBufExtra extra = {
    cur_frame_data.frame_id,
    cur_frame_data.timestamp_sof,
    cur_frame_data.timestamp_eof,
  };
  cur_yuv_buf->set_frame_id(cur_frame_data.frame_id);
  vipc_server->send(cur_yuv_buf, &extra);
}

// common functions

kj::Array<uint8_t> get_raw_frame_image(const CameraBuf *b) {
  const uint8_t *dat = (const uint8_t *)b->cur_camera_buf->addr;

  kj::Array<uint8_t> frame_image = kj::heapArray<uint8_t>(b->cur_camera_buf->len);
  uint8_t *resized_dat = frame_image.begin();

  memcpy(resized_dat, dat, b->cur_camera_buf->len);

  return kj::mv(frame_image);
}

float calculate_exposure_value(const CameraBuf *b, Rect ae_xywh, int x_skip, int y_skip) {
  int lum_med;
  uint32_t lum_binning[256] = {0};
  const uint8_t *pix_ptr = b->cur_yuv_buf->y;

  unsigned int lum_total = 0;
  for (int y = ae_xywh.y; y < ae_xywh.y + ae_xywh.h; y += y_skip) {
    for (int x = ae_xywh.x; x < ae_xywh.x + ae_xywh.w; x += x_skip) {
      uint8_t lum = pix_ptr[(y * b->out_img_width) + x];
      lum_binning[lum]++;
      lum_total += 1;
    }
  }

  // Find mean lumimance value
  unsigned int lum_cur = 0;
  for (lum_med = 255; lum_med >= 0; lum_med--) {
    lum_cur += lum_binning[lum_med];

    if (lum_cur >= lum_total / 2) {
      break;
    }
  }

  return lum_med / 256.0;
}

int open_v4l_by_name_and_index(const char name[], int index, int flags) {
  for (int v4l_index = 0; /**/; ++v4l_index) {
    std::string v4l_name = util::read_file(util::string_format("/sys/class/video4linux/v4l-subdev%d/name", v4l_index));
    if (v4l_name.empty()) return -1;
    if (v4l_name.find(name) == 0) {
      if (index == 0) {
        return HANDLE_EINTR(open(util::string_format("/dev/v4l-subdev%d", v4l_index).c_str(), flags));
      }
      index--;
    }
  }
}
