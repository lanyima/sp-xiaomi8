#pragma once

#include <memory>

#include "openpilot/cereal/messaging/messaging.h"
#include "msgq/visionipc/visionipc_server.h"
#include "common/util.h"

#ifdef __APPLE__
#include <OpenCL/cl.h>
#else
#include <CL/cl.h>
#endif


const int VIPC_BUFFER_COUNT = 18;

typedef struct FrameMetadata {
  uint32_t frame_id;
  uint32_t request_id;
  uint64_t timestamp_sof;
  uint64_t timestamp_eof;
  float processing_time;
} FrameMetadata;

class SpectraCamera;

class CameraBuf {
private:
  int frame_buf_count;

public:
  VisionIpcServer *vipc_server;
  VisionStreamType stream_type;

  int cur_buf_idx;
  FrameMetadata cur_frame_data;
  VisionBuf *cur_yuv_buf;
  VisionBuf *cur_camera_buf;
  std::unique_ptr<VisionBuf[]> camera_bufs_raw;
  uint32_t out_img_width, out_img_height;

  CameraBuf() = default;
  ~CameraBuf();
  void init(cl_device_id device_id, cl_context context, SpectraCamera *cam, VisionIpcServer * v, int frame_cnt, VisionStreamType type);
  void sendFrameToVipc();

  // GPU demosaic (RAW10 Bayer → NV12)
  cl_device_id cl_device = nullptr;
  cl_context cl_ctx = nullptr;
  cl_command_queue cl_queue = nullptr;
  cl_program cl_prg = nullptr;
  cl_kernel cl_krnl = nullptr;

  // Raw frame dimensions (may differ from output with out_scale > 1)
  int raw_width = 0;
  int raw_height = 0;
  int raw_stride = 0;

  // Auto White Balance state (Grey World AWB)
  // r_total = combined DPAF correction + WB for R channel (Gb_avg / R_avg)
  // b_gain = WB for B channel (Gb_avg / B_avg)
  float awb_r_total = 1.16f;  // Initial estimate
  float awb_b_gain = 1.00f;   // Initial estimate
  int awb_frame_count = 0;
};

void camerad_thread();
kj::Array<uint8_t> get_raw_frame_image(const CameraBuf *b);
float calculate_exposure_value(const CameraBuf *b, Rect ae_xywh, int x_skip, int y_skip);
int open_v4l_by_name_and_index(const char name[], int index = 0, int flags = O_RDWR | O_NONBLOCK);
