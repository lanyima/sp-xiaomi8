#pragma once

#include "common/util.h"
#include "openpilot/cereal/gen/cpp/log.capnp.h"
#include "openpilot/cereal/visionstream.h"
#include "msgq/visionipc/visionipc_server.h"

#include "media/cam_isp_ife.h"


typedef enum {
  ISP_RAW_OUTPUT,   // raw frame from sensor
  ISP_IFE_PROCESSED,  // fully processed image through the IFE
  ISP_BPS_PROCESSED,  // fully processed image through the BPS
} SpectraOutputType;

// For the Xiaomi Mi 8 (dipper) / comma 3X platform

struct CameraConfig {
  int camera_num;
  VisionStreamType stream_type;
  float focal_len;  // millimeters
  const char *publish_name;
  cereal::FrameData::Builder (cereal::Event::Builder::*init_camera_state)();
  bool enabled;
  uint32_t phy;
  bool vignetting_correction;
  SpectraOutputType output_type;
};

// Xiaomi Mi 8: camera_num 0 = IMX363 rear main (CSIPHY 0)
// Use as ROAD camera for driving
const CameraConfig ROAD_CAMERA_CONFIG = {
  .camera_num = 0,
  .stream_type = VISION_STREAM_ROAD,
  .focal_len = 4.44,
  .publish_name = "roadCameraState",
  .init_camera_state = &cereal::Event::Builder::initRoadCameraState,
  .enabled = !getenv("DISABLE_ROAD"),
  .phy = CAM_ISP_IFE_IN_RES_PHY_0,
  .vignetting_correction = true,
  .output_type = ISP_IFE_PROCESSED,  // Use hardware ISP for demosaic+color+gamma
};

// Also publish as WIDE_ROAD using the same camera for model compatibility
const CameraConfig WIDE_ROAD_CAMERA_CONFIG = {
  .camera_num = 0,
  .stream_type = VISION_STREAM_WIDE_ROAD,
  .focal_len = 4.44,
  .publish_name = "wideRoadCameraState",
  .init_camera_state = &cereal::Event::Builder::initWideRoadCameraState,
  .enabled = false,  // disabled - single rear camera only
  .phy = CAM_ISP_IFE_IN_RES_PHY_0,
  .vignetting_correction = false,
  .output_type = ISP_IFE_PROCESSED,
};

// Xiaomi Mi 8: camera_num 2 = IMX576 front camera (CSIPHY 2)
// Use as DRIVER camera for driver monitoring
const CameraConfig DRIVER_CAMERA_CONFIG = {
  .camera_num = 2,
  .stream_type = VISION_STREAM_DRIVER,
  .focal_len = 3.67,
  .publish_name = "driverCameraState",
  .init_camera_state = &cereal::Event::Builder::initDriverCameraState,
  .enabled = false,  // disabled for now - IMX576 driver not yet implemented
  .phy = CAM_ISP_IFE_IN_RES_PHY_2,
  .vignetting_correction = false,
  .output_type = ISP_BPS_PROCESSED,
};

const CameraConfig ALL_CAMERA_CONFIGS[] = {ROAD_CAMERA_CONFIG, WIDE_ROAD_CAMERA_CONFIG, DRIVER_CAMERA_CONFIG};
