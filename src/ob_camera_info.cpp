/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2022 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/
#include "astra_camera/ob_camera_node.h"

namespace astra_camera {

OBCameraParams OBCameraNode::getCameraParams() {
  if (camera_params_) {
    return camera_params_.value();
  }
  auto pid = device_info_.getUsbProductId();
  if (pid != DABAI_DCW_DEPTH_PID && pid != DABAI_DW_PID) {
    OBCameraParams params;
    int data_size = sizeof(OBCameraParams);
    device_->getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t*)&params, &data_size);
    camera_params_ = params;
    return params;
  } else {
    OBCameraParamsData camera_params_data;
    int data_size = sizeof(camera_params_data);
    memset(&camera_params_data, 0, data_size);
    openni::VideoMode video_mode;
    if (stream_video_mode_.count(DEPTH)) {
      video_mode = stream_video_mode_[DEPTH];
    } else if (stream_video_mode_.count(COLOR)) {
      video_mode = stream_video_mode_[COLOR];
    } else if (stream_video_mode_.count(INFRA1)) {
      video_mode = stream_video_mode_[INFRA1];
    } else {
      ROS_ERROR_STREAM(
          "Have not found property video mode,read default avoid crash, but still have problem");
      OBCameraParams params;
      data_size = sizeof(OBCameraParams);
      device_->getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t*)&params, &data_size);
      camera_params_ = params;
      return params;
    }
    if (video_mode.getResolutionX() == 1024 && video_mode.getResolutionY() == 768) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_1024_768;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_DEFAULT;
    } else if (video_mode.getResolutionX() == 512 && video_mode.getResolutionY() == 384) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_512_384;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_DEFAULT;
    } else if (video_mode.getResolutionX() == 640 && video_mode.getResolutionY() == 480) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_640_480;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_DEFAULT;
    } else if (video_mode.getResolutionX() == 320 && video_mode.getResolutionY() == 240) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_320_240;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_DEFAULT;
    } else if (video_mode.getResolutionX() == 160 && video_mode.getResolutionY() == 120) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_160_120;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_DEFAULT;
    } else if (video_mode.getResolutionX() == 480 && video_mode.getResolutionY() == 360) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_480_360;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_DEFAULT;
    } else if (video_mode.getResolutionX() == 240 && video_mode.getResolutionY() == 180) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_240_180;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_DEFAULT;
    } else if (video_mode.getResolutionX() == 640 && video_mode.getResolutionY() == 360 &&
               pid == DABAI_DCW_DEPTH_PID) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_640_360;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_640_360;
    } else if (video_mode.getResolutionX() == 320 && video_mode.getResolutionY() == 180 &&
               pid == DABAI_DCW_DEPTH_PID) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_320_180;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_320_180;
    } else {
      ROS_WARN_STREAM("dose not match  resolution: " << video_mode.getResolutionX() << "x"
                                                     << video_mode.getResolutionY());
      OBCameraParams params;
      data_size = sizeof(OBCameraParams);
      device_->getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t*)&params, &data_size);
      camera_params_ = params;
      return params;
    }
    device_->getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t*)&camera_params_data,
                         &data_size);
    camera_params_ = camera_params_data.params;
    return camera_params_data.params;
  }
}

sensor_msgs::CameraInfo OBCameraNode::OBCameraParamsToCameraInfo(const OBCameraParams& params) {
  sensor_msgs::CameraInfo camera_info;
  camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  camera_info.D.resize(5, 0.0);
  camera_info.D[0] = params.r_k[0];
  camera_info.D[1] = params.r_k[1];
  camera_info.D[2] = params.r_k[3];
  camera_info.D[3] = params.r_k[4];
  camera_info.D[4] = params.r_k[2];

  camera_info.K.fill(0.0);
  camera_info.K[0] = params.r_intr_p[0];
  camera_info.K[2] = params.r_intr_p[2];
  camera_info.K[4] = params.r_intr_p[1];
  camera_info.K[5] = params.r_intr_p[3];
  camera_info.K[8] = 1.0;

  camera_info.R.fill(0.0);
  for (int i = 0; i < 9; i++) {
    camera_info.R[i] = params.r2l_r[i];
  }

  camera_info.P.fill(0.0);
  camera_info.P[0] = camera_info.K[0];
  camera_info.P[2] = camera_info.K[2];
  camera_info.P[3] = params.r2l_t[0];
  camera_info.P[5] = camera_info.K[4];
  camera_info.P[6] = camera_info.K[5];
  camera_info.P[7] = params.r2l_t[1];
  camera_info.P[10] = 1.0;
  camera_info.P[11] = params.r2l_t[2];
  return camera_info;
}

double OBCameraNode::getFocalLength(const stream_index_pair& stream_index, int y_resolution) {
  CHECK(streams_.count(stream_index));
  auto stream = streams_.at(stream_index);
  CHECK_NOTNULL(stream.get());
  return static_cast<double>(y_resolution) / (2 * tan(stream->getVerticalFieldOfView() / 2));
}

sensor_msgs::CameraInfo OBCameraNode::getIRCameraInfo() {
  int width = width_[INFRA1];
  int height = height_[INFRA1];

  if (ir_camera_info_->isCalibrated()) {
    auto camera_info = ir_camera_info_->getCameraInfo();
    if (camera_info.width != static_cast<uint32_t>(width) || static_cast<uint32_t>(height)) {
      ROS_WARN_ONCE(
          "Image resolution doesn't match calibration of the RGB camera. Using default "
          "parameters.");
      double depth_focal_length = getFocalLength(DEPTH, height);
      return getDefaultCameraInfo(width, height, depth_focal_length);
    } else {
      return camera_info;
    }
  }

  double depth_focal_length = getFocalLength(DEPTH, height);
  auto camera_info = getDefaultCameraInfo(width, height, depth_focal_length);
  auto camera_params = getCameraParams();
  if (isValidCameraParams(camera_params)) {
    camera_info.K.fill(0.0);

    camera_info.K.fill(0.0);
    camera_info.K[0] = camera_params.r_intr_p[0];
    camera_info.K[2] = camera_params.r_intr_p[2];
    camera_info.K[4] = camera_params.r_intr_p[1];
    camera_info.K[5] = camera_params.r_intr_p[3];
    camera_info.K[8] = 1.0;
    // left camera is depth camera, right is rgb camera
    if (!depth_align_) {
      // if depth is not registered, then rgb is registered to depth
      camera_info.K[0] = camera_params.l_intr_p[0];
      camera_info.K[2] = camera_params.l_intr_p[2];
      camera_info.K[4] = camera_params.l_intr_p[1];
      camera_info.K[5] = camera_params.l_intr_p[3];
      camera_info.K[8] = 1.0;
    } else {
      camera_info.K[0] = camera_params.r_intr_p[0];
      camera_info.K[2] = camera_params.r_intr_p[2];
      camera_info.K[4] = camera_params.r_intr_p[1];
      camera_info.K[5] = camera_params.r_intr_p[3];
      camera_info.K[8] = 1.0;
    }

    camera_info.R.fill(0.0);
    camera_info.R[0] = 1.0;
    camera_info.R[4] = 1.0;
    camera_info.R[8] = 1.0;

    camera_info.P.fill(0.0);
    camera_info.P[0] = camera_info.K[0];
    camera_info.P[2] = camera_info.K[2];
    camera_info.P[5] = camera_info.K[4];
    camera_info.P[6] = camera_info.K[5];
    camera_info.P[10] = 1.0;
    if (device_info_.getUsbProductId() != DABAI_DCW_DEPTH_PID &&
        device_info_.getUsbProductId() != DABAI_DW_PID) {
      /* 02122020 Scale IR Params */
      double scaling = static_cast<double>(width) / 640;
      camera_info.K[0] *= scaling;  // fx
      camera_info.K[2] *= scaling;  // cx
      camera_info.K[4] *= scaling;  // fy
      camera_info.K[5] *= scaling;  // cy
      camera_info.P[0] *= scaling;  // fx
      camera_info.P[2] *= scaling;  // cx
      camera_info.P[5] *= scaling;  // fy
      camera_info.P[6] *= scaling;  // cy

      /* 02122020 end */
    }
  }
  return camera_info;
}

sensor_msgs::CameraInfo OBCameraNode::getDepthCameraInfo() {
  // The depth image has essentially the same intrinsics as the IR image, BUT the
  // principal point is offset by half the size of the hardware correlation window
  // (probably 9x9 or 9x7 in 640x480 mode). See
  // http://www.ros.org/wiki/kinect_calibration/technical
  int width = width_[DEPTH];
  double scaling = (double)width / 640;
  auto camera_info = getIRCameraInfo();
  camera_info.K[2] -= depth_ir_x_offset_ * scaling;
  camera_info.K[5] -= depth_ir_y_offset_ * scaling;
  camera_info.K[2] -= depth_ir_x_offset_ * scaling;
  camera_info.K[6] -= depth_ir_y_offset_ * scaling;

  // TODO: Could put this in projector frame so as to encode the baseline in P[3]
  return camera_info;
}

sensor_msgs::CameraInfo OBCameraNode::getColorCameraInfo() {
  int width = width_[COLOR];
  int height = height_[COLOR];
  if (color_camera_info_->isCalibrated()) {
    auto camera_info = color_camera_info_->getCameraInfo();
    if (camera_info.width != static_cast<uint32_t>(width) || static_cast<uint32_t>(height)) {
      ROS_WARN_ONCE(
          "Image resolution doesn't match calibration of the RGB camera. Using default "
          "parameters.");
      double color_focal_length = getFocalLength(COLOR, height);
      return getDefaultCameraInfo(width, height, color_focal_length);
    } else {
      return camera_info;
    }
  }
  // If uncalibrated, fill in default values
  auto camera_params = getCameraParams();
  if (isValidCameraParams(camera_params)) {
    auto default_camera_info = OBCameraParamsToCameraInfo(camera_params);
    sensor_msgs::CameraInfo camera_info;
    camera_info.D.resize(5, 0.0);
    camera_info.K.fill(0.0);
    camera_info.R.fill(0.0);
    camera_info.P.fill(0.0);
    camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    camera_info.width = width;
    camera_info.height = height;

    for (int i = 0; i < 9; i++) {
      camera_info.K[i] = default_camera_info.K[i];
      camera_info.R[i] = default_camera_info.R[i];
    }

    for (int i = 0; i < 12; i++) {
      camera_info.P[i] = default_camera_info.P[i];
    }
    if (device_info_.getUsbProductId() != DABAI_DCW_DEPTH_PID) {
      /*02112020 color camera param change according to resolution */
      double scaling = (double)width / 640;
      camera_info.K[0] *= scaling;  // fx
      camera_info.K[2] *= scaling;  // cx
      camera_info.K[4] *= scaling;  // fy
      camera_info.K[5] *= scaling;  // cy
      camera_info.P[0] *= scaling;  // fx
      camera_info.P[2] *= scaling;  // cx
      camera_info.P[5] *= scaling;  // fy
      camera_info.P[6] *= scaling;  // cy

      /* 02112020 end*/
    }
    return camera_info;
    // end if (isValidCameraParams(camera_params))
  } else {
    double color_focal_length = getFocalLength(COLOR, height);
    return getDefaultCameraInfo(width, height, color_focal_length);
  }
}
sensor_msgs::CameraInfo OBCameraNode::getDefaultCameraInfo(int width, int height, double f) {
  sensor_msgs::CameraInfo info;

  info.width = width;
  info.height = height;

  // No distortion
  info.D.resize(5, 0.0);
  info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // Simple camera matrix: square pixels (fx = fy), principal point at center
  info.K.fill(0.0);
  info.K[0] = info.K[4] = f;

  info.K[2] = (static_cast<double>(width) / 2) - 0.5;
  // Aspect ratio for the camera center on Astra (and other devices?) is 4/3
  // This formula keeps the principal point the same in VGA and SXGA modes
  info.K[5] = (static_cast<double>(width) * (3. / 8.)) - 0.5;
  info.K[8] = 1.0;

  // No separate rectified image plane, so R = I
  info.R[0] = info.R[4] = info.R[8] = 1.0;

  // Then P=K(I|0) = (K|0)
  info.P.fill(0.0);
  info.P[0] = info.P[5] = f;  // fx, fy
  info.P[2] = info.K[2];      // cx
  info.P[6] = info.K[5];      // cy
  info.P[10] = 1.0;

  return info;
}

}  // namespace astra_camera
