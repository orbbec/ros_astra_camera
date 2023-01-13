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

#pragma once
#include <cstdlib>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <magic_enum.hpp>

#include <openni2/OpenNI.h>

#include "astra_camera_msgs/srv/get_device_info.hpp"
#include "astra_camera_msgs/msg/extrinsics.hpp"
#include "astra_camera_msgs/srv/set_int32.hpp"
#include "astra_camera_msgs/srv/get_int32.hpp"
#include "astra_camera_msgs/srv/get_string.hpp"
#include "astra_camera_msgs/srv/get_camera_info.hpp"
#include "astra_camera_msgs/srv/get_camera_params.hpp"

namespace astra_camera {

using FrameCallbackFunction = std::function<void(const openni::VideoFrameRef& frame)>;
using GetDeviceInfo = astra_camera_msgs::srv::GetDeviceInfo;
using Extrinsics = astra_camera_msgs::msg::Extrinsics;
using SetInt32 = astra_camera_msgs::srv::SetInt32;
using GetInt32 = astra_camera_msgs::srv::GetInt32;
using GetString = astra_camera_msgs::srv::GetString;
using SetBool = std_srvs::srv::SetBool;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using GetCameraInfo = astra_camera_msgs::srv::GetCameraInfo;
using GetCameraParams = astra_camera_msgs::srv::GetCameraParams;

using stream_index_pair = std::pair<openni::SensorType, int>;

const stream_index_pair COLOR{openni::SENSOR_COLOR, 0};
const stream_index_pair DEPTH{openni::SENSOR_DEPTH, 0};
const stream_index_pair INFRA1{openni::SENSOR_IR, 0};
const stream_index_pair INFRA2{openni::SENSOR_IR, 1};

const std::vector<stream_index_pair> IMAGE_STREAMS = {DEPTH, INFRA1, COLOR};

typedef enum {
  RGBResolution4_3 = 0,   // 4:3分辨率 如：640x480
  RGBResolution16_9 = 1,  // 16:9分辨率 如：1920x1280
} RgbResolution;

struct ImageROI {
  int x = -1;
  int y = -1;
  int width = -1;
  int height = -1;
};

}  // namespace astra_camera
