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
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

#include <glog/logging.h>
#include <openni2/OpenNI.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "constants.h"
#include "types.h"
#include "astra_camera_msgs/msg/extrinsics.hpp"
#include "dynamic_params.h"

namespace astra_camera {

bool operator==(const openni::VideoMode& lhs, const openni::VideoMode& rhs);

bool operator!=(const openni::VideoMode& lhs, const openni::VideoMode& rhs);

std::ostream& operator<<(std::ostream& os, const openni::VideoMode& video_mode);

tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]);

astra_camera_msgs::msg::Extrinsics obExtrinsicsToMsg(const float rotation[9],
                                                     const float transition[3],
                                                     const std::string& frame_id);

sensor_msgs::msg::CameraInfo::UniquePtr getDefaultCameraInfo(int width, int height, double f);

bool isValidCameraParams(const OBCameraParams& params);

std::vector<std::string> split(const std::string& str, char delim);

template <class T>
inline void setAndGetNodeParameter(
    const std::shared_ptr<Parameters>& parameters, T& param, const std::string& param_name,
    const T& default_value,
    const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor =
        rcl_interfaces::msg::ParameterDescriptor()) {
  try {
    param = parameters
                ->setParam(param_name, rclcpp::ParameterValue(default_value),
                           std::function<void(const rclcpp::Parameter&)>(), parameter_descriptor)
                .get<T>();
  } catch (const rclcpp::ParameterTypeException& ex) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("astra_camera_utils"),
                        "Failed to set parameter: " << param_name << ". " << ex.what());
  }
}

rmw_qos_profile_t qosFromString(const std::string& str_qos);
}  // namespace astra_camera
