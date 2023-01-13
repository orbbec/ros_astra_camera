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

#include "astra_camera/utils.h"
#include <magic_enum.hpp>

#include <sensor_msgs/distortion_models.hpp>

namespace astra_camera {

bool operator==(const openni::VideoMode& lhs, const openni::VideoMode& rhs) {
  return lhs.getResolutionY() == rhs.getResolutionY() &&
         lhs.getResolutionX() == rhs.getResolutionX() && lhs.getFps() == rhs.getFps() &&
         lhs.getPixelFormat() == rhs.getPixelFormat();
}

bool operator!=(const openni::VideoMode& lhs, const openni::VideoMode& rhs) {
  return !(lhs == rhs);
}

std::ostream& operator<<(std::ostream& os, const openni::VideoMode& video_mode) {
  os << "Resolution :" << video_mode.getResolutionX() << "x" << video_mode.getResolutionY() << "@"
     << video_mode.getFps() << "Hz" << std::endl
     << "format " << magic_enum::enum_name(video_mode.getPixelFormat());
  return os;
}

tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]) {
  Eigen::Matrix3f m;
  // We need to be careful about the order, as RS2 rotation matrix is
  // column-major, while Eigen::Matrix3f expects row-major.
  m << rotation[0], rotation[3], rotation[6], rotation[1], rotation[4], rotation[7], rotation[2],
      rotation[5], rotation[8];
  Eigen::Quaternionf q(m);
  return {q.x(), q.y(), q.z(), q.w()};
}

astra_camera_msgs::msg::Extrinsics obExtrinsicsToMsg(const float rotation[9],
                                                     const float transition[3],
                                                     const std::string& frame_id) {
  astra_camera_msgs::msg::Extrinsics msg;
  for (int i = 0; i < 9; ++i) {
    msg.rotation[i] = rotation[i];
    if (i < 3) {
      msg.translation[i] = transition[i];
    }
  }

  msg.header.frame_id = frame_id;
  return msg;
}

sensor_msgs::msg::CameraInfo::UniquePtr getDefaultCameraInfo(int width, int height, double f) {
  auto info = std::make_unique<sensor_msgs::msg::CameraInfo>();

  info->width = width;
  info->height = height;

  // No distortion
  info->d.resize(5, 0.0);
  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // Simple camera matrix: square pixels (fx = fy), principal point at center
  info->k.fill(0.0);
  info->k[0] = info->k[4] = f;

  info->k[2] = (static_cast<double>(width) / 2) - 0.5;
  // Aspect ratio for the camera center on Astra (and other devices?) is 4/3
  // This formula keeps the principal point the same in VGA and SXGA modes
  info->k[5] = (static_cast<double>(width) * (3. / 8.)) - 0.5;
  info->k[8] = 1.0;

  // No separate rectified image plane, so R = I
  info->r[0] = info->r[4] = info->r[8] = 1.0;

  // Then P=K(I|0) = (K|0)
  info->p.fill(0.0);
  info->p[0] = info->p[5] = f;  // fx, fy
  info->p[2] = info->k[2];      // cx
  info->p[6] = info->k[5];      // cy
  info->p[10] = 1.0;

  return info;
}

bool isValidCameraParams(const OBCameraParams& params) {
  if (std::isnan(params.l_intr_p[0]) || std::isnan(params.l_intr_p[1]) ||
      std::isnan(params.l_intr_p[2]) || std::isnan(params.l_intr_p[3])) {
    return false;
  }
  return true;
}

std::vector<std::string> split(const std::string& str, char delim) {
  // https://stackoverflow.com/questions/16749069/c-split-string-by-regex
  std::stringstream ss(str);
  std::vector<std::string> elems;
  std::string item;
  while (std::getline(ss, item, delim)) {
    if (item.length() > 0) {
      elems.push_back(item);
    }
  }
  return elems;
}
rmw_qos_profile_t qosFromString(const std::string& str_qos) {
  if (str_qos == "SYSTEM_DEFAULT") {
    return rmw_qos_profile_system_default;
  } else if (str_qos == "DEFAULT") {
    return rmw_qos_profile_default;
  } else if (str_qos == "PARAMETER_EVENTS") {
    return rmw_qos_profile_parameter_events;
  } else if (str_qos == "SERVICES_DEFAULT") {
    return rmw_qos_profile_services_default;
  } else if (str_qos == "PARAMETERS") {
    return rmw_qos_profile_parameters;
  } else if (str_qos == "SENSOR_DATA") {
    return rmw_qos_profile_sensor_data;
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("astra_camera"),
                        "Invalid QoS profile: " << str_qos << ". Using default QoS profile.");
    return rmw_qos_profile_default;
  }
}

}  // namespace astra_camera
