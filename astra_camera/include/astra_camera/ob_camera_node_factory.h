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
#include <atomic>
#include <thread>

#include <magic_enum.hpp>

#include <rclcpp/rclcpp.hpp>

#include <openni2/OpenNI.h>
#include "dynamic_params.h"
#include "ob_camera_node.h"
#include "uvc_camera_driver.h"
#include "ob_context.h"

namespace astra_camera {
class OBCameraNodeFactory : public rclcpp::Node {
 public:
  explicit OBCameraNodeFactory(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

  [[maybe_unused]] OBCameraNodeFactory(
      const std::string& node_name, const std::string& ns,
      const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

  ~OBCameraNodeFactory() override;

  static void cleanUpSharedMemory();

 private:
  void init();

  void queryDeviceThread();

  void startDevice(const std::shared_ptr<openni::Device>& device,
                   const openni::DeviceInfo* device_info);

  void onDeviceConnected(const openni::DeviceInfo* device_info);

  void onDeviceDisconnected(const openni::DeviceInfo* device_info);

  void checkConnectionTimer();

 private:
  rclcpp::Logger logger_;
  std::atomic_bool is_alive_{false};
  std::unique_ptr<OBCameraNode> ob_camera_node_ = nullptr;
  std::shared_ptr<openni::Device> device_ = nullptr;
  std::shared_ptr<Parameters> parameters_ = nullptr;
  std::shared_ptr<UVCCameraDriver> uvc_camera_driver_ = nullptr;
  std::shared_ptr<openni::DeviceInfo> device_info_ = nullptr;
  bool use_uvc_camera_ = false;
  UVCCameraConfig uvc_config_;
  std::unique_ptr<OBContext> ob_context_ = nullptr;
  std::string serial_number_;
  std::string device_type_;
  std::string device_uri_;
  rclcpp::TimerBase::SharedPtr check_connection_timer_;
  std::atomic_bool device_connected_{false};
  size_t device_num_ = 1;
  long connection_delay_ = 0;
  bool oni_log_to_console_ = false;
  bool oni_log_to_file_ = false;
  std::recursive_mutex device_lock_;
  std::shared_ptr<std::thread> query_device_thread_ = nullptr;
};

}  // namespace astra_camera

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(astra_camera::OBCameraNodeFactory)
