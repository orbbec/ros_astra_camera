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
#include <dynamic_reconfigure/server.h>
#include <openni2/OpenNI.h>
#include <ros/ros.h>
#include <semaphore.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <atomic>
#include <cerrno>
#include <thread>

#include "ob_camera_node.h"
#include "ob_context.h"
#include "std_msgs/Empty.h"
#include "uvc_camera_driver.h"

namespace astra_camera {
class OBCameraNodeFactory {
 public:
  explicit OBCameraNodeFactory(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  ~OBCameraNodeFactory();

 private:
  void init();

  void queryDeviceThread();

  void resetPtrWithTimeout();

  void startDevice();

  void onDeviceConnected(const openni::DeviceInfo* device_info);

  void onDeviceDisconnected(const openni::DeviceInfo* device_info);

  void checkConnectionTimer();

  static OniLogSeverity getLogLevelFromString(const std::string& level);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::atomic_bool is_alive_{false};
  std::shared_ptr<OBCameraNode> ob_camera_node_ = nullptr;
  std::shared_ptr<openni::Device> device_ = nullptr;
  std::shared_ptr<UVCCameraDriver> uvc_camera_driver_ = nullptr;
  std::shared_ptr<openni::DeviceInfo> device_info_ = nullptr;
  std::shared_ptr<dynamic_reconfigure::Reconfigure> reconfigure_ = nullptr;
  bool use_uvc_camera_ = false;
  UVCCameraConfig uvc_config_;
  std::shared_ptr<OBContext> context_ = nullptr;
  std::string serial_number_;
  std::string device_type_;
  std::string device_uri_;
  ros::WallTimer check_connection_timer_;
  std::atomic_bool device_connected_{false};
  size_t device_num_ = 1;
  std::shared_ptr<std::thread> device_listener_thread_ = nullptr;
  std::string oni_log_level_str_ = "none";
  bool oni_log_to_console_ = false;
  bool oni_log_to_file_ = false;
  ros::Subscriber reset_device_sub_;
  std::recursive_mutex device_lock_;
  std::shared_ptr<std::thread> query_device_thread_ = nullptr;
};
}  // namespace astra_camera
