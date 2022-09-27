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

#include "astra_camera/ob_camera_node_factory.h"

#include <fcntl.h>

#include <boost/filesystem.hpp>

#include "astra_camera/ob_camera_node.h"

namespace astra_camera {

OBCameraNodeFactory::OBCameraNodeFactory(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  ROS_INFO_STREAM("OBCameraNodeFactory::OBCameraNodeFactory");
  init();
}

OBCameraNodeFactory::~OBCameraNodeFactory() {
  sem_unlink(DEFAULT_SEM_NAME.c_str());
  openni::OpenNI::shutdown();
}

void OBCameraNodeFactory::initOpenNI() {
  auto log_level = getLogLevelFromString(oni_log_level_str_);
  openni::OpenNI::setLogMinSeverity(log_level);
  openni::OpenNI::setLogConsoleOutput(oni_log_to_console_);
  openni::OpenNI::setLogFileOutput(oni_log_to_file_);
  auto rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK) {
    ROS_ERROR("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());
    exit(-1);
  }
}

void OBCameraNodeFactory::initDeviceListener() {
  auto connected_cb = [this](const openni::DeviceInfo* device_info) {
    this->onDeviceConnected(device_info);
  };
  auto disconnected_cb = [this](const openni::DeviceInfo* device_info) {
    this->onDeviceDisconnected(device_info);
  };
  if (device_listener_) {
    device_listener_.reset();
  }
  device_listener_ = std::make_unique<DeviceListener>(connected_cb, disconnected_cb);
}

void OBCameraNodeFactory::init() {
  ROS_INFO_STREAM("Initializing OBCameraNodeFactory...");
  if (boost::filesystem::exists("/dev/shm/sem." + DEFAULT_SEM_NAME)) {
    sem_unlink(DEFAULT_SEM_NAME.c_str());
  }
  oni_log_level_str_ = nh_private_.param<std::string>("oni_log_level", "info");
  oni_log_to_console_ = nh_private_.param<bool>("oni_log_to_console", false);
  oni_log_to_file_ = nh_private_.param<bool>("oni_log_to_file", false);
  initOpenNI();
  use_uvc_camera_ = nh_private_.param<bool>("use_uvc_camera", false);
  serial_number_ = nh_private_.param<std::string>("serial_number", "");
  number_of_devices_ = nh_private_.param<int>("number_of_devices", 1);
  reconnection_delay_ = nh_private_.param<int>("reconnection_delay", 1);
  initDeviceListener();
  check_connection_timer_ = nh_.createWallTimer(
      ros::WallDuration(1.0), [this](const ros::WallTimerEvent&) { this->checkConnectionTimer(); });
  reset_device_sub_ =
      nh_.subscribe("reset_device", 1, &OBCameraNodeFactory::resetDeviceCallback, this);
  ROS_INFO_STREAM("init Done");
}

void OBCameraNodeFactory::startDevice() {
  ROS_INFO("Start device");
  if (ob_camera_node_) {
    ob_camera_node_.reset();
  }
  CHECK_NOTNULL(device_.get());
  if (use_uvc_camera_) {
    if (uvc_camera_driver_) {
      uvc_camera_driver_.reset();
    }
    uvc_camera_driver_ = std::make_shared<UVCCameraDriver>(nh_, nh_private_, serial_number_);
    ob_camera_node_ = std::make_unique<OBCameraNode>(nh_, nh_private_, device_, uvc_camera_driver_);
  } else {
    ob_camera_node_ = std::make_unique<OBCameraNode>(nh_, nh_private_, device_);
  }
  device_connected_ = true;
}

void OBCameraNodeFactory::onDeviceConnected(const openni::DeviceInfo* device_info) {
  ROS_INFO_STREAM("Device connected: " << device_info->getName());
  if (device_info->getUri() == nullptr) {
    ROS_ERROR_STREAM("Device connected: " << device_info->getName() << " uri is null");
    return;
  }
  auto device_sem = sem_open(DEFAULT_SEM_NAME.c_str(), O_CREAT, 0644, 1);
  if (device_sem == (void*)SEM_FAILED) {
    ROS_ERROR("Failed to create semaphore");
    return;
  }
  ROS_INFO_STREAM("Waiting for device to be ready");
  int ret = sem_wait(device_sem);
  if (!ret && !connected_devices_.count(device_info->getUri())) {
    auto device = std::make_shared<openni::Device>();
    ROS_INFO_STREAM("Trying to open device: " << device_info->getUri());
    std::this_thread::sleep_for(std::chrono::seconds(reconnection_delay_));
    auto rc = device->open(device_info->getUri());
    if (rc != openni::STATUS_OK) {
      if (errno == EBUSY) {
        connected_devices_[device_info->getUri()] = *device_info;
      }
    } else {
      char serial_number[64];
      int data_size = sizeof(serial_number);
      rc = device->getProperty(openni::OBEXTENSION_ID_SERIALNUMBER, serial_number, &data_size);
      if (rc != openni::STATUS_OK) {
        ROS_ERROR_STREAM("Failed to get serial number: " << openni::OpenNI::getExtendedError());
      } else if (serial_number_.empty() || serial_number == serial_number_) {
        ROS_INFO_STREAM("Device connected: " << device_info->getName()
                                             << " serial number: " << serial_number);
        device_uri_ = device_info->getUri();
        connected_devices_[device_uri_] = *device_info;
        device_ = device;
        startDevice();
      }
    }
    if (!device_connected_) {
      device->close();
    }
  }
  ROS_INFO_STREAM("Release device semaphore");
  sem_post(device_sem);
  ROS_INFO_STREAM("Release device semaphore done");
  if (connected_devices_.size() == number_of_devices_) {
    ROS_INFO_STREAM("All devices connected");
    sem_unlink(DEFAULT_SEM_NAME.c_str());
  }
}

void OBCameraNodeFactory::onDeviceDisconnected(const openni::DeviceInfo* device_info) {
  if (device_uri_ == device_info->getUri()) {
    device_uri_.clear();
    if (ob_camera_node_) {
      ob_camera_node_.reset();
    }
    if (device_) {
      device_->close();
      device_.reset();
    }
    if (uvc_camera_driver_) {
      uvc_camera_driver_.reset();
    }
    ROS_INFO_STREAM("Device disconnected: " << device_info->getUri());
    device_connected_ = false;
    sem_unlink(DEFAULT_SEM_NAME.c_str());
  }
}

OniLogSeverity OBCameraNodeFactory::getLogLevelFromString(const std::string& level) {
  if (level == "verbose") {
    return OniLogSeverity::ONI_LOG_VERBOSE;
  } else if (level == "info") {
    return OniLogSeverity::ONI_LOG_INFO;
  } else if (level == "warning") {
    return OniLogSeverity::ONI_LOG_WARNING;
  } else if (level == "error") {
    return OniLogSeverity::ONI_LOG_ERROR;
  } else if (level == "none") {
    return OniLogSeverity::ONI_LOG_SEVERITY_NONE;
  } else {
    ROS_WARN_STREAM("Unknown log level: " << level << " oni log will be disable");
    return OniLogSeverity::ONI_LOG_SEVERITY_NONE;
  }
}

void OBCameraNodeFactory::checkConnectionTimer() {
  if (!device_connected_) {
    ROS_INFO_STREAM("wait for device " << serial_number_ << " to be connected");
  }
}

void OBCameraNodeFactory::resetDeviceCallback(std_msgs::Empty::ConstPtr msg) {
  (void)msg;
  ROS_WARN_STREAM("Reset device");
  sem_unlink(DEFAULT_SEM_NAME.c_str());
  if (ob_camera_node_) {
    ob_camera_node_.reset();
  }
  if (device_) {
    device_.reset();
  }
  connected_devices_.erase(device_uri_);
  device_connected_ = false;
  openni::OpenNI::shutdown();
  initOpenNI();
  initDeviceListener();
}
}  // namespace astra_camera
