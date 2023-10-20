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
#include <sys/mman.h>

#include <boost/filesystem.hpp>

#include "astra_camera/ob_camera_node.h"

namespace astra_camera {

OBCameraNodeFactory::OBCameraNodeFactory(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  ROS_INFO_STREAM("OBCameraNodeFactory::OBCameraNodeFactory");
  init();
}

OBCameraNodeFactory::~OBCameraNodeFactory() {
  is_alive_ = false;
  device_connected_ = false;
  ROS_INFO_STREAM("OBCameraNodeFactory::~OBCameraNodeFactory stop query device thread");
  if (query_device_thread_ && query_device_thread_->joinable()) {
    query_device_thread_->join();
  }
  ROS_INFO_STREAM("OBCameraNodeFactory::~OBCameraNodeFactory stop query device thread  done.");
  if (device_ && device_->isValid()) {
    device_->close();
  }
  openni::OpenNI::shutdown();
}

void OBCameraNodeFactory::init() {
  ROS_INFO_STREAM("Initializing OBCameraNodeFactory...");
  shm_id_ = shm_open(DEFAULT_LOCK_FILE.c_str(), O_CREAT | O_RDWR, 0777);
  if (shm_id_ == -1) {
    ROS_ERROR_STREAM("shm_open failed");
    exit(-1);
  }
  ROS_INFO_STREAM("ftruncate");
  auto ret = ftruncate(shm_id_, sizeof(pthread_mutex_t));
  ROS_INFO_STREAM("ftruncate done");
  if (ret == -1) {
    ROS_ERROR_STREAM("ftruncate failed");
    exit(-1);
  }
  ROS_INFO_STREAM("mmap");
  astra_device_lock_ = static_cast<pthread_mutex_t*>(
      mmap(NULL, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED, shm_id_, 0));
  ROS_INFO_STREAM("mmap done");
  if (astra_device_lock_ == MAP_FAILED) {
    ROS_ERROR_STREAM("mmap failed");
    exit(-1);
  }
  pthread_mutexattr_init(&astra_device_lock_attr_);
  pthread_mutexattr_setpshared(&astra_device_lock_attr_, PTHREAD_PROCESS_SHARED);
  pthread_mutex_init(astra_device_lock_, &astra_device_lock_attr_);
  is_alive_ = true;
  oni_log_level_str_ = nh_private_.param<std::string>("oni_log_level", "info");
  oni_log_to_console_ = nh_private_.param<bool>("oni_log_to_console", false);
  oni_log_to_file_ = nh_private_.param<bool>("oni_log_to_file", false);
  auto log_level = getLogLevelFromString(oni_log_level_str_);
  openni::OpenNI::setLogMinSeverity(log_level);
  openni::OpenNI::setLogConsoleOutput(oni_log_to_console_);
  openni::OpenNI::setLogFileOutput(oni_log_to_file_);
  auto rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK) {
    ROS_ERROR("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());
    exit(-1);
  }
  use_uvc_camera_ = nh_private_.param<bool>("use_uvc_camera", false);
  serial_number_ = nh_private_.param<std::string>("serial_number", "");
  device_num_ = nh_private_.param<int>("device_num", 1);
  connection_delay_ = nh_private_.param<int>("connection_delay", 100);
  auto disconnected_cb = [this](const openni::DeviceInfo* device_info) {
    this->onDeviceDisconnected(device_info);
  };
  if (context_) {
    context_.reset();
  }
  context_ = std::make_unique<Context>(disconnected_cb);
  check_connection_timer_ = nh_.createWallTimer(
      ros::WallDuration(1.0), [this](const ros::WallTimerEvent&) { this->checkConnectionTimer(); });
  CHECK(check_connection_timer_.isValid());
  query_device_thread_ = std::make_unique<std::thread>([this] { this->queryDevice(); });
  ROS_INFO_STREAM("init Done");
}

void OBCameraNodeFactory::startDevice(const std::shared_ptr<openni::Device>& device,
                                      const openni::DeviceInfo* device_info) {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  ROS_INFO_STREAM("Start device " << serial_number_);
  device_uri_ = device_info->getUri();
  device_ = device;
  if (ob_camera_node_) {
    ob_camera_node_.reset();
  }
  CHECK_NOTNULL(device_.get());
  try {
    ob_camera_node_ = std::make_unique<OBCameraNode>(nh_, nh_private_, device_, use_uvc_camera_);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Start device " << serial_number_ << " failed: " << e.what());
    ob_camera_node_.reset();
    has_exception_ = true;
    device_uri_ = "";
    device_connected_ = false;
    return;
  } catch (...) {
    ROS_ERROR_STREAM("Start device " << serial_number_ << " failed");
    ob_camera_node_.reset();
    device_uri_ = "";
    device_connected_ = false;
    has_exception_ = true;
    ROS_ERROR_STREAM("Device is Not Connected, return");
    return;
  }
  device_connected_ = true;
  ROS_INFO_STREAM("Start device " << serial_number_ << " done");
}

void OBCameraNodeFactory::onDeviceConnected(const openni::DeviceInfo* device_info) {
  ROS_INFO_STREAM_THROTTLE(1, "Device connected: (name, " << device_info->getName() << ") (uri, "
                                                          << device_info->getUri() << ") (vendor, "
                                                          << device_info->getVendor() << ")");
  if (device_info->getUri() == nullptr) {
    ROS_ERROR_STREAM_THROTTLE(1, "Device connected: " << device_info->getName() << " uri is null");
    return;
  }
  pthread_mutex_lock(astra_device_lock_);
  std::shared_ptr<int> lock_guard(nullptr,
                                  [this](int*) { pthread_mutex_unlock(astra_device_lock_); });
  auto device = std::make_shared<openni::Device>();
  ROS_INFO_STREAM_THROTTLE(1, "Trying to open device: " << device_info->getUri());
  std::this_thread::sleep_for(std::chrono::milliseconds(connection_delay_));
  ROS_INFO_STREAM_THROTTLE(1, "OBCameraNodeFactory::onDeviceConnected Open device start");
  auto rc = device->open(device_info->getUri());
  ROS_INFO_STREAM_THROTTLE(
      1, "OBCameraNodeFactory::onDeviceConnected Open device done, STATUS " << rc);
  if (rc != openni::STATUS_OK) {
    if (errno == EBUSY) {
      ROS_INFO_STREAM_THROTTLE(
          1, "Device is already opened OR device is in use, may be it belong to other node");
    } else {
      ROS_ERROR_STREAM_THROTTLE(1, "Failed to open device: " << device_info->getUri() << " "
                                                             << openni::OpenNI::getExtendedError());
    }
  } else {
    char serial_number[64];
    int data_size = sizeof(serial_number);
    rc = device->getProperty(openni::OBEXTENSION_ID_SERIALNUMBER, serial_number, &data_size);
    if (rc != openni::STATUS_OK) {
      ROS_ERROR_STREAM("Failed to get serial number: " << openni::OpenNI::getExtendedError());
    } else if (serial_number_.empty() || std::string(serial_number) == serial_number_) {
      ROS_INFO_STREAM("Device connected: " << device_info->getName()
                                           << " serial number: " << serial_number);
      try {
        startDevice(device, device_info);
      } catch (const std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(1, "Failed to start device: " << e.what());
        device.reset();
      } catch (...) {
        ROS_ERROR_STREAM_THROTTLE(1, "Failed to start device");
        device.reset();
      }
    } else {
      ROS_INFO_STREAM("Device connected: "
                      << device_info->getName() << " serial number: " << serial_number
                      << " does not match expected serial number: " << serial_number_);
    }
  }
  if (!device_connected_ && device && device->isValid()) {
    ROS_INFO_STREAM("Device: " << device_info->getUri() << " is not connected");
    if (!has_exception_) {
      device->close();
    }
    has_exception_ = false;
    ROS_INFO_STREAM("OBCameraNodeFactory::onDeviceConnected close done.");
  }
}

void OBCameraNodeFactory::onDeviceDisconnected(const openni::DeviceInfo* device_info) {
  ROS_INFO_STREAM("Device disconnected: (name, " << device_info->getName() << ") (uri, "
                                                 << device_info->getUri() << ") (vendor, "
                                                 << device_info->getVendor() << ")");
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (device_uri_ == device_info->getUri()) {
    device_uri_.clear();
    if (ob_camera_node_) {
      ROS_INFO("Stop ob camera node");
      ob_camera_node_.reset();
      ROS_INFO("Stop camera node done...");
    }
    if (device_ && device_->isValid()) {
      ROS_INFO("OBCameraNodeFactory:: Close device");
      device_->close();
      ROS_INFO("Close device done...");
      device_.reset();
    }
    ROS_INFO_STREAM("Device disconnected: " << device_info->getUri());
    device_connected_ = false;
    ROS_INFO_STREAM("sem_unlink");
    sem_unlink(DEFAULT_SEM_NAME.c_str());
    ROS_INFO_STREAM("sem_unlink done...");
  } else {
    ROS_INFO_STREAM("Device disconnected: " << device_info->getUri() << " not used in this node");
    ROS_INFO_STREAM("current device uri: " << device_uri_);
  }
  ROS_INFO_STREAM("OBCameraNodeFactory::onDeviceDisconnected done...");
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

void OBCameraNodeFactory::queryDevice() {
  while (is_alive_ && ros::ok()) {
    if (!device_connected_) {
      ROS_INFO_STREAM_THROTTLE(1, "Query device");
      auto device_info_list = context_->queryDeviceList();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      for (auto& device_info : device_info_list) {
        onDeviceConnected(&device_info);
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }
}

}  // namespace astra_camera
