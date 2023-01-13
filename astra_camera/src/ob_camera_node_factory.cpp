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

#include <filesystem>
#include <fcntl.h>
#include "astra_camera/ob_camera_node_factory.h"
#include <semaphore.h>
#include <sys/shm.h>

namespace astra_camera {
using namespace std::chrono_literals;
OBCameraNodeFactory::OBCameraNodeFactory(const rclcpp::NodeOptions& node_options)
    : Node("astra_camera_node", "/", node_options), logger_(get_logger()) {
  init();
}

[[maybe_unused]] OBCameraNodeFactory::OBCameraNodeFactory(const std::string& node_name,
                                                          const std::string& ns,
                                                          const rclcpp::NodeOptions& node_options)
    : Node(node_name, ns, node_options), logger_(get_logger()) {
  init();
}

OBCameraNodeFactory::~OBCameraNodeFactory() {
  is_alive_ = false;
  device_connected_ = false;
  if (query_device_thread_ && query_device_thread_->joinable()) {
    query_device_thread_->join();
  }
  if (device_num_ > 1) {
    cleanUpSharedMemory();
  }
  openni::OpenNI::shutdown();
}

void OBCameraNodeFactory::cleanUpSharedMemory() {
  sem_unlink(DEFAULT_SEM_NAME.c_str());
  int shm_id = shmget(DEFAULT_SEM_KEY, 1, 0666 | IPC_CREAT);
  if (shm_id != -1) {
    shmctl(shm_id, IPC_RMID, nullptr);
  }
}

void OBCameraNodeFactory::queryDeviceThread() {
  while (rclcpp::ok() && is_alive_) {
    if (!device_connected_) {
      RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 1000, "Waiting for device connection...");
      auto device_list = ob_context_->queryDeviceList();
      for (auto& device_info : device_list) {
        onDeviceConnected(&device_info);
      }
      std::this_thread::sleep_for(100ms);
    } else {
      std::this_thread::sleep_for(1000ms);
    }
  }
}

void OBCameraNodeFactory::init() {
  is_alive_ = true;
  auto rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK) {
    RCLCPP_ERROR(logger_, "Initialize failed\n%s\n", openni::OpenNI::getExtendedError());
    exit(-1);
  }
  parameters_ = std::make_shared<Parameters>(this);
  use_uvc_camera_ = declare_parameter<bool>("uvc_camera.enable", false);
  serial_number_ = declare_parameter<std::string>("serial_number", "");
  device_num_ = declare_parameter<int>("device_num", 1);
  connection_delay_ = declare_parameter<int>("connection_delay", 100);

  auto disconnected_cb = [this](const openni::DeviceInfo* device_info) {
    onDeviceDisconnected(device_info);
  };
  ob_context_ = std::make_unique<OBContext>(disconnected_cb);
  CHECK_NOTNULL(ob_context_);
  check_connection_timer_ = this->create_wall_timer(1s, [this] { checkConnectionTimer(); });
  CHECK_NOTNULL(check_connection_timer_);
  query_device_thread_ = std::make_shared<std::thread>([this]() { queryDeviceThread(); });
  CHECK_NOTNULL(query_device_thread_);
  RCLCPP_INFO_STREAM(logger_, "init done.");
}

void OBCameraNodeFactory::startDevice(const std::shared_ptr<openni::Device>& device,
                                      const openni::DeviceInfo* device_info) {
  RCLCPP_INFO_STREAM(logger_, "starting device " << serial_number_);
  device_uri_ = device_info->getUri();
  device_ = device;
  if (ob_camera_node_) {
    ob_camera_node_.reset();
  }
  CHECK_NOTNULL(device_);
  CHECK_NOTNULL(parameters_);
  if (use_uvc_camera_) {
    if (uvc_camera_driver_) {
      uvc_camera_driver_.reset();
    }
    uvc_camera_driver_ = std::make_shared<UVCCameraDriver>(this, parameters_, serial_number_);
    ob_camera_node_ =
        std::make_unique<OBCameraNode>(this, device_, parameters_, uvc_camera_driver_);
  } else {
    ob_camera_node_ = std::make_unique<OBCameraNode>(this, device_, parameters_);
  }
  device_connected_ = true;
}

void OBCameraNodeFactory::onDeviceConnected(const openni::DeviceInfo* device_info) {
  if (device_info->getUri() == nullptr) {
    RCLCPP_ERROR_STREAM(logger_, "Device connected: " << device_info->getName() << " uri is null");
    return;
  }
  size_t connected_device_num = 0;
  sem_t* device_sem = nullptr;
  std::shared_ptr<int> sem_guard(nullptr, [&](int*) {
    if (device_num_ > 1 && device_sem != nullptr) {
      RCLCPP_INFO_STREAM(logger_, "Unlock device");
      sem_post(device_sem);
      if (connected_device_num >= device_num_) {
        RCLCPP_INFO_STREAM(logger_, "All devices connected,  sem_unlink");
        sem_unlink(DEFAULT_SEM_NAME.c_str());
        RCLCPP_INFO_STREAM(logger_, "All devices connected,  sem_unlink done..");
      }
    }
  });
  if (device_num_ > 1) {
    device_sem = sem_open(DEFAULT_SEM_NAME.c_str(), O_CREAT, 0644, 1);
    if (device_sem == (void*)SEM_FAILED) {
      RCLCPP_ERROR(logger_, "Failed to create semaphore");
      return;
    }
    RCLCPP_INFO_STREAM(logger_, "Waiting for device to be ready");
    int ret = sem_wait(device_sem);
    if (ret != 0) {
      RCLCPP_ERROR(logger_, "Failed to wait for device");
      return;
    }
  }
  auto device = std::make_shared<openni::Device>();
  RCLCPP_INFO_STREAM(logger_, "Trying to open device: " << device_info->getUri());
  std::this_thread::sleep_for(std::chrono::milliseconds(connection_delay_));
  auto rc = device->open(device_info->getUri());
  if (rc != openni::STATUS_OK) {
    RCLCPP_INFO_STREAM(logger_, "Failed to open device: " << device_info->getUri() << " error: "
                                                          << openni::OpenNI::getExtendedError());
    if (errno == EBUSY) {
      RCLCPP_INFO_STREAM(logger_, "Device is already opened OR device is in use");
    }
  } else {
    char serial_number[64];
    int data_size = sizeof(serial_number);
    rc = device->getProperty(openni::OBEXTENSION_ID_SERIALNUMBER, serial_number, &data_size);
    if (rc != openni::STATUS_OK) {
      RCLCPP_ERROR_STREAM(logger_,
                          "Failed to get serial number: " << openni::OpenNI::getExtendedError());
    } else if (serial_number_.empty() || serial_number == serial_number_) {
      RCLCPP_INFO_STREAM(logger_, "Device connected: " << device_info->getName()
                                                       << " serial number: " << serial_number);
      try {
        startDevice(device, device_info);
      } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(logger_, "Failed to start device: " << e.what());
        cleanUpSharedMemory();
      } catch (...) {
        RCLCPP_ERROR_STREAM(logger_, "Failed to start device: unknown error");
        cleanUpSharedMemory();
      }
      if (device_num_ > 1) {
        int shm_id = shmget(DEFAULT_SEM_KEY, 1, 0666 | IPC_CREAT);
        if (shm_id == -1) {
          RCLCPP_ERROR_STREAM(logger_, "Failed to create shared memory " << strerror(errno));
        } else {
          RCLCPP_INFO_STREAM(logger_, "Created shared memory");
          auto shm_ptr = (int*)shmat(shm_id, nullptr, 0);
          if (shm_ptr == (void*)-1) {
            RCLCPP_ERROR_STREAM(logger_, "Failed to attach shared memory " << strerror(errno));
          } else {
            RCLCPP_INFO_STREAM(logger_, "Attached shared memory");
            connected_device_num = *shm_ptr + 1;
            RCLCPP_ERROR_STREAM(logger_, "Current connected device " << connected_device_num);
            *shm_ptr = static_cast<int>(connected_device_num);
            RCLCPP_INFO_STREAM(logger_, "Wrote to shared memory");
            shmdt(shm_ptr);
            if (connected_device_num >= device_num_) {
              RCLCPP_INFO_STREAM(logger_, "All devices connected, removing shared memory");
              shmctl(shm_id, IPC_RMID, nullptr);
            }
          }
        }
      }
    } else {
      RCLCPP_INFO_STREAM(logger_, "Device connected: " << device_info->getName()
                                                       << " serial number: " << serial_number
                                                       << " does not match expected serial number: "
                                                       << serial_number_);
    }
  }
  if (!device_connected_) {
    RCLCPP_INFO_STREAM(logger_, "Device: " << device_info->getUri() << " is not connected");
    device->close();
    RCLCPP_INFO_STREAM(logger_, "OBCameraNodeFactory::onDeviceConnected close done.");
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
    RCLCPP_INFO_STREAM(logger_, "Device disconnected: " << device_info->getUri());
    device_connected_ = false;
    sem_unlink(DEFAULT_SEM_NAME.c_str());
  }
}

void OBCameraNodeFactory::checkConnectionTimer() {
  if (!device_connected_) {
    RCLCPP_INFO(logger_, "wait for device %s connect... ", serial_number_.c_str());
  }
}

}  // namespace astra_camera
