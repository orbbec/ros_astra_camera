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

#include <astra_camera/ob_context.h>

namespace astra_camera {
OBContext::OBContext(DeviceDisconnectedCb disconnected_cb)
    : openni::OpenNI::DeviceConnectedListener(),
      openni::OpenNI::DeviceDisconnectedListener(),
      openni::OpenNI::DeviceStateChangedListener(),
      disconnected_cb_(std::move(disconnected_cb)) {
  openni::OpenNI::addDeviceConnectedListener(this);
  openni::OpenNI::addDeviceDisconnectedListener(this);
  openni::OpenNI::addDeviceStateChangedListener(this);
}

OBContext::~OBContext() {
  openni::OpenNI::removeDeviceConnectedListener(this);
  openni::OpenNI::removeDeviceDisconnectedListener(this);
  openni::OpenNI::removeDeviceStateChangedListener(this);
}

std::vector<openni::DeviceInfo> OBContext::queryDeviceList() {
  // get list of currently connected devices
  std::vector<openni::DeviceInfo> device_list;
  if (first_query_) {
    openni::Array<openni::DeviceInfo> device_info_list;
    openni::OpenNI::enumerateDevices(&device_info_list);
    ROS_INFO_STREAM("Found " << device_info_list.getSize() << " devices");
    for (int i = 0; i < device_info_list.getSize(); ++i) {
      device_list.emplace_back(device_info_list[i]);
      std::lock_guard<decltype(mutex_)> lock(mutex_);
      device_info_list_[device_info_list[i].getUri()] = device_info_list[i];
    }
    first_query_ = false;
  } else {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    for (auto& device_info : device_info_list_) {
      device_list.emplace_back(device_info.second);
    }
  }
  return device_list;
}

void OBContext::onDeviceStateChanged(const openni::DeviceInfo* device_info,
                                     openni::DeviceState state) {
  ROS_INFO_STREAM("Device " << device_info->getUri() << " state changed to " << state);
}

void OBContext::onDeviceConnected(const openni::DeviceInfo* device_info) {
  ROS_INFO_STREAM("OBContext::onDeviceConnected");
  std::lock_guard<decltype(mutex_)> lock(mutex_);
  device_info_list_[device_info->getUri()] = *device_info;
}

void OBContext::onDeviceDisconnected(const openni::DeviceInfo* device_info) {
  ROS_INFO_STREAM("OBContext::onDeviceDisconnected");
  std::lock_guard<decltype(mutex_)> lock(mutex_);
  disconnected_cb_(device_info);
  device_info_list_.erase(device_info->getUri());
  ROS_INFO_STREAM("OBContext::onDeviceDisconnected done...");
}
}  // namespace astra_camera
