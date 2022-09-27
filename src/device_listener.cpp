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

#include <astra_camera/device_listener.h>


namespace astra_camera {
DeviceListener::DeviceListener(DeviceConnectedCb connected_cb, DeviceDisconnectedCb disconnected_cb)
    : openni::OpenNI::DeviceConnectedListener(),
      openni::OpenNI::DeviceDisconnectedListener(),
      openni::OpenNI::DeviceStateChangedListener(),
      connected_cb_(std::move(connected_cb)),
      disconnected_cb_(std::move(disconnected_cb)) {
  openni::OpenNI::addDeviceConnectedListener(this);
  openni::OpenNI::addDeviceDisconnectedListener(this);
  openni::OpenNI::addDeviceStateChangedListener(this);
  // get list of currently connected devices
  openni::Array<openni::DeviceInfo> device_info_list;
  openni::OpenNI::enumerateDevices(&device_info_list);
  ROS_INFO_STREAM("Found " << device_info_list.getSize() << " devices");
  for (int i = 0; i < device_info_list.getSize(); ++i) {
    onDeviceConnected(&device_info_list[i]);
  }
}

DeviceListener::~DeviceListener() {
  openni::OpenNI::removeDeviceConnectedListener(this);
  openni::OpenNI::removeDeviceDisconnectedListener(this);
  openni::OpenNI::removeDeviceStateChangedListener(this);
}

void DeviceListener::onDeviceStateChanged(const openni::DeviceInfo* device_info,
                                          openni::DeviceState state) {
  ROS_INFO_STREAM("Device " << device_info->getUri() << " state changed to "
                            << state);
}

void DeviceListener::onDeviceConnected(const openni::DeviceInfo* device_info) {
  ROS_INFO_STREAM("onDeviceConnected");

  connected_cb_(device_info);
}

void DeviceListener::onDeviceDisconnected(const openni::DeviceInfo* device_info) {
  ROS_INFO_STREAM("onDeviceDisconnected");
  disconnected_cb_(device_info);
}
}  // namespace astra_camera
