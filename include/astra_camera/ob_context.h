/***************************************************************************/
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
#include <openni2/OpenNI.h>
#include <ros/ros.h>

namespace astra_camera {

using DeviceConnectedCb = std::function<void(const openni::DeviceInfo*)>;

using DeviceDisconnectedCb = std::function<void(const openni::DeviceInfo*)>;

class OBContext : public openni::OpenNI::DeviceConnectedListener,
                  public openni::OpenNI::DeviceDisconnectedListener,
                  public openni::OpenNI::DeviceStateChangedListener {
 public:
  explicit OBContext(DeviceDisconnectedCb disconnected_cb);

  ~OBContext() override;

  std::vector<openni::DeviceInfo> queryDeviceList();

  void onDeviceStateChanged(const openni::DeviceInfo* pInfo, openni::DeviceState state) override;

  void onDeviceConnected(const openni::DeviceInfo* pInfo) override;

  void onDeviceDisconnected(const openni::DeviceInfo* pInfo) override;

 private:
  DeviceDisconnectedCb disconnected_cb_;
  bool first_query_ = true;
  std::recursive_mutex mutex_;
  std::map<std::string, openni::DeviceInfo> device_info_list_;
};

}  // namespace astra_camera
