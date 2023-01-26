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

#include <thread>
#include <mutex>
#include <atomic>
#include <rclcpp/rclcpp.hpp>

#include <openni2/OpenNI.h>
#include <glog/logging.h>

#include "utils.h"
#include "ob_timer_filter.h"
#include "constants.h"

namespace astra_camera {
class OBFrameListener : public openni::VideoStream::NewFrameListener {
 public:
  OBFrameListener();

  ~OBFrameListener() override = default;

  void onNewFrame(openni::VideoStream& stream) override;

  void setUseDeviceTimer(bool enable);

  void setCallback(FrameCallbackFunction callback);

 private:
  openni::VideoFrameRef frame_;
  rclcpp::Logger logger_;
  FrameCallbackFunction callback_;
  std::mutex callback_lock_;
  std::atomic_bool have_callback_;
  bool user_device_timer_;
  std::shared_ptr<OBTimerFilter> timer_filter_;
};

}  // namespace astra_camera
