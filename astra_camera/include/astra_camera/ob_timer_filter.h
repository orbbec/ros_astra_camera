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
#include <deque>

#include <rclcpp/rclcpp.hpp>

#include <openni2/OpenNI.h>
#include <glog/logging.h>

namespace astra_camera {
class OBTimerFilter {
 public:
  explicit OBTimerFilter(std::size_t buffer_len_);
  ~OBTimerFilter();

  void addSample(double sample);

  double getMedian();

  double getMovingAvg();

  void clear();

 private:
  std::size_t buffer_len_;
  rclcpp::Logger logger_;

  std::deque<double> buffer_;
};
}  // namespace astra_camera
