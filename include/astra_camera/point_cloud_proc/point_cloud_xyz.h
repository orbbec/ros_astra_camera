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

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/imgproc/imgproc.hpp>

#include "depth_conversions.h"
#include "depth_traits.h"

#include "astra_camera/utils.h"
#include "astra_camera/types.h"

namespace astra_camera {
using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;
using PointCloud2 = sensor_msgs::PointCloud2;

class PointCloudXyzNode {
 public:
  PointCloudXyzNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~PointCloudXyzNode();

 private:
  void connectCb();
  void disconnectCb();

  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

  bool SavePointCloudXyzCallback(std_srvs::Empty::Request& request,
                                 std_srvs::Empty::Response& response);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_depth_;
  ros::ServiceServer save_point_cloud_srv_;
  std::atomic_bool save_cloud_{false};
  int queue_size_ = 5;

  // Publications
  std::mutex connect_mutex_;
  ros::Publisher pub_point_cloud_;

  image_geometry::PinholeCameraModel model_;
};
}  // namespace astra_camera
