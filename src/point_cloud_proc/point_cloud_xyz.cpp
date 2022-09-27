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

#include "astra_camera/point_cloud_proc/point_cloud_xyz.h"

namespace astra_camera {
PointCloudXyzNode::PointCloudXyzNode(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  nh_private_.param<int>("queue_size", queue_size_, 5);
  ros::SubscriberStatusCallback connect_cb = std::bind(&PointCloudXyzNode::connectCb, this);
  ros::SubscriberStatusCallback disconnect_cb = std::bind(&PointCloudXyzNode::disconnectCb, this);
  it_.reset(new image_transport::ImageTransport(nh));

  std::lock_guard<decltype(connect_mutex_)> lock_(connect_mutex_);
  pub_point_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("depth/points", queue_size_,
                                                             connect_cb, disconnect_cb);
  save_point_cloud_srv_ = nh_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
      "save_point_cloud_xyz",
      [this](auto &&req, auto &&res) { return this->SavePointCloudXyzCallback(req, res); });
}

PointCloudXyzNode::~PointCloudXyzNode() = default;

void PointCloudXyzNode::connectCb() {
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (!sub_depth_) {
    image_transport::TransportHints hints("raw", ros::TransportHints(), nh_private_);
    sub_depth_ = it_->subscribeCamera("depth/image_raw", queue_size_, &PointCloudXyzNode::depthCb,
                                      this, hints);
  }
}

void PointCloudXyzNode::disconnectCb() {
  if (pub_point_cloud_.getNumSubscribers() == 0) {
    sub_depth_.shutdown();
  }
}

void PointCloudXyzNode::depthCb(const sensor_msgs::ImageConstPtr &depth_msg,
                                const sensor_msgs::CameraInfoConstPtr &info_msg) {
  PointCloud2::Ptr cloud_msg(new PointCloud2);
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;
  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  // Update camera model
  model_.fromCameraInfo(info_msg);

  if (depth_msg->encoding == enc::TYPE_16UC1 || depth_msg->encoding == enc::MONO16) {
    convert<uint16_t>(depth_msg, cloud_msg, model_);
  } else if (depth_msg->encoding == enc::TYPE_32FC1) {
    convert<float>(depth_msg, cloud_msg, model_);
  } else {
    ROS_WARN_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }
  if (save_cloud_) {
    save_cloud_ = false;
    auto now = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
    auto current_path = boost::filesystem::current_path().string();
    std::string filename = current_path + "/point_cloud/points_xyz_" + ss.str() + ".ply";
    if (!boost::filesystem::exists(current_path + "/point_cloud")) {
      boost::filesystem::create_directory(current_path + "/point_cloud");
    }
    ROS_INFO_STREAM("Saving point cloud to " << filename);
    savePointToPly(cloud_msg, filename);
  }
  pub_point_cloud_.publish(cloud_msg);
}

bool PointCloudXyzNode::SavePointCloudXyzCallback(std_srvs::Empty::Request &request,
                                                  std_srvs::Empty::Response &response) {
  (void)request;
  (void)response;
  ROS_INFO("SavePointCloudXyzCallback");
  save_cloud_ = true;
  return true;
}

}  // namespace astra_camera
