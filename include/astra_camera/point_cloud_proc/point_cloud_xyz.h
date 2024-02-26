/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
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

#include "astra_camera/types.h"
#include "astra_camera/utils.h"
#include "depth_conversions.h"
#include "depth_traits.h"

namespace astra_camera {
    using namespace message_filters::sync_policies;
    namespace enc = sensor_msgs::image_encodings;
    using PointCloud2 = sensor_msgs::PointCloud2;

    class PointCloudXyzNode {
    public:
        PointCloudXyzNode(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

        ~PointCloudXyzNode();

    private:
        void connectCb();

        void disconnectCb();

        void depthCb(const sensor_msgs::ImageConstPtr &depth_msg,
                     const sensor_msgs::CameraInfoConstPtr &info_msg);

        bool SavePointCloudXyzCallback(std_srvs::Empty::Request &request,
                                       std_srvs::Empty::Response &response);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        std::shared_ptr <image_transport::ImageTransport> it_;
        image_transport::CameraSubscriber sub_depth_;
        ros::ServiceServer save_point_cloud_srv_;
        std::atomic_bool save_cloud_{false};
        int queue_size_ = 5;

        // Publications
        std::mutex connect_mutex_;
        ros::Publisher pub_point_cloud_;

        image_geometry::PinholeCameraModel model_;
        std::string data_save_path_;
        bool ordered_pc_ = true;
    };
}  // namespace astra_camera
