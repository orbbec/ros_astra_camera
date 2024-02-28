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

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <limits>

#include "depth_traits.h"

namespace astra_camera {
    using PointCloud2 = sensor_msgs::PointCloud2;

// Handles float or uint16 depths
    template<typename T>
    inline void convert(const sensor_msgs::ImageConstPtr &depth_msg, PointCloud2::Ptr &cloud_msg,
                        const image_geometry::PinholeCameraModel &model, bool ordered_pc = true) {
        // Use correct principal point from calibration
        float center_x = model.cx();
        float center_y = model.cy();

        // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
        double unit_scaling = DepthTraits<T>::toMeters(T(1));
        float constant_x = unit_scaling / model.fx();
        float constant_y = unit_scaling / model.fy();
        int valid_points = 0;
        sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(depth_msg->width * depth_msg->height);
        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
        const T *depth_row = reinterpret_cast<const T *>(&depth_msg->data[0]);
        int row_step = depth_msg->step / sizeof(T);
        cloud_msg->header = depth_msg->header;
        cloud_msg->height = depth_msg->height;
        cloud_msg->width = depth_msg->width;
        cloud_msg->is_dense = false;
        cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
        cloud_msg->data.resize(cloud_msg->height * cloud_msg->row_step);
        auto bad_point = std::numeric_limits<float>::quiet_NaN();
        for (int v = 0; v < (int) cloud_msg->height; ++v, depth_row += row_step) {
            for (int u = 0; u < (int) cloud_msg->width; ++u) {
                T depth = depth_row[u];
                bool valid = true;
                // Missing points denoted by NaNs
                if (!DepthTraits<T>::valid(depth)) {
                    valid = false;
                    depth = bad_point;
                }

                // Fill in XYZ
                if (valid || ordered_pc) {
                    *iter_x = (u - center_x) * depth * constant_x;
                    *iter_y = (v - center_y) * depth * constant_y;
                    *iter_z = DepthTraits<T>::toMeters(depth);
                    ++iter_x, ++iter_y, ++iter_z;
                    valid_points++;
                }
            }
        }
        if (!ordered_pc) {
            cloud_msg->width = valid_points;
            cloud_msg->height = 1;
            cloud_msg->is_dense = true;
            modifier.resize(valid_points);
        }
    }
}  // namespace astra_camera
