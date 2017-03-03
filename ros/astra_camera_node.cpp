/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * Copyright (c) 2016, Orbbec Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Tim Liu (liuhua@orbbec.com)
 */

#include "astra_camera/astra_driver.h"

static std::string get_command_option(const std::vector<std::string> &args, const std::string &option)
{
  auto it = std::find(args.begin(), args.end(), option);
  if (it != args.end() && ++it != args.end()) {
    return *it;
  }
  return std::string();
}

static void parse_command_options(int argc, char **argv, size_t *width, size_t *height,
                                  double *framerate)
{
  std::vector<std::string> args(argv, argv + argc);

  std::string width_str = get_command_option(args, "-w");
  if (!width_str.empty()) {
    *width = std::stoul(width_str.c_str());
  }

  std::string height_str = get_command_option(args, "-h");
  if (!height_str.empty()) {
    *height = std::stoul(height_str.c_str());
  }

  std::string framerate_str = get_command_option(args, "-f");
  if (!framerate_str.empty()) {
    *framerate = std::stod(framerate_str.c_str());
  }
}

struct AllowedVal {
  size_t width;
  size_t height;
  double framerate;
};

int main(int argc, char **argv){

  // This list of allowed values comes from cfg/Astra.cfg
  std::vector<AllowedVal> allowed{
    AllowedVal{1280, 1024, 30},
    AllowedVal{1280, 1024, 15},
    AllowedVal{1280, 720, 30},
    AllowedVal{1280, 720, 15},
    AllowedVal{640, 480, 30},
    AllowedVal{640, 480, 25},
    AllowedVal{320, 240, 25},
    AllowedVal{320, 240, 30},
    AllowedVal{320, 240, 60},
    AllowedVal{160, 120, 25},
    AllowedVal{160, 120, 30},
    AllowedVal{160, 120, 60},
  };
  size_t width = 1280;
  size_t height = 1024;
  double framerate = 30;

  // TODO(clalancette): parsing the command-line options here is temporary until
  // we get parameters working in ROS2.
  parse_command_options(argc, argv, &width, &height, &framerate);

  bool good_combo = false;
  for (std::vector<AllowedVal>::iterator it = allowed.begin() ; it != allowed.end(); ++it) {
    if (it->width == width && it->height == height && it->framerate == framerate) {
      good_combo = true;
      break;
    }
  }

  if (!good_combo) {
    std::cerr << "Invalid width/height/framerate combo; allowed combos are:" << std::endl;
    for (std::vector<AllowedVal>::iterator it = allowed.begin() ; it != allowed.end(); ++it) {
      std::cerr << "  " << it->width << "x" << it->height << "@" << it->framerate << "Hz" << std::endl;
    }
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::node::Node::SharedPtr n = rclcpp::node::Node::make_shared("astra_camera");
  rclcpp::node::Node::SharedPtr pnh = rclcpp::node::Node::make_shared("astra_camera_");

  astra_wrapper::AstraDriver drv(n, pnh, width, height, framerate);

  rclcpp::spin(n);

  return 0;
}
