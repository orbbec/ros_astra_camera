/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2022 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <magic_enum.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/distortion_models.hpp>

#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <openni2/OpenNI.h>
#include <openni2/PS1080.h>

#include "astra_camera_msgs/msg/device_info.hpp"
#include "astra_camera_msgs/srv/get_device_info.hpp"
#include "astra_camera_msgs/msg/extrinsics.hpp"
#include "astra_camera_msgs/msg/metadata.hpp"
#include "astra_camera_msgs/srv/get_int32.hpp"
#include "astra_camera_msgs/srv/get_string.hpp"
#include "astra_camera_msgs/srv/set_int32.hpp"

#include "constants.h"
#include "utils.h"
#include "uvc_camera_driver.h"
#include "dynamic_params.h"
#include "types.h"
#include "ob_frame_listener.h"

namespace astra_camera {
class OBCameraNode {
 public:
  OBCameraNode() = delete;

  OBCameraNode(rclcpp::Node* node, std::shared_ptr<openni::Device> device,
               std::shared_ptr<Parameters> parameters);

  OBCameraNode(rclcpp::Node* node, std::shared_ptr<openni::Device> device,
               std::shared_ptr<Parameters> parameters,
               std::shared_ptr<UVCCameraDriver> uvc_camera_driver);

  ~OBCameraNode();

  void clean();

  void init();

 private:
  void pollFrame();

  void setupCameraCtrlServices();

  void setupConfig();

  void setupDevices();

  void setupVideoMode();

  void startStreams();

  void stopStreams();

  void getParameters();

  void setupTopics();

  void setupPublishers();

  void publishStaticTF(const rclcpp::Time& t, const std::vector<float>& trans,
                       const tf2::Quaternion& q, const std::string& from, const std::string& to);

  void calcAndPublishStaticTransform();

  void publishDynamicTransforms();

  void publishStaticTransforms();

  void setImageRegistrationMode(bool data);

  bool setMirrorCallback(const std::shared_ptr<SetBool::Request>& request,
                         std::shared_ptr<SetBool::Response>& response,
                         const stream_index_pair& stream_index);

  bool getExposureCallback(const std::shared_ptr<GetInt32::Request>& request,
                           std::shared_ptr<GetInt32::Response>& response,
                           const stream_index_pair& stream_index);

  bool setExposureCallback(const std::shared_ptr<SetInt32::Request>& request,
                           std::shared_ptr<SetInt32::Response>& response,
                           const stream_index_pair& stream_index);

  bool getGainCallback(const std::shared_ptr<GetInt32::Request>& request,
                       std::shared_ptr<GetInt32::Response>& response,
                       const stream_index_pair& stream_index);

  bool setGainCallback(const std::shared_ptr<SetInt32::Request>& request,
                       std::shared_ptr<SetInt32::Response>& response,
                       const stream_index_pair& stream_index);

  bool getAutoWhiteBalanceEnabledCallback(const std::shared_ptr<GetInt32::Request>& request,
                                          std::shared_ptr<GetInt32::Response>& response,
                                          const stream_index_pair& stream_index);

  bool setAutoWhiteBalanceEnabledCallback(const std::shared_ptr<SetBool::Request>& request,
                                          std::shared_ptr<SetBool::Response>& response,
                                          const stream_index_pair& stream_index);

  bool setAutoExposureCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response>& response,
                               const stream_index_pair& stream_index);

  bool setLaserEnableCallback(const std::shared_ptr<SetBool::Request>& request,
                              std::shared_ptr<SetBool::Response>& response);

  bool setLdpEnableCallback(const std::shared_ptr<SetBool::Request>& request,
                            std::shared_ptr<SetBool::Response>& response);

  bool setFanCallback(const std::shared_ptr<SetBool::Request>& request,
                      std::shared_ptr<SetBool::Response>& response);

  bool getDeviceInfoCallback(const std::shared_ptr<GetDeviceInfo::Request>& request,
                             std::shared_ptr<GetDeviceInfo::Response>& response);

  bool getCameraInfoCallback(const std::shared_ptr<GetCameraInfo::Request>& request,
                             std::shared_ptr<GetCameraInfo::Response>& response);

  bool getSDKVersion(const std::shared_ptr<GetString::Request>& request,
                     std::shared_ptr<GetString::Response>& response);

  bool getCameraParamsCallback(const std::shared_ptr<GetCameraParams::Request>& request,
                               std::shared_ptr<GetCameraParams::Response>& response);

  bool toggleSensorCallback(const std::shared_ptr<SetBool::Request>& request,
                            std::shared_ptr<SetBool::Response>& response,
                            const stream_index_pair& stream_index);

  bool toggleSensor(const stream_index_pair& stream_index, bool enabled, std::string& msg);

  bool getSupportedVideoModesCallback(const std::shared_ptr<GetString ::Request>& request,
                                      std::shared_ptr<GetString ::Response>& response,
                                      const stream_index_pair& stream_index);

  void onNewFrameCallback(const openni::VideoFrameRef& frame,
                          const stream_index_pair& stream_index);

  void setDepthColorSync(bool data);

  void setDepthToColorResolution(int width, int height);

  OBCameraParams getCameraParams();

  static sensor_msgs::msg::CameraInfo OBCameraParamsToCameraInfo(const OBCameraParams& params);

  double getFocalLength(const stream_index_pair& stream_index, int y_resolution);

  CameraInfo::UniquePtr getIRCameraInfo();

  CameraInfo::UniquePtr getDepthCameraInfo();

  CameraInfo::UniquePtr getColorCameraInfo();

 private:
  rclcpp::Node* node_ = nullptr;
  std::shared_ptr<openni::Device> device_ = nullptr;
  std::shared_ptr<Parameters> parameters_ = nullptr;
  std::shared_ptr<UVCCameraDriver> uvc_camera_driver_ = nullptr;
  rclcpp::Logger logger_;
  bool use_uvc_camera_ = false;
  openni::DeviceInfo device_info_{};
  std::string camera_name_ = "camera";
  std::atomic_bool is_running_{false};
  std::atomic_bool is_initialized_{false};
  std::condition_variable stream_started_cv_;
  std::mutex stream_lock_;
  std::atomic_bool run_streaming_poller_{false};
  std::shared_ptr<std::thread> poll_stream_thread_ = nullptr;
  std::map<stream_index_pair, bool> enable_;
  std::map<stream_index_pair, bool> stream_started_;
  std::map<stream_index_pair, int> width_;
  std::map<stream_index_pair, int> height_;
  std::map<stream_index_pair, int> fps_;
  std::map<stream_index_pair, openni::PixelFormat> format_;
  std::map<stream_index_pair, int> image_format_;
  std::map<stream_index_pair, std::string> encoding_;
  std::map<stream_index_pair, cv::Mat> images_;
  std::vector<int> compression_params_;
  std::string camera_link_frame_id_;
  std::map<stream_index_pair, std::string> frame_id_;
  std::map<stream_index_pair, std::string> optical_frame_id_;
  std::map<stream_index_pair, std::string> depth_aligned_frame_id_;
  std::map<stream_index_pair, std::string> stream_name_;
  std::map<stream_index_pair, std::shared_ptr<openni::VideoStream>> streams_;
  std::map<stream_index_pair, openni::VideoMode> stream_video_mode_;
  std::map<stream_index_pair, std::vector<openni::VideoMode>> supported_video_modes_;
  std::map<stream_index_pair, int> unit_step_size_;
  std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
      image_publishers_;
  std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>
      camera_info_publishers_;

  std::map<stream_index_pair, rclcpp::Service<GetInt32>::SharedPtr> get_exposure_srv_;
  std::map<stream_index_pair, rclcpp::Service<SetInt32>::SharedPtr> set_exposure_srv_;
  std::map<stream_index_pair, rclcpp::Service<GetInt32>::SharedPtr> get_gain_srv_;
  std::map<stream_index_pair, rclcpp::Service<SetInt32>::SharedPtr> set_gain_srv_;
  std::map<stream_index_pair, rclcpp::Service<SetBool>::SharedPtr> set_mirror_srv_;
  std::map<stream_index_pair, rclcpp::Service<SetBool>::SharedPtr> toggle_sensor_srv_;
  std::map<stream_index_pair, rclcpp::Service<GetInt32>::SharedPtr> get_white_balance_srv_;
  std::map<stream_index_pair, rclcpp::Service<SetBool>::SharedPtr> set_white_balance_srv_;
  std::map<stream_index_pair, rclcpp::Service<GetString>::SharedPtr> get_supported_video_modes_srv_;
  rclcpp::Service<GetString>::SharedPtr get_sdk_version_srv_;
  rclcpp::Service<GetCameraParams>::SharedPtr get_camera_params_srv_;
  std::map<stream_index_pair, rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr>
      set_auto_exposure_srv_;
  rclcpp::Service<GetDeviceInfo>::SharedPtr get_device_srv_;
  rclcpp::Service<SetBool>::SharedPtr set_laser_enable_srv_;
  rclcpp::Service<SetBool>::SharedPtr set_ldp_enable_srv_;
  rclcpp::Service<SetBool>::SharedPtr set_fan_enable_srv_;
  rclcpp::Service<GetCameraInfo>::SharedPtr get_camera_info_srv_;

  bool publish_tf_ = true;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_ = nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_ = nullptr;
  std::vector<geometry_msgs::msg::TransformStamped> static_tf_msgs_;
  rclcpp::Publisher<Extrinsics>::SharedPtr extrinsics_publisher_ = nullptr;
  std::shared_ptr<std::thread> tf_thread_ = nullptr;
  std::condition_variable tf_cv_;
  double tf_publish_rate_ = 10.0;
  bool depth_align_ = false;
  std::optional<OBCameraParams> camera_params_;
  double depth_ir_x_offset_ = 0.0;
  double depth_ir_y_offset_ = 0.0;
  bool color_depth_synchronization_ = false;
  ImageROI color_roi_;
  ImageROI depth_roi_;
  int depth_scale_ = 1;
};

}  // namespace astra_camera
