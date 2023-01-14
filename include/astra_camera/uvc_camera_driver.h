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

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <libuvc/libuvc.h>
#include <openni2/OpenNI.h>
#include <sensor_msgs/CameraInfo.h>

#include <boost/optional.hpp>

#include "constants.h"
#include "types.h"
#include "utils.h"

namespace astra_camera {
struct UVCCameraConfig {
  int vendor_id = 0;
  int product_id = 0;
  int width = 0;
  int height = 0;
  int fps = 0;
  std::string serial_number;
  std::string format;
  std::string frame_id;
  std::string optical_frame_id;
  int retry_count = 0;
  UVCCameraConfig() = default;
  UVCCameraConfig(const UVCCameraConfig&) = default;
  UVCCameraConfig(UVCCameraConfig&&) = default;
  UVCCameraConfig& operator=(const UVCCameraConfig&) = default;
  UVCCameraConfig& operator=(UVCCameraConfig&&) = default;
};

std::ostream& operator<<(std::ostream& os, const UVCCameraConfig& config);

class UVCCameraDriver {
 public:
  explicit UVCCameraDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                           const sensor_msgs::CameraInfo& camera_info,
                           const std::string& serial_number = "");

  ~UVCCameraDriver();

  void updateConfig(const UVCCameraConfig& config);

  void setVideoMode();

  void imageSubscribedCallback();

  void imageUnsubscribedCallback();

  void startStreaming();

  void stopStreaming() noexcept;

  int getResolutionX() const;

  int getResolutionY() const;

 private:
  void setupCameraControlService();

  sensor_msgs::CameraInfo getCameraInfo();

  static enum uvc_frame_format UVCFrameFormatString(const std::string& format);

  static void frameCallbackWrapper(uvc_frame_t* frame, void* ptr);

  void frameCallback(uvc_frame_t* frame);

  void autoControlsCallback(enum uvc_status_class status_class, int event, int selector,
                            enum uvc_status_attribute status_attribute, void* data,
                            size_t data_len);
  static void autoControlsCallbackWrapper(enum uvc_status_class status_class, int event,
                                          int selector, enum uvc_status_attribute status_attribute,
                                          void* data, size_t data_len, void* ptr);

  void openCamera();

  bool getUVCExposureCb(GetInt32Request& request, GetInt32Response& response);

  bool setUVCExposureCb(SetInt32Request& request, SetInt32Response& response);

  bool getUVCGainCb(GetInt32Request& request, GetInt32Response& response);

  bool setUVCGainCb(SetInt32Request& request, SetInt32Response& response);

  bool getUVCWhiteBalanceCb(GetInt32Request& request, GetInt32Response& response);

  bool setUVCWhiteBalanceCb(SetInt32Request& request, SetInt32Response& response);

  bool setUVCAutoExposureCb(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response);

  bool setUVCAutoWhiteBalanceCb(std_srvs::SetBoolRequest& request,
                                std_srvs::SetBoolResponse& response);

  bool getUVCMirrorCb(GetInt32Request& request, GetInt32Response& response);

  bool setUVCMirrorCb(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response);

  bool toggleUVCCamera(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response);

  bool saveImageCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response);

  int UVCGetControl(int control, int unit, int len, uvc_req_code req_code);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  UVCCameraConfig config_;
  std::string camera_name_ = "camera";
  std::string frame_id_;
  std::string color_info_uri_;
  ImageROI roi_;
  uvc_context_t* ctx_ = nullptr;
  uvc_device_t* device_ = nullptr;
  uvc_device_handle_t* device_handle_ = nullptr;
  uvc_frame_t* frame_buffer_ = nullptr;
  uvc_stream_ctrl_t ctrl_{};
  std::atomic_bool uvc_flip_{false};
  std::atomic_bool is_streaming_started{false};
  std::atomic_bool save_image_{false};
  std::atomic_bool is_camera_opened_{false};

  ros::ServiceServer get_uvc_exposure_srv_;
  ros::ServiceServer set_uvc_exposure_srv_;
  ros::ServiceServer get_uvc_gain_srv_;
  ros::ServiceServer set_uvc_gain_srv_;
  ros::ServiceServer get_uvc_white_balance_srv_;
  ros::ServiceServer set_uvc_white_balance_srv_;
  ros::ServiceServer set_uvc_auto_exposure_srv_;
  ros::ServiceServer set_uvc_auto_white_balance_srv_;
  ros::ServiceServer get_uvc_mirror_srv_;
  ros::ServiceServer set_uvc_mirror_srv_;
  ros::ServiceServer toggle_uvc_camera_srv_;
  ros::ServiceServer save_image_srv_;
  ros::Publisher image_publisher_;
  ros::Publisher camera_info_publisher_;
  sensor_msgs::CameraInfo camera_info_;
  std::recursive_mutex device_lock_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> color_info_manager_ = nullptr;
  int device_num_ = 1;
};
}  // namespace astra_camera
