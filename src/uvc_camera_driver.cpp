/*********************************************************************
 * Software License Agreement (BSD License)
 *  Copyright (C) 2012 Ken Tossell
 *  Copyright (c) 2022 Orbbec 3D Technology, Inc
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
 *   * Neither the name of the author nor other contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
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

#include "astra_camera/uvc_camera_driver.h"

#include <cv_bridge/cv_bridge.h>

#include <boost/optional.hpp>
#include <opencv2/opencv.hpp>

#define libuvc_VERSION \
  (libuvc_VERSION_MAJOR * 10000 + libuvc_VERSION_MINOR * 100 + libuvc_VERSION_PATCH)
/** Converts an unaligned four-byte little-endian integer into an int32 */
#define DW_TO_INT(p) ((p)[0] | ((p)[1] << 8) | ((p)[2] << 16) | ((p)[3] << 24))
/** Converts an unaligned two-byte little-endian integer into an int16 */
#define SW_TO_SHORT(p) ((p)[0] | ((p)[1] << 8))
/** Converts an int16 into an unaligned two-byte little-endian integer */
#define SHORT_TO_SW(s, p) \
  (p)[0] = (s);           \
  (p)[1] = (s) >> 8;
/** Converts an int32 into an unaligned four-byte little-endian integer */
#define INT_TO_DW(i, p) \
  (p)[0] = (i);         \
  (p)[1] = (i) >> 8;    \
  (p)[2] = (i) >> 16;   \
  (p)[3] = (i) >> 24;

/** Selects the nth item in a doubly linked list. n=-1 selects the last item. */
#define DL_NTH(head, out, n)               \
  do {                                     \
    int dl_nth_i = 0;                      \
    LDECLTYPE(head) dl_nth_p = (head);     \
    if ((n) < 0) {                         \
      while (dl_nth_p && dl_nth_i > (n)) { \
        dl_nth_p = dl_nth_p->prev;         \
        dl_nth_i--;                        \
      }                                    \
    } else {                               \
      while (dl_nth_p && dl_nth_i < (n)) { \
        dl_nth_p = dl_nth_p->next;         \
        dl_nth_i++;                        \
      }                                    \
    }                                      \
    (out) = dl_nth_p;                      \
  } while (0);

namespace astra_camera {

std::ostream& operator<<(std::ostream& os, const UVCCameraConfig& config) {
  os << "vendor_id: " << std::hex << config.vendor_id << std::endl;
  os << "product_id: " << std::hex << config.product_id << std::endl;
  os << "width: " << std::dec << config.width << std::endl;
  os << "height: " << config.height << std::endl;
  os << "fps: " << config.fps << std::endl;
  os << "serial_number: " << config.serial_number << std::endl;
  os << "format: " << config.format << std::endl;
  return os;
}

UVCCameraDriver::UVCCameraDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                                 const sensor_msgs::CameraInfo& camera_info,
                                 const std::string& serial_number)
    : nh_(nh), nh_private_(nh_private), camera_info_(camera_info) {
  auto err = uvc_init(&ctx_, nullptr);
  if (err != UVC_SUCCESS) {
    uvc_perror(err, "ERROR: uvc_init");
    ROS_ERROR_STREAM("init uvc context failed, exit");
    throw std::runtime_error("init uvc context failed");
  }
  config_.serial_number = serial_number;
  device_num_ = nh_private_.param<int>("device_num", 1);
  uvc_flip_ = nh_private_.param<bool>("uvc_flip", false);
  config_.vendor_id = nh_private_.param<int>("uvc_vendor_id", 0x0);
  config_.product_id = nh_private_.param<int>("uvc_product_id", 0x0);
  config_.width = nh_private_.param<int>("color_width", 640);
  config_.height = nh_private_.param<int>("color_height", 480);
  config_.fps = nh_private_.param<int>("color_fps", 30);
  config_.format = nh_private.param<std::string>("uvc_camera_format", "mjpeg");
  config_.retry_count = nh_private.param<int>("uvc_retry_count", 500);
  roi_.x = nh_private.param<int>("color_roi_x", -1);
  roi_.y = nh_private.param<int>("color_roi_y", -1);
  roi_.width = nh_private.param<int>("color_roi_width", -1);
  roi_.height = nh_private.param<int>("color_roi_height", -1);
  camera_name_ = nh_private.param<std::string>("camera_name", "camera");
  config_.frame_id = camera_name_ + "_color_frame";
  config_.optical_frame_id = camera_name_ + "_color_optical_frame";
  camera_info_publisher_ = nh_.advertise<sensor_msgs::CameraInfo>("color/camera_info", 1, true);
  color_info_uri_ = nh_private.param<std::string>("color_info_uri", "");
  color_info_manager_ =
      std::make_shared<camera_info_manager::CameraInfoManager>(nh_, "rgb_camera", color_info_uri_);
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  image_publisher_ = nh_.advertise<sensor_msgs::Image>(
      "color/image_raw", 10, boost::bind(&UVCCameraDriver::imageSubscribedCallback, this),
      boost::bind(&UVCCameraDriver::imageUnsubscribedCallback, this));
  setupCameraControlService();
  openCamera();
  auto info = getCameraInfo();
  camera_info_publisher_.publish(info);
}

UVCCameraDriver::~UVCCameraDriver() {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  uvc_close(device_handle_);
  device_handle_ = nullptr;
  uvc_unref_device(device_);
  device_ = nullptr;
  uvc_exit(ctx_);
  ctx_ = nullptr;
  if (frame_buffer_) {
    uvc_free_frame(frame_buffer_);
  }
  frame_buffer_ = nullptr;
}

void UVCCameraDriver::openCamera() {
  ROS_INFO_STREAM("open uvc camera");
  uvc_error_t err;
  auto serial_number = config_.serial_number.empty() ? nullptr : config_.serial_number.c_str();
  CHECK(device_ == nullptr);
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  err = uvc_find_device(ctx_, &device_, config_.vendor_id, config_.product_id, serial_number);
  if (err != UVC_SUCCESS && device_num_ == 1) {
    // retry serial number == nullptr
    err = uvc_find_device(ctx_, &device_, config_.vendor_id, config_.product_id, nullptr);
  }
  if (err != UVC_SUCCESS) {
    uvc_perror(err, "ERROR: uvc_find_device");
    ROS_ERROR_STREAM("find uvc device failed, retry " << config_.retry_count << " times");
    for (int i = 0; i < config_.retry_count; i++) {
      err = uvc_find_device(ctx_, &device_, config_.vendor_id, config_.product_id, serial_number);
      if (err == UVC_SUCCESS) {
        break;
      }
      usleep(100 * i);
    }
  }
  ROS_INFO_STREAM("uvc config: " << config_);
  if (err != UVC_SUCCESS) {
    std::stringstream ss;
    ss << "Find device error " << uvc_strerror(err) << " process will be exit";
    ROS_ERROR_STREAM(ss.str());
    uvc_unref_device(device_);
    throw std::runtime_error(ss.str());
  }
  CHECK(device_handle_ == nullptr);
  err = uvc_open(device_, &device_handle_);
  if (err != UVC_SUCCESS) {
    if (UVC_ERROR_ACCESS == err) {
      ROS_ERROR("Permission denied opening /dev/bus/usb/%03d/%03d", uvc_get_bus_number(device_),
                uvc_get_device_address(device_));
    } else {
      ROS_ERROR("Can't open /dev/bus/usb/%03d/%03d: %s (%d)", uvc_get_bus_number(device_),
                uvc_get_device_address(device_), uvc_strerror(err), err);
    }
    uvc_unref_device(device_);
    return;
  }
  uvc_set_status_callback(device_handle_, &UVCCameraDriver::autoControlsCallbackWrapper, this);
  CHECK_NOTNULL(device_handle_);
  CHECK_NOTNULL(device_);
  ROS_INFO_STREAM("open camera success");
  is_camera_opened_ = true;
}

void UVCCameraDriver::updateConfig(const UVCCameraConfig& config) { config_ = config; }

void UVCCameraDriver::setVideoMode() {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  auto uvc_format = UVCFrameFormatString(config_.format);
  int width = config_.width;
  int height = config_.height;
  int fps = config_.fps;
  uvc_error_t err;
  CHECK_NOTNULL(device_);
  CHECK_NOTNULL(device_handle_);
  for (int i = 0; i < 5; i++) {
    err = uvc_get_stream_ctrl_format_size(device_handle_, &ctrl_, uvc_format, width, height, fps);
    if (err == UVC_SUCCESS) {
      break;
    }
  }
  ROS_INFO_STREAM("set uvc mode " << width << "x" << height << "@" << fps << " format "
                                  << config_.format);
  if (err != UVC_SUCCESS) {
    ROS_ERROR_STREAM("set uvc ctrl error " << uvc_strerror(err));
    uvc_close(device_handle_);
    device_handle_ = nullptr;
    uvc_unref_device(device_);
    device_ = nullptr;
    return;
  }
}

void UVCCameraDriver::imageSubscribedCallback() {
  ROS_INFO_STREAM("UVCCameraDriver image subscribed");
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  startStreaming();
}

void UVCCameraDriver::imageUnsubscribedCallback() {
  ROS_INFO_STREAM("UVCCameraDriver image unsubscribed");
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  auto subscriber_count = image_publisher_.getNumSubscribers();
  if (subscriber_count == 0) {
    stopStreaming();
  }
}

void UVCCameraDriver::startStreaming() {
  if (is_streaming_started) {
    ROS_WARN_STREAM("UVCCameraDriver streaming is already started");
    return;
  }
  if (!is_camera_opened_) {
    ROS_WARN_STREAM("UVCCameraDriver camera is not opened");
    return;
  }
  CHECK_NOTNULL(device_);
  CHECK_NOTNULL(device_handle_);
  setVideoMode();
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  uvc_error_t stream_err =
      uvc_start_streaming(device_handle_, &ctrl_, &UVCCameraDriver::frameCallbackWrapper, this, 0);
  if (stream_err != UVC_SUCCESS) {
    ROS_ERROR_STREAM("uvc start streaming error " << uvc_strerror(stream_err) << " retry "
                                                  << config_.retry_count << " times");
    for (int i = 0; i < config_.retry_count; i++) {
      stream_err = uvc_start_streaming(device_handle_, &ctrl_,
                                       &UVCCameraDriver::frameCallbackWrapper, this, 0);
      if (stream_err == UVC_SUCCESS) {
        break;
      }
      usleep(100 * i);
    }
  }
  if (stream_err != UVC_SUCCESS) {
    uvc_perror(stream_err, "uvc_start_streaming");
    uvc_close(device_handle_);
    uvc_unref_device(device_);
    return;
  }

  if (frame_buffer_) {
    uvc_free_frame(frame_buffer_);
    frame_buffer_ = nullptr;
  }

  frame_buffer_ = uvc_allocate_frame(config_.width * config_.height * 3);
  CHECK_NOTNULL(frame_buffer_);
  is_streaming_started.store(true);
}

void UVCCameraDriver::stopStreaming() noexcept {
  if (!is_streaming_started) {
    ROS_WARN_STREAM("streaming is already stopped");
    return;
  }
  ROS_INFO_STREAM("stop uvc streaming");
  uvc_stop_streaming(device_handle_);
  if (frame_buffer_) {
    uvc_free_frame(frame_buffer_);
    frame_buffer_ = nullptr;
  }
  is_streaming_started.store(false);
}

int UVCCameraDriver::getResolutionX() const { return config_.width; }

int UVCCameraDriver::getResolutionY() const { return config_.height; }

void UVCCameraDriver::setupCameraControlService() {
  get_uvc_exposure_srv_ = nh_.advertiseService<GetInt32Request, GetInt32Response>(
      "get_uvc_exposure", [this](auto&& request, auto&& response) {
        response.success = this->getUVCExposureCb(request, response);
        return response.success;
      });
  set_uvc_exposure_srv_ = nh_.advertiseService<SetInt32Request, SetInt32Response>(
      "set_uvc_exposure", [this](auto&& request, auto&& response) {
        response.success = this->setUVCExposureCb(request, response);
        return response.success;
      });
  get_uvc_gain_srv_ = nh_.advertiseService<GetInt32Request, GetInt32Response>(
      "get_uvc_gain", [this](auto&& request, auto&& response) {
        response.success = this->getUVCGainCb(request, response);
        return response.success;
      });
  set_uvc_gain_srv_ = nh_.advertiseService<SetInt32Request, SetInt32Response>(
      "set_uvc_gain", [this](auto&& request, auto&& response) {
        response.success = this->setUVCGainCb(request, response);
        return response.success;
      });
  get_uvc_white_balance_srv_ = nh_.advertiseService<GetInt32Request, GetInt32Response>(
      "get_uvc_white_balance", [this](auto&& request, auto&& response) {
        response.success = this->getUVCWhiteBalanceCb(request, response);
        return response.success;
      });
  set_uvc_white_balance_srv_ = nh_.advertiseService<SetInt32Request, SetInt32Response>(
      "set_uvc_white_balance", [this](auto&& request, auto&& response) {
        response.success = this->setUVCWhiteBalanceCb(request, response);
        return response.success;
      });
  set_uvc_auto_exposure_srv_ =
      nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
          "set_uvc_auto_exposure", [this](auto&& request, auto&& response) {
            response.success = this->setUVCAutoExposureCb(request, response);
            return response.success;
          });
  set_uvc_auto_white_balance_srv_ =
      nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
          "set_uvc_auto_white_balance", [this](auto&& request, auto&& response) {
            response.success = this->setUVCAutoWhiteBalanceCb(request, response);
            return response.success;
          });

  get_uvc_mirror_srv_ = nh_.advertiseService<GetInt32Request, GetInt32Response>(
      "get_uvc_mirror", [this](auto&& request, auto&& response) {
        response.success = this->getUVCMirrorCb(request, response);
        return response.success;
      });
  set_uvc_mirror_srv_ = nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
      "set_uvc_mirror", [this](auto&& request, auto&& response) {
        response.success = this->setUVCMirrorCb(request, response);
        return response.success;
      });

  toggle_uvc_camera_srv_ =
      nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
          "toggle_uvc_camera", [this](auto&& request, auto&& response) {
            response.success = this->toggleUVCCamera(request, response);
            return response.success;
          });
  save_image_srv_ = nh_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
      "save_uvc_image", [this](auto&& request, auto&& response) {
        return this->saveImageCallback(request, response);
      });
}

sensor_msgs::CameraInfo UVCCameraDriver::getCameraInfo() {
  if (color_info_manager_ && color_info_manager_->isCalibrated()) {
    auto camera_info = color_info_manager_->getCameraInfo();
    camera_info.header.frame_id = config_.optical_frame_id;
    camera_info.header.stamp = ros::Time::now();
    return camera_info;
  } else {
    camera_info_.header.frame_id = config_.optical_frame_id;
    camera_info_.header.stamp = ros::Time::now();
    return camera_info_;
  }
}

enum uvc_frame_format UVCCameraDriver::UVCFrameFormatString(const std::string& format) {
  if (format == "uncompressed") {
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  } else if (format == "compressed") {
    return UVC_COLOR_FORMAT_COMPRESSED;
  } else if (format == "yuyv") {
    return UVC_COLOR_FORMAT_YUYV;
  } else if (format == "uyvy") {
    return UVC_COLOR_FORMAT_UYVY;
  } else if (format == "rgb") {
    return UVC_COLOR_FORMAT_RGB;
  } else if (format == "bgr") {
    return UVC_COLOR_FORMAT_BGR;
  } else if (format == "mjpeg") {
    return UVC_COLOR_FORMAT_MJPEG;
  } else if (format == "gray8") {
    return UVC_COLOR_FORMAT_GRAY8;
  } else {
    ROS_WARN_STREAM("Invalid Video Mode: " << format);
    ROS_WARN_STREAM("Continue using video mode: uncompressed");
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  }
}

void UVCCameraDriver::frameCallbackWrapper(uvc_frame_t* frame, void* ptr) {
  CHECK_NOTNULL(ptr);
  auto driver = static_cast<UVCCameraDriver*>(ptr);
  driver->frameCallback(frame);
}

void UVCCameraDriver::frameCallback(uvc_frame_t* frame) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(frame_buffer_);
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  static constexpr int unit_step = 3;
  sensor_msgs::Image image;
  image.width = frame->width;
  image.height = frame->height;
  image.step = image.width * unit_step;
  image.header.frame_id = frame_id_;
  image.header.stamp = ros::Time::now();
  image.data.resize(image.height * image.step);
  if (frame->frame_format == UVC_FRAME_FORMAT_BGR) {
    image.encoding = "bgr8";
    memcpy(&(image.data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_RGB) {
    image.encoding = "rgb8";
    memcpy(&(image.data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_UYVY) {
    image.encoding = "yuv422";
    memcpy(&(image.data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_YUYV) {
    // FIXME: uvc_any2bgr does not work on "yuyv" format, so use uvc_yuyv2bgr directly.
    uvc_error_t conv_ret = uvc_yuyv2bgr(frame, frame_buffer_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    image.encoding = "bgr8";
    memcpy(&(image.data[0]), frame_buffer_->data, frame_buffer_->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG) {
    // Enable mjpeg support despite uvs_any2bgr shortcoming
    //  https://github.com/ros-drivers/libuvc_ros/commit/7508a09f
    uvc_error_t conv_ret = uvc_mjpeg2rgb(frame, frame_buffer_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    image.encoding = "rgb8";
    memcpy(&(image.data[0]), frame_buffer_->data, frame_buffer_->data_bytes);
  } else {
    uvc_error_t conv_ret = uvc_any2bgr(frame, frame_buffer_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    image.encoding = "bgr8";
    memcpy(&(image.data[0]), frame_buffer_->data, frame_buffer_->data_bytes);
  }

  if (roi_.x != -1 && roi_.y != -1 && roi_.width != -1 && roi_.height != -1) {
    auto cv_image_ptr = cv_bridge::toCvCopy(image);
    auto cv_img = cv_image_ptr->image;
    cv::Rect roi(roi_.x, roi_.y, roi_.width, roi_.height);
    cv::Mat dst(cv_image_ptr->image, roi);
    cv_image_ptr->image = dst;
    image = *(cv_image_ptr->toImageMsg());
  }
  if (uvc_flip_) {
    auto cv_image_ptr = cv_bridge::toCvCopy(image);
    auto cv_img = cv_image_ptr->image;
    cv::flip(cv_img, cv_img, 1);
    cv_image_ptr->image = cv_img;
    image = *(cv_image_ptr->toImageMsg());
  }
  auto camera_info = getCameraInfo();
  camera_info.header.stamp = ros::Time::now();
  camera_info_publisher_.publish(camera_info);
  image.header.frame_id = config_.optical_frame_id;
  image.header.stamp = camera_info.header.stamp;
  image_publisher_.publish(image);
  if (save_image_) {
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(image);
    auto now = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
    auto current_path = boost::filesystem::current_path().string();
    std::string filename = current_path + "/image/uvc_color_" + std::to_string(image.width) + "x" +
                           std::to_string(image.height) + "_" + ss.str() + ".jpg";
    if (!boost::filesystem::exists(current_path + "/image")) {
      boost::filesystem::create_directory(current_path + "/image");
    }
    ROS_INFO_STREAM("Saving image to " << filename);
    cv::imwrite(filename, cv_image_ptr->image);
    save_image_ = false;
  }
}

void UVCCameraDriver::autoControlsCallback(enum uvc_status_class status_class, int event,
                                           int selector, enum uvc_status_attribute status_attribute,
                                           void* data, size_t data_len) {
  char buff[256];
  CHECK(data_len < 256);
  (void)data;
  sprintf(buff, "Controls callback. class: %d, event: %d, selector: %d, attr: %d, data_len: %zu\n",
          status_class, event, selector, status_attribute, data_len);
  ROS_INFO_STREAM(buff);
}

void UVCCameraDriver::autoControlsCallbackWrapper(enum uvc_status_class status_class, int event,
                                                  int selector,
                                                  enum uvc_status_attribute status_attribute,
                                                  void* data, size_t data_len, void* ptr) {
  CHECK_NOTNULL(ptr);
  auto driver = static_cast<UVCCameraDriver*>(ptr);
  driver->autoControlsCallback(status_class, event, selector, status_attribute, data, data_len);
}

bool UVCCameraDriver::getUVCExposureCb(GetInt32Request& request, GetInt32Response& response) {
  (void)request;
  uint32_t data;
  uvc_error_t err = uvc_get_exposure_abs(device_handle_, &data, UVC_GET_CUR);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    ROS_ERROR_STREAM("getUVCExposureCb " << msg);
    response.message = msg;
    return false;
  }
  response.data = static_cast<int>(data);
  return true;
}

bool UVCCameraDriver::setUVCExposureCb(SetInt32Request& request, SetInt32Response& response) {
  if (request.data == 0) {
    ROS_ERROR("set auto mode");
    uvc_error_t err = uvc_set_ae_mode(device_handle_, 8);  // 8才是自动8: aperture priority mode
    ROS_ERROR("ret :%d", (int)err);
    return true;
  }
  uint32_t max_expo, min_expo;
  uvc_get_exposure_abs(device_handle_, &max_expo, UVC_GET_MAX);
  uvc_get_exposure_abs(device_handle_, &min_expo, UVC_GET_MIN);
  if (request.data < static_cast<int>(min_expo) || request.data > static_cast<int>(max_expo)) {
    std::stringstream ss;
    ss << "Exposure value out of range. Min: " << min_expo << ", Max: " << max_expo;
    response.message = ss.str();
    ROS_ERROR_STREAM(response.message);
    return false;
  }
  uvc_set_ae_mode(
      device_handle_,
      1);  // mode 1: manual mode; 2: auto mode; 4: shutter priority mode; 8: aperture priority mode

  uvc_error_t err = uvc_set_exposure_abs(device_handle_, request.data);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    ROS_ERROR_STREAM("setUVCExposureCb " << msg);
    response.message = msg;
    return false;
  }
  return true;
}

bool UVCCameraDriver::getUVCGainCb(GetInt32Request& request, GetInt32Response& response) {
  (void)request;
  uint16_t gain;
  uvc_error_t err = uvc_get_gain(device_handle_, &gain, UVC_GET_CUR);
  response.data = gain;
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    ROS_ERROR_STREAM("getUVCGainCb " << msg);
    response.message = msg;
    return false;
  }
  return true;
}

bool UVCCameraDriver::setUVCGainCb(SetInt32Request& request, SetInt32Response& response) {
  uint16_t min_gain, max_gain;
  uvc_get_gain(device_handle_, &min_gain, UVC_GET_MIN);
  uvc_get_gain(device_handle_, &max_gain, UVC_GET_MAX);
  if (request.data < min_gain || request.data > max_gain) {
    std::stringstream ss;
    ss << "Gain must be between " << min_gain << " and " << max_gain;
    response.message = ss.str();
    ROS_ERROR_STREAM(response.message);
    return false;
  }
  uvc_error_t err = uvc_set_gain(device_handle_, request.data);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    ROS_ERROR_STREAM("setUVCGainCb " << msg);
    response.message = msg;
    return false;
  }
  return true;
}

bool UVCCameraDriver::getUVCWhiteBalanceCb(GetInt32Request& request, GetInt32Response& response) {
  (void)request;
  uint16_t data;
  uvc_error_t err = uvc_get_white_balance_temperature(device_handle_, &data, UVC_GET_CUR);
  response.data = data;
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    ROS_ERROR_STREAM("getUVCWhiteBalanceCb " << msg);
    response.message = msg;
    return false;
  }
  return true;
}

bool UVCCameraDriver::setUVCWhiteBalanceCb(SetInt32Request& request, SetInt32Response& response) {
  if (request.data == 0) {
    uvc_set_white_balance_temperature_auto(device_handle_, 1);
    return true;
  }
  uvc_set_white_balance_temperature_auto(device_handle_, 0);  // 0: manual, 1: auto
  uint8_t data[4];
  INT_TO_DW(request.data, data);
  int unit = uvc_get_processing_units(device_handle_)->bUnitID;
  int control = UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL;
  int min_white_balance = UVCGetControl(control, unit, sizeof(int16_t), UVC_GET_MIN);
  int max_white_balance = UVCGetControl(control, unit, sizeof(int16_t), UVC_GET_MAX);
  if (request.data < min_white_balance || request.data > max_white_balance) {
    std::stringstream ss;
    ss << "Please set white balance between " << min_white_balance << " and " << max_white_balance;
    response.message = ss.str();
    ROS_ERROR_STREAM(ss.str());
    return false;
  }
  int ret = uvc_set_ctrl(device_handle_, unit, control, data, sizeof(int32_t));
  if (ret != sizeof(int32_t)) {
    auto err = static_cast<uvc_error_t>(ret);
    std::stringstream ss;
    ss << "set white balance failed " << uvc_strerror(err);
    ROS_ERROR_STREAM(ss.str());
    response.message = ss.str();
    return false;
  }
  return true;
}

bool UVCCameraDriver::setUVCAutoExposureCb(std_srvs::SetBoolRequest& request,
                                           std_srvs::SetBoolResponse& response) {
  (void)response;
  if (request.data) {
    uvc_set_ae_mode(device_handle_, 8);
  } else {
    uvc_set_ae_mode(device_handle_, 1);
  }
  return true;
}

bool UVCCameraDriver::setUVCAutoWhiteBalanceCb(std_srvs::SetBoolRequest& request,
                                               std_srvs::SetBoolResponse& response) {
  if (request.data) {
    uvc_set_white_balance_temperature_auto(device_handle_, 1);
  } else {
    uvc_set_white_balance_temperature_auto(device_handle_, 0);
  }
  response.success = true;
  return true;
}

bool UVCCameraDriver::getUVCMirrorCb(GetInt32Request& request, GetInt32Response& response) {
  (void)request;
  int16_t mirror;
  uvc_error_t err = uvc_get_roll_abs(device_handle_, &mirror, UVC_GET_CUR);
  response.data = mirror;
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    ROS_ERROR_STREAM("getUVCMirrorCb " << msg);
    response.message = msg;
    return false;
  }
  return true;
}

bool UVCCameraDriver::setUVCMirrorCb(std_srvs::SetBoolRequest& request,
                                     std_srvs::SetBoolResponse& response) {
  (void)response;
  uvc_flip_ = request.data;
  return true;
}

bool UVCCameraDriver::toggleUVCCamera(std_srvs::SetBoolRequest& request,
                                      std_srvs::SetBoolResponse& response) {
  (void)response;
  if (request.data) {
    startStreaming();
  } else {
    stopStreaming();
  }
  return true;
}

bool UVCCameraDriver::saveImageCallback(std_srvs::EmptyRequest& request,
                                        std_srvs::EmptyResponse& response) {
  (void)request;
  (void)response;
  save_image_ = true;
  return true;
}

int UVCCameraDriver::UVCGetControl(int control, int unit, int len, uvc_req_code req_code) {
  uint8_t data[4];
  int ret = uvc_get_ctrl(device_handle_, unit, control, data, len, req_code);
  if (ret < 0) {
    auto err = static_cast<uvc_error>(ret);
    ROS_ERROR("Failed to get control %d on unit %d: %s", control, unit, uvc_strerror(err));
    return -1;
  }
  return SW_TO_SHORT(data);
}

}  // namespace astra_camera
