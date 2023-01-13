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

#include <utility>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "astra_camera/uvc_camera_driver.h"
#include "astra_camera/utils.h"

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

namespace astra_camera {
std::ostream& operator<<(std::ostream& os, const UVCCameraConfig& config) {
  os << "vendor_id: " << std::hex << config.vendor_id << std::endl;
  os << "product_id: " << std::hex << config.product_id << std::endl;
  os << "width: " << std::dec << config.width << std::endl;
  os << "height: " << config.height << std::endl;
  os << "fps: " << config.fps << std::endl;
  os << "serial_number: " << config.serial_number << std::endl;
  os << "format: " << config.format << std::endl;
  os << "frame_id: " << config.frame_id << std::endl;
  os << "optical_frame_id : " << config.optical_frame_id << std::endl;
  return os;
}

UVCCameraDriver::UVCCameraDriver(rclcpp::Node* node, std::shared_ptr<Parameters> parameters,
                                 const std::string& serial_number)
    : node_(node), logger_(node_->get_logger()), parameters_(parameters) {
  auto err = uvc_init(&ctx_, nullptr);
  if (err != UVC_SUCCESS) {
    uvc_perror(err, "ERROR: uvc_init");
    RCLCPP_ERROR_STREAM(logger_, "init uvc context failed, exit");
    exit(err);
  }
  config_.serial_number = serial_number;
  setAndGetNodeParameter(parameters_, config_.vendor_id, "uvc_camera.vid", 0);
  setAndGetNodeParameter(parameters_, config_.product_id, "uvc_camera.pid", 0);
  setAndGetNodeParameter(parameters_, config_.width, "color_width", 640);
  setAndGetNodeParameter(parameters_, config_.height, "color_height", 480);
  setAndGetNodeParameter(parameters_, config_.fps, "color_fps", 30);
  setAndGetNodeParameter<std::string>(parameters_, config_.format, "uvc_camera.format", "mjpeg");
  setAndGetNodeParameter(parameters_, config_.retry_count, "uvc_camera.retry_count", 500);
  setAndGetNodeParameter(parameters_, roi_.x, "color_roi.x", -1);
  setAndGetNodeParameter(parameters_, roi_.y, "color_roi.y", -1);
  setAndGetNodeParameter(parameters_, roi_.width, "color_roi.width", -1);
  setAndGetNodeParameter(parameters_, roi_.height, "color_roi.height", -1);
  setAndGetNodeParameter(parameters_, camera_name_, "camera_name", camera_name_);
  config_.frame_id = camera_name_ + "_color_frame";
  config_.optical_frame_id = camera_name_ + "_color_optical_frame";
  setupCameraControlService();
  image_publisher_ =
      node_->create_publisher<sensor_msgs::msg::Image>("color/image_raw", rclcpp::QoS{1});
  camera_info_publisher_ =
      node_->create_publisher<sensor_msgs::msg::CameraInfo>("color/camera_info", rclcpp::QoS{1});
  openCamera();
}

UVCCameraDriver::~UVCCameraDriver() {
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
  RCLCPP_INFO_STREAM(logger_, "open uvc camera");
  uvc_error_t err;
  auto serial_number = config_.serial_number.empty() ? nullptr : config_.serial_number.c_str();
  CHECK(device_ == nullptr);
  err = uvc_find_device(ctx_, &device_, config_.vendor_id, config_.product_id, serial_number);
  if (err != UVC_SUCCESS) {
    uvc_perror(err, "ERROR: uvc_find_device");
    RCLCPP_ERROR_STREAM(logger_,
                        "find uvc device failed, retry " << config_.retry_count << " times");
    for (int i = 0; i < config_.retry_count; i++) {
      err = uvc_find_device(ctx_, &device_, config_.vendor_id, config_.product_id, serial_number);
      if (err == UVC_SUCCESS) {
        break;
      }
      usleep(100 * i);
    }
  }
  RCLCPP_INFO_STREAM(logger_, "uvc config: " << config_);
  if (err != UVC_SUCCESS) {
    RCLCPP_ERROR_STREAM(logger_, "Find device error " << uvc_strerror(err));
    uvc_unref_device(device_);
    exit(-1);
  }
  CHECK(device_handle_ == nullptr);
  err = uvc_open(device_, &device_handle_);
  if (err != UVC_SUCCESS) {
    if (UVC_ERROR_ACCESS == err) {
      RCLCPP_ERROR(logger_, "Permission denied opening /dev/bus/usb/%03d/%03d",
                   uvc_get_bus_number(device_), uvc_get_device_address(device_));
    } else {
      RCLCPP_ERROR(logger_, "Can't open /dev/bus/usb/%03d/%03d: %s (%d)",
                   uvc_get_bus_number(device_), uvc_get_device_address(device_), uvc_strerror(err),
                   err);
    }
    uvc_unref_device(device_);
    return;
  }
  uvc_set_status_callback(device_handle_, &UVCCameraDriver::autoControlsCallbackWrapper, this);
}

void UVCCameraDriver::setVideoMode() {
  auto uvc_format = UVCFrameFormatString(config_.format);
  int width = config_.width;
  int height = config_.height;
  int fps = config_.fps;
  uvc_error_t err;
  for (int i = 0; i < 5; i++) {
    err = uvc_get_stream_ctrl_format_size(device_handle_, &ctrl_, uvc_format, width, height, fps);
    if (err == UVC_SUCCESS) {
      break;
    }
  }
  RCLCPP_INFO_STREAM(logger_, "set uvc mode" << width << "x" << height << "@" << fps << " format "
                                             << magic_enum::enum_name(uvc_format));
  if (err != UVC_SUCCESS) {
    RCLCPP_ERROR_STREAM(logger_, "set uvc ctrl error " << uvc_strerror(err));
    uvc_close(device_handle_);
    device_handle_ = nullptr;
    uvc_unref_device(device_);
    device_ = nullptr;
    return;
  }
}

void UVCCameraDriver::startStreaming() {
  if (is_streaming_started) {
    RCLCPP_WARN_STREAM(logger_, "streaming is already started");
    return;
  }
  setVideoMode();
  uvc_error_t stream_err =
      uvc_start_streaming(device_handle_, &ctrl_, &UVCCameraDriver::frameCallbackWrapper, this, 0);
  if (stream_err != UVC_SUCCESS) {
    RCLCPP_ERROR_STREAM(logger_, "uvc start streaming error " << uvc_strerror(stream_err)
                                                              << " retry " << config_.retry_count
                                                              << " times");
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
  }

  frame_buffer_ = uvc_allocate_frame(config_.width * config_.height * 3);
  CHECK_NOTNULL(frame_buffer_);
  is_streaming_started.store(true);
}

void UVCCameraDriver::stopStreaming() {
  if (!is_streaming_started) {
    RCLCPP_WARN_STREAM(logger_, "streaming is already stopped");
    return;
  }
  RCLCPP_INFO_STREAM(logger_, "stop uvc streaming");
  uvc_stop_streaming(device_handle_);
  if (frame_buffer_) {
    uvc_free_frame(frame_buffer_);
    frame_buffer_ = nullptr;
  }
  is_streaming_started.store(false);
}

void UVCCameraDriver::setupCameraControlService() {
  get_uvc_exposure_srv_ = node_->create_service<GetInt32>(
      "get_uvc_exposure", [this](const std::shared_ptr<GetInt32::Request> request,
                                 std::shared_ptr<GetInt32::Response> response) {
        response->success = getUVCExposureCb(request, response);
      });
  set_uvc_exposure_srv_ = node_->create_service<SetInt32>(
      "set_uvc_exposure", [this](const std::shared_ptr<SetInt32::Request> request,
                                 std::shared_ptr<SetInt32::Response> response) {
        response->success = setUVCExposureCb(request, response);
      });
  get_uvc_gain_srv_ = node_->create_service<GetInt32>(
      "get_uvc_gain", [this](const std::shared_ptr<GetInt32::Request> request,
                             std::shared_ptr<GetInt32::Response> response) {
        response->success = getUVCGainCb(request, response);
      });
  set_uvc_gain_srv_ = node_->create_service<SetInt32>(
      "set_uvc_gain", [this](const std::shared_ptr<SetInt32::Request> request,
                             std::shared_ptr<SetInt32::Response> response) {
        response->success = setUVCGainCb(request, response);
      });
  get_uvc_white_balance_srv_ = node_->create_service<GetInt32>(
      "get_uvc_white_balance", [this](const std::shared_ptr<GetInt32::Request> request,
                                      std::shared_ptr<GetInt32::Response> response) {
        response->success = getUVCWhiteBalanceCb(request, response);
      });
  set_uvc_white_balance_srv_ = node_->create_service<SetInt32>(
      "set_uvc_white_balance", [this](const std::shared_ptr<SetInt32::Request> request,
                                      std::shared_ptr<SetInt32::Response> response) {
        response->success = setUVCWhiteBalanceCb(request, response);
      });
  set_uvc_auto_exposure_srv_ = node_->create_service<SetBool>(
      "set_uvc_auto_exposure", [this](const std::shared_ptr<SetBool::Request> request,
                                      std::shared_ptr<SetBool::Response> response) {
        response->success = setUVCAutoExposureCb(request, response);
      });
  set_uvc_auto_white_balance_srv_ = node_->create_service<SetBool>(
      "set_uvc_auto_white_balance", [this](const std::shared_ptr<SetBool::Request> request,
                                           std::shared_ptr<SetBool::Response> response) {
        response->success = setUVCAutoWhiteBalanceCb(request, response);
      });

  get_uvc_mirror_srv_ = node_->create_service<GetInt32>(
      "get_uvc_mirror", [this](const std::shared_ptr<GetInt32::Request> request,
                               std::shared_ptr<GetInt32::Response> response) {
        response->success = getUVCMirrorCb(request, response);
      });
  set_uvc_mirror_srv_ = node_->create_service<SetBool>(
      "set_uvc_mirror", [this](const std::shared_ptr<SetBool::Request> request,
                               std::shared_ptr<SetBool::Response> response) {
        response->success = setUVCMirrorCb(request, response);
      });

  toggle_uvc_camera_srv_ = node_->create_service<SetBool>(
      "toggle_uvc_camera", [this](const std::shared_ptr<SetBool::Request> request,
                                  std::shared_ptr<SetBool::Response> response) {
        response->success = toggleUVCCamera(request, response);
      });
  get_camera_info_cli_ = node_->create_client<GetCameraInfo>("get_camera_info");
}

sensor_msgs::msg::CameraInfo UVCCameraDriver::getCameraInfo() {
  using namespace std::chrono_literals;
  while (!get_camera_info_cli_->wait_for_service(1s)) {
    CHECK(rclcpp::ok()) << "Interrupted while waiting for the service. Exiting.";
  }
  auto request = std::make_shared<GetCameraInfo::Request>();
  auto future = get_camera_info_cli_->async_send_request(request);
  const auto& result = future.get();
  camera_info_ = result->info;
  camera_info_->header.frame_id = config_.optical_frame_id;
  return result->info;
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
    RCLCPP_WARN_STREAM(logger_, "Invalid Video Mode: " << format);
    RCLCPP_WARN_STREAM(logger_, "Continue using video mode: uncompressed");
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  }
}

void UVCCameraDriver::frameCallback(uvc_frame_t* frame) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(frame_buffer_);
  static constexpr int unit_step = 3;
  sensor_msgs::msg::Image image;
  image.width = frame->width;
  image.height = frame->height;
  image.step = image.width * unit_step;
  image.header.frame_id = config_.optical_frame_id;
  image.header.stamp = node_->now();
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
  if (!camera_info_.has_value()) {
    getCameraInfo();
  }
  if (roi_.x != -1 && roi_.y != -1 && roi_.width != -1 && roi_.height != -1) {
    auto cv_image_ptr = cv_bridge::toCvCopy(image);
    auto cv_img = cv_image_ptr->image;
    cv::Rect roi(roi_.x, roi_.y, roi_.width, roi_.height);
    cv::Mat dst(cv_image_ptr->image, roi);
    cv_image_ptr->image = dst;
    image = *(cv_image_ptr->toImageMsg());
    image.header.frame_id = config_.optical_frame_id;
    image.header.stamp = node_->now();
  }
  if (camera_info_.has_value()) {
    camera_info_->header.stamp = node_->now();
    camera_info_->height = image.height;
    camera_info_->width = image.width;
    camera_info_publisher_->publish(*camera_info_);
  }
  image_publisher_->publish(image);
}

void UVCCameraDriver::frameCallbackWrapper(uvc_frame_t* frame, void* ptr) {
  CHECK_NOTNULL(ptr);
  auto driver = static_cast<UVCCameraDriver*>(ptr);
  driver->frameCallback(frame);
}

void UVCCameraDriver::autoControlsCallbackWrapper(enum uvc_status_class status_class, int event,
                                                  int selector,
                                                  enum uvc_status_attribute status_attribute,
                                                  void* data, size_t data_len, void* ptr) {
  CHECK_NOTNULL(ptr);
  auto driver = static_cast<UVCCameraDriver*>(ptr);
  driver->autoControlsCallback(status_class, event, selector, status_attribute, data, data_len);
}

void UVCCameraDriver::autoControlsCallback(enum uvc_status_class status_class, int event,
                                           int selector, enum uvc_status_attribute status_attribute,
                                           void* data, size_t data_len) {
  char buff[256];
  CHECK(data_len < 256);
  (void)data;
  sprintf(buff, "Controls callback. class: %d, event: %d, selector: %d, attr: %d, data_len: %zu\n",
          status_class, event, selector, status_attribute, data_len);
  RCLCPP_INFO_STREAM(logger_, buff);
}

bool UVCCameraDriver::getUVCExposureCb(const std::shared_ptr<GetInt32::Request>& request,
                                       std::shared_ptr<GetInt32::Response>& response) {
  (void)request;
  uint32_t data;
  uvc_error_t err = uvc_get_exposure_abs(device_handle_, &data, UVC_GET_CUR);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->message = msg;
    return false;
  }
  response->data = static_cast<int>(data);
  return true;
}

bool UVCCameraDriver::setUVCExposureCb(const std::shared_ptr<SetInt32::Request>& request,
                                       std::shared_ptr<SetInt32::Response>& response) {
  if (request->data == 0) {
    RCLCPP_ERROR_STREAM(logger_, "set auto mode");
    uvc_error_t err = uvc_set_ae_mode(device_handle_, 8);  // 8才是自动8: aperture priority mode
    RCLCPP_ERROR_STREAM(logger_, "uvc_set_ae_mode error " << uvc_strerror(err));
    return true;
  }
  uint32_t max_expo, min_expo;
  uvc_get_exposure_abs(device_handle_, &max_expo, UVC_GET_MAX);
  uvc_get_exposure_abs(device_handle_, &min_expo, UVC_GET_MIN);
  if (request->data < static_cast<int>(min_expo) || request->data > static_cast<int>(max_expo)) {
    std::stringstream ss;
    ss << "Exposure value out of range. Min: " << min_expo << ", Max: " << max_expo;
    response->message = ss.str();
    RCLCPP_ERROR_STREAM(logger_, response->message);
    return false;
  }
  uvc_set_ae_mode(
      device_handle_,
      1);  // mode 1: manual mode; 2: auto mode; 4: shutter priority mode; 8: aperture priority mode

  uvc_error_t err = uvc_set_exposure_abs(device_handle_, request->data);
  return (err == UVC_SUCCESS);
}

bool UVCCameraDriver::getUVCGainCb(const std::shared_ptr<GetInt32::Request>& request,
                                   std::shared_ptr<GetInt32::Response>& response) {
  (void)request;
  uint16_t data;
  auto err = uvc_get_gain(device_handle_, &data, UVC_GET_CUR);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->message = msg;
    return false;
  }
  response->data = data;
  return true;
}

bool UVCCameraDriver::setUVCGainCb(const std::shared_ptr<SetInt32::Request>& request,
                                   std::shared_ptr<SetInt32::Response>& response) {
  uint16_t min_gain, max_gain;
  uvc_get_gain(device_handle_, &min_gain, UVC_GET_MIN);
  uvc_get_gain(device_handle_, &max_gain, UVC_GET_MAX);
  if (request->data < min_gain || request->data > max_gain) {
    std::stringstream ss;
    ss << "Gain must be between " << min_gain << " and " << max_gain;
    response->message = ss.str();
    RCLCPP_ERROR_STREAM(logger_, response->message);
    return false;
  }
  uvc_error_t err = uvc_set_gain(device_handle_, request->data);
  return (err == UVC_SUCCESS);
}

bool UVCCameraDriver::getUVCWhiteBalanceCb(const std::shared_ptr<GetInt32::Request>& request,
                                           std::shared_ptr<GetInt32::Response>& response) {
  (void)request;
  uint16_t data;
  auto err = uvc_get_white_balance_temperature(device_handle_, &data, UVC_GET_CUR);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->message = msg;
    return false;
  }
  response->data = data;
  return true;
}

bool UVCCameraDriver::setUVCWhiteBalanceCb(const std::shared_ptr<SetInt32::Request>& request,
                                           std::shared_ptr<SetInt32::Response>& response) {
  if (request->data == 0) {
    uvc_set_white_balance_temperature_auto(device_handle_, 1);
    return true;
  }
  uvc_set_white_balance_temperature_auto(device_handle_, 0);  // 0: manual, 1: auto
  uint8_t data[4];
  INT_TO_DW(request->data, data);
  int unit = uvc_get_processing_units(device_handle_)->bUnitID;
  int control = UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL;
  int min_white_balance = UVCGetControl(control, unit, sizeof(int16_t), UVC_GET_MIN);
  int max_white_balance = UVCGetControl(control, unit, sizeof(int16_t), UVC_GET_MAX);
  if (request->data < min_white_balance || request->data > max_white_balance) {
    std::stringstream ss;
    ss << "Please set white balance between " << min_white_balance << "and " << max_white_balance;
    response->message = ss.str();
    RCLCPP_ERROR_STREAM(logger_, ss.str());
    return false;
  }
  int ret = uvc_set_ctrl(device_handle_, unit, control, data, sizeof(int32_t));
  if (ret != sizeof(int32_t)) {
    auto err = static_cast<uvc_error_t>(ret);
    std::stringstream ss;
    ss << "set white balance failed " << uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, ss.str());
    response->message = ss.str();
    return false;
  }
  return true;
}

bool UVCCameraDriver::setUVCAutoExposureCb(const std::shared_ptr<SetBool::Request>& request,
                                           std::shared_ptr<SetBool::Response>& response) {
  uint8_t mode = request->data ? 8 : 1;  // 8 auto, 1manual
  auto err = uvc_set_ae_mode(device_handle_, mode);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->success = false;
    response->message = msg;
    return false;
  }
  return true;
}

bool UVCCameraDriver::setUVCAutoWhiteBalanceCb(const std::shared_ptr<SetBool::Request>& request,
                                               std::shared_ptr<SetBool::Response>& response) {
  auto err = uvc_set_white_balance_temperature_auto(device_handle_, request->data);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->success = false;
    response->message = msg;
    return false;
  }
  return true;
}

bool UVCCameraDriver::getUVCMirrorCb(const std::shared_ptr<GetInt32::Request>& request,
                                     std::shared_ptr<GetInt32::Response>& response) {
  (void)request;
  int16_t roll;
  auto err = uvc_get_roll_abs(device_handle_, &roll, UVC_GET_CUR);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->message = msg;
    return false;
  }
  return true;
}

bool UVCCameraDriver::setUVCMirrorCb(const std::shared_ptr<SetBool::Request>& request,
                                     std::shared_ptr<SetBool::Response>& response) {
  int16_t roll;
  if (request->data) {
    roll = config_.product_id == DEEYEA_PID ? 3 : 1;
  } else {
    roll = 0;
  }
  CHECK_NOTNULL(device_handle_);
  auto err = uvc_set_roll_abs(device_handle_, roll);
  if (err != UVC_SUCCESS) {
    auto msg = uvc_strerror(err);
    RCLCPP_ERROR_STREAM(logger_, "setUVCMirrorCb " << msg);
    response->success = false;
    response->message = msg;
    return false;
  }
  return true;
}

bool UVCCameraDriver::toggleUVCCamera(const std::shared_ptr<SetBool::Request>& request,
                                      std::shared_ptr<SetBool::Response>& response) {
  (void)response;
  if (request->data) {
    startStreaming();
  } else {
    stopStreaming();
  }
  return true;
}

int UVCCameraDriver::UVCGetControl(int control, int unit, int len, uvc_req_code req_code) {
  uint8_t data[4];
  int ret = uvc_get_ctrl(device_handle_, unit, control, data, len, req_code);
  if (ret < 0) {
    auto err = static_cast<uvc_error>(ret);
    RCLCPP_ERROR(logger_, "Failed to get control %d on unit %d: %s", control, unit,
                 uvc_strerror(err));
    return -1;
  }
  return SW_TO_SHORT(data);
}

int UVCCameraDriver::getResolutionX() const { return config_.width; }

int UVCCameraDriver::getResolutionY() const { return config_.height; }
}  // namespace astra_camera
