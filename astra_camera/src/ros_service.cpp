
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

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <thread>

#include "astra_camera/utils.h"
#include "astra_camera/ob_camera_node.h"

namespace astra_camera {

void OBCameraNode::setupCameraCtrlServices() {
  using std_srvs::srv::SetBool;
  for (auto stream_index : IMAGE_STREAMS) {
    if (!enable_[stream_index]) {
      RCLCPP_INFO_STREAM(logger_, stream_name_[stream_index] << " is not enable");
      continue;
    }
    auto stream_name = stream_name_[stream_index];
    std::string service_name = "get_" + stream_name + "_exposure";
    get_exposure_srv_[stream_index] = node_->create_service<GetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<GetInt32::Request> request,
                                            std::shared_ptr<GetInt32::Response> response) {
          response->success = getExposureCallback(request, response, stream_index);
        });

    service_name = "set_" + stream_name + "_exposure";
    set_exposure_srv_[stream_index] = node_->create_service<SetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetInt32::Request> request,
                                            std::shared_ptr<SetInt32::Response> response) {
          response->success = setExposureCallback(request, response, stream_index);
        });
    service_name = "get_" + stream_name + "_gain";
    get_gain_srv_[stream_index] = node_->create_service<GetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<GetInt32::Request> request,
                                            std::shared_ptr<GetInt32::Response> response) {
          response->success = getGainCallback(request, response, stream_index);
        });

    service_name = "set_" + stream_name + "_gain";
    set_gain_srv_[stream_index] = node_->create_service<SetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetInt32::Request> request,
                                            std::shared_ptr<SetInt32::Response> response) {
          response->success = setGainCallback(request, response, stream_index);
        });
    service_name = "set_" + stream_name + "_auto_exposure";
    set_auto_exposure_srv_[stream_index] = node_->create_service<SetBool>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetBool::Request> request,
                                            std::shared_ptr<SetBool::Response> response) {
          response->success = setAutoExposureCallback(request, response, stream_index);
        });

    service_name = "toggle_" + stream_name;

    toggle_sensor_srv_[stream_index] = node_->create_service<SetBool>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetBool::Request> request,
                                            std::shared_ptr<SetBool::Response> response) {
          response->success = toggleSensorCallback(request, response, stream_index);
        });
    service_name = "get_" + stream_name + "_auto_white_balance";
    get_white_balance_srv_[stream_index] = node_->create_service<GetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<GetInt32::Request> request,
                                            std::shared_ptr<GetInt32::Response> response) {
          response->success = getAutoWhiteBalanceEnabledCallback(request, response, stream_index);
        });
    service_name = "set_" + stream_name + "_auto_white_balance";

    set_white_balance_srv_[stream_index] = node_->create_service<SetBool>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetBool::Request> request,
                                            std::shared_ptr<SetBool::Response> response) {
          response->success = setAutoWhiteBalanceEnabledCallback(request, response, stream_index);
        });
    service_name = "set_" + stream_name + "_mirror";
    set_mirror_srv_[stream_index] = node_->create_service<SetBool>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetBool::Request> request,
                                            std::shared_ptr<SetBool::Response> response) {
          response->success = setMirrorCallback(request, response, stream_index);
        });
    service_name = "get_" + stream_name + "_supported_video_modes";
    get_supported_video_modes_srv_[stream_index] = node_->create_service<GetString>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<GetString::Request> request,
                                            std::shared_ptr<GetString::Response> response) {
          response->success = getSupportedVideoModesCallback(request, response, stream_index);
        });
  }
  set_fan_enable_srv_ = node_->create_service<SetBool>(
      "set_fan_mode", [this](const std::shared_ptr<SetBool::Request> request,
                             std::shared_ptr<SetBool::Response> response) {
        response->success = setFanCallback(request, response);
      });

  set_laser_enable_srv_ = node_->create_service<SetBool>(
      "set_laser_enable", [this](const std::shared_ptr<SetBool::Request> request,
                                 std::shared_ptr<SetBool::Response> response) {
        response->success = setLaserEnableCallback(request, response);
      });
  set_ldp_enable_srv_ = node_->create_service<SetBool>(
      "set_ldp_enable", [this](const std::shared_ptr<SetBool::Request> request,
                               std::shared_ptr<SetBool::Response> response) {
        response->success = setLdpEnableCallback(request, response);
      });

  get_device_srv_ = node_->create_service<GetDeviceInfo>(
      "get_device_info", [this](const std::shared_ptr<GetDeviceInfo::Request> request,
                                std::shared_ptr<GetDeviceInfo::Response> response) {
        response->success = getDeviceInfoCallback(request, response);
      });
  get_sdk_version_srv_ = node_->create_service<GetString>(
      "get_sdk_version", [this](const std::shared_ptr<GetString::Request> request,
                                std::shared_ptr<GetString::Response> response) {
        response->success = getSDKVersion(request, response);
      });
  get_camera_params_srv_ = node_->create_service<GetCameraParams>(
      "get_camera_params", [this](const std::shared_ptr<GetCameraParams::Request> request,
                                  std::shared_ptr<GetCameraParams::Response> response) {
        response->success = getCameraParamsCallback(request, response);
      });
  get_camera_info_srv_ = node_->create_service<GetCameraInfo>(
      "get_camera_info", [this](const std::shared_ptr<GetCameraInfo::Request> request,
                                std::shared_ptr<GetCameraInfo::Response> response) {
        response->success = getCameraInfoCallback(request, response);
      });
}

bool OBCameraNode::setExposureCallback(const std::shared_ptr<SetInt32::Request>& request,
                                       std::shared_ptr<SetInt32::Response>& response,
                                       const stream_index_pair& stream_index) {
  if (stream_index == COLOR) {
    auto stream = streams_.at(stream_index);
    auto camera_settings = stream->getCameraSettings();
    if (camera_settings == nullptr) {
      response->success = false;
      response->message = stream_name_[stream_index] + " Camera settings not available";
      return false;
    }
    auto rc = camera_settings->setExposure(request->data);
    std::stringstream ss;
    if (rc != openni::STATUS_OK) {
      ss << "Couldn't set color exposure: " << openni::OpenNI::getExtendedError();
      response->message = ss.str();
      RCLCPP_ERROR_STREAM(logger_, response->message);
      return false;
    } else {
      return true;
    }
  } else if (stream_index == INFRA1 || stream_index == INFRA2 || stream_index == DEPTH) {
    auto data = static_cast<uint32_t>(request->data);
    auto rc = device_->setProperty(openni::OBEXTENSION_ID_IR_EXP, data);
    if (rc != openni::STATUS_OK) {
      std::stringstream ss;
      ss << "Couldn't set IR exposure: " << openni::OpenNI::getExtendedError();
      response->message = ss.str();
      RCLCPP_ERROR_STREAM(logger_, response->message);
      return false;
    } else {
      return true;
    }
  }
  return false;
}

bool OBCameraNode::getGainCallback(const std::shared_ptr<GetInt32::Request>& request,
                                   std::shared_ptr<GetInt32::Response>& response,
                                   const stream_index_pair& stream_index) {
  (void)request;
  if (stream_index == COLOR) {
    auto stream = streams_.at(stream_index);
    auto camera_settings = stream->getCameraSettings();
    if (camera_settings == nullptr) {
      response->success = false;
      response->message = stream_name_[stream_index] + " Camera settings not available";
      return false;
    }
    response->data = camera_settings->getGain();
  } else if (stream_index == INFRA1 || stream_index == INFRA2 || stream_index == DEPTH) {
    int data = 0;
    int data_size = 4;
    device_->getProperty(openni::OBEXTENSION_ID_IR_GAIN, (uint8_t*)&data, &data_size);
    response->data = data;
  } else {
    response->message = "stream not support get gain";
    return false;
  }
  return true;
}

bool OBCameraNode::setGainCallback(const std::shared_ptr<SetInt32::Request>& request,
                                   std::shared_ptr<SetInt32::Response>& response,
                                   const stream_index_pair& stream_index) {
  (void)response;
  if (stream_index == COLOR) {
    auto stream = streams_.at(stream_index);
    auto camera_settings = stream->getCameraSettings();
    if (camera_settings == nullptr) {
      response->success = false;
      response->message = stream_name_[stream_index] + " Camera settings not available";
      return false;
    }
    camera_settings->setGain(request->data);
  } else if (stream_index == INFRA1 || stream_index == INFRA2 || stream_index == DEPTH) {
    int data = request->data;
    int data_size = 4;
    device_->setProperty(openni::OBEXTENSION_ID_IR_GAIN, (uint8_t*)&data, data_size);
  }
  return true;
}

bool OBCameraNode::getAutoWhiteBalanceEnabledCallback(
    const std::shared_ptr<GetInt32::Request>& request,
    std::shared_ptr<GetInt32::Response>& response, const stream_index_pair& stream_index) {
  (void)request;
  auto stream = streams_.at(stream_index);
  auto camera_settings = stream->getCameraSettings();
  if (camera_settings == nullptr) {
    response->data = 0;
    response->message = stream_name_[stream_index] + " Camera settings not available";
    return false;
  }
  response->data = camera_settings->getAutoWhiteBalanceEnabled();
  return true;
}

bool OBCameraNode::setAutoWhiteBalanceEnabledCallback(
    const std::shared_ptr<SetBool::Request>& request, std::shared_ptr<SetBool::Response>& response,
    const stream_index_pair& stream_index) {
  auto stream = streams_.at(stream_index);
  auto camera_settings = stream->getCameraSettings();
  if (camera_settings == nullptr) {
    response->message = stream_name_[stream_index] + " Camera settings not available";
    RCLCPP_ERROR_STREAM(logger_, response->message);
    return false;
  }
  auto data = request->data;
  auto rc = camera_settings->setAutoWhiteBalanceEnabled(data);
  if (rc != openni::STATUS_OK) {
    std::stringstream ss;
    ss << " Couldn't set auto white balance: " << openni::OpenNI::getExtendedError();
    response->message = ss.str();
    RCLCPP_ERROR_STREAM(logger_, ss.str());
    return false;
  } else {
    return true;
  }
}

bool OBCameraNode::setAutoExposureCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response,
    const stream_index_pair& stream_index) {
  openni::Status status;
  if (stream_index == COLOR) {
    auto stream = streams_.at(stream_index);
    auto camera_settings = stream->getCameraSettings();
    if (camera_settings == nullptr) {
      response->success = false;
      response->message = stream_name_[stream_index] + " Camera settings not available";
      RCLCPP_ERROR_STREAM(logger_, response->message);
      return false;
    }
    status = camera_settings->setAutoExposureEnabled(request->data);
  } else if (stream_index == INFRA1 || stream_index == INFRA2 || stream_index == DEPTH) {
    status = device_->setProperty(XN_MODULE_PROPERTY_AE, request->data);
  } else {
    response->message = "Stream not supported set auto exposure";
    return false;
  }

  if (status != openni::STATUS_OK) {
    std::stringstream ss;
    ss << "Couldn't set auto exposure: " << openni::OpenNI::getExtendedError();
    response->message = ss.str();
    RCLCPP_ERROR_STREAM(logger_, ss.str());
    return false;
  } else {
    return true;
  }
}

bool OBCameraNode::setFanCallback(const std::shared_ptr<SetBool::Request>& request,
                                  std::shared_ptr<SetBool::Response>& response) {
  device_->setProperty(XN_MODULE_PROPERTY_FAN_ENABLE, request->data);
  response->success = true;
  return true;
}

bool OBCameraNode::setLaserEnableCallback(const std::shared_ptr<SetBool::Request>& request,
                                          std::shared_ptr<SetBool::Response>& response) {
  device_->setProperty(openni::OBEXTENSION_ID_LASER_EN, request->data);
  device_->setProperty(XN_MODULE_PROPERTY_EMITTER_STATE, request->data);
  response->success = true;
  return true;
}

bool OBCameraNode::setLdpEnableCallback(const std::shared_ptr<SetBool::Request>& request,
                                        std::shared_ptr<SetBool::Response>& response) {
  (void)response;
  stopStreams();
  auto status = device_->setProperty(XN_MODULE_PROPERTY_LDP_ENABLE, request->data);
  startStreams();
  if (status != openni::STATUS_OK) {
    std::stringstream ss;
    ss << "Couldn't set LDP enable: " << openni::OpenNI::getExtendedError();
    RCLCPP_ERROR_STREAM(logger_, ss.str());
    response->message = ss.str();
    return false;
  } else {
    return true;
  }
}

bool OBCameraNode::setMirrorCallback(const std::shared_ptr<SetBool::Request>& request,
                                     std::shared_ptr<SetBool::Response>& response,
                                     const stream_index_pair& stream_index) {
  (void)response;
  auto stream = streams_.at(stream_index);
  stream->setMirroringEnabled(request->data);
  return true;
}

bool OBCameraNode::getExposureCallback(const std::shared_ptr<GetInt32::Request>& request,
                                       std::shared_ptr<GetInt32 ::Response>& response,
                                       const stream_index_pair& stream_index) {
  (void)request;
  if (stream_index == COLOR) {
    auto stream = streams_.at(stream_index);
    auto camera_settings = stream->getCameraSettings();
    if (camera_settings == nullptr) {
      response->success = false;
      response->message = stream_name_[stream_index] + " Camera settings not available";
      RCLCPP_ERROR_STREAM(logger_, response->message);
      return false;
    }
    response->data = camera_settings->getExposure();
  } else if (stream_index == INFRA1 || stream_index == INFRA2 || stream_index == DEPTH) {
    int data = 0;
    int data_size = 4;
    device_->getProperty(openni::OBEXTENSION_ID_IR_EXP, (uint32_t*)&data, &data_size);
    response->data = data;
    return true;
  } else {
    response->message = "Stream not supported get exposure";
    return false;
  }
  return true;
}

bool OBCameraNode::getDeviceInfoCallback(const std::shared_ptr<GetDeviceInfo::Request>& request,
                                         std::shared_ptr<GetDeviceInfo::Response>& response) {
  (void)request;
  auto device_info = device_->getDeviceInfo();
  response->info.name = device_info.getName();
  response->info.pid = device_info.getUsbProductId();
  response->info.vid = device_info.getUsbVendorId();
  char serial_number[64];
  int data_size = sizeof(serial_number);
  auto rc = device_->getProperty(openni::OBEXTENSION_ID_SERIALNUMBER, serial_number, &data_size);
  if (rc == openni::STATUS_OK) {
    response->info.serial_number = serial_number;
  } else {
    response->info.serial_number = "";
  }
  response->info.serial_number = serial_number;
  return true;
}

bool OBCameraNode::getCameraInfoCallback(const std::shared_ptr<GetCameraInfo::Request>& request,
                                         std::shared_ptr<GetCameraInfo::Response>& response) {
  (void)request;
  auto camera_info = getColorCameraInfo();
  response->info = *camera_info;
  return true;
}

bool OBCameraNode::getSDKVersion(const std::shared_ptr<GetString::Request>& request,
                                 std::shared_ptr<GetString::Response>& response) {
  (void)request;
  nlohmann::json data;
  data["ros_sdk_version"] = OB_ROS_VERSION_STR;
  data["openni_version"] = ONI_VERSION_STRING;
  char buffer[128] = {0};
  int data_size = sizeof(buffer);
  device_->getProperty(XN_MODULE_PROPERTY_SENSOR_PLATFORM_STRING, buffer, &data_size);
  data["firmware_version"] = buffer;
  response->data = data.dump(2);
  return true;
}

bool OBCameraNode::getCameraParamsCallback(const std::shared_ptr<GetCameraParams::Request>& request,
                                           std::shared_ptr<GetCameraParams::Response>& response) {
  (void)request;
  auto camera_params = getCameraParams();
  for (int i = 0; i < 9; i++) {
    response->r2l_r[i] = camera_params.r2l_r[i];
    if (i < 4) {
      response->l_intr_p[i] = camera_params.l_intr_p[i];
      response->r_intr_p[i] = camera_params.r_intr_p[i];
    }
    if (i < 3) {
      response->r2l_t[i] = camera_params.r2l_t[i];
    }
    if (i < 5) {
      response->l_k[i] = camera_params.l_k[i];
      response->r_k[i] = camera_params.r_k[i];
    }
  }
  return true;
}

bool OBCameraNode::toggleSensorCallback(const std::shared_ptr<SetBool::Request>& request,
                                        std::shared_ptr<SetBool::Response>& response,
                                        const stream_index_pair& stream_index) {
  if (request->data) {
    RCLCPP_INFO_STREAM(logger_, stream_name_[stream_index] << " ON");
  } else {
    RCLCPP_INFO_STREAM(logger_, stream_name_[stream_index] << " OFF");
  }
  response->success = toggleSensor(stream_index, request->data, response->message);
  return true;
}

bool OBCameraNode::toggleSensor(const stream_index_pair& stream_index, bool enabled,
                                std::string& msg) {
  if (!device_->hasSensor(stream_index.first)) {
    std::stringstream ss;

    ss << "doesn't  have " << stream_name_[stream_index];
    msg = ss.str();
    RCLCPP_WARN_STREAM(logger_, msg);
    return false;
  }

  if (enabled) {
    enable_[stream_index] = true;
  } else {
    enable_[stream_index] = false;
  }
  stopStreams();
  startStreams();
  return true;
}

bool OBCameraNode::getSupportedVideoModesCallback(
    const std::shared_ptr<GetString ::Request>& request,
    std::shared_ptr<GetString ::Response>& response, const stream_index_pair& stream_index) {
  (void)request;
  if (!supported_video_modes_.count(stream_index)) {
    response->data = "";
    response->message = "No supported video modes";
    return false;
  } else {
    auto modes = supported_video_modes_[stream_index];
    nlohmann::json data;
    for (auto& mode : modes) {
      std::stringstream ss;
      ss << mode.getResolutionX() << "x" << mode.getResolutionY() << "@" << mode.getFps();
      if (data.empty() || data.back() != ss.str()) {
        data.push_back(ss.str());
      }
    }
    response->data = data.dump(2);
    return true;
  }
}

}  // namespace astra_camera
