# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2022 Intel Corporation. All Rights Reserved.


# fmt: off
configurable_parameters = [
    {"name": "camera_name", "default": "camera", "description": "camera unique name"},
    {"name": "serial_number", "default": "", "description": "choose device by serial number"},
    {"name": "depth_align", "default": "false", "description": "hardware depth align to color"},
    {"name": "log_level", "default": "verbose", "description": "OpenNI log level"},
    {"name": "log_to_console", "default": "false", "description": "openNI log to console"},
    {"name": "log_to_file", "default": "false", "description": "OpenNI log file"},
    {"name": "depth_info_url", "default": "", "description": "depth camera calibration file path"},
    {"name": "color_info_url", "default": "", "description": "color camera calibration file path"},
    {"name": "output", "default": "screen", "description": "pipe node output [screen|log]"},
    {"name": "enable_d2c_viewer", "default": "false", "description": "enable D2C overlay image"},
    {"name": "device_num", "default": "1", "description": "number of camera to run"},
    {"name": "vendor_id", "default": "0x2bc5", "description": "orbbec vid"},
    {"name": "product_id", "default": "0", "description": ""},
    {"name": "enable_point_cloud", "default": "false", "description": "enable point cloud"},
    {"name": "enable_colored_point_cloud", "false": "false", "description": "enable coloread point cloud"},
    {"name": "connection_delay", "default": "100", "description": "delay time ms before open camera"},
    {"name": "enable_color", "default": "true", "description": "enable camera stream"},
    {"name": "color_width", "default": "640", "description": "color resolution  width"},
    {"name": "color_height", "default": "480", "description": "color resolution  height"},
    {"name": "color_fps", "default": "30", "description": "color frame rate"},
    {"name": "color_format", "default": "RGB", "description": ""},
    {"name": "flip_color", "default": "false", "description": ""},
    {"name": "enable_depth", "default": "true", "description": "''"},
    {"name": "depth_width", "default": "640", "description": "''"},
    {"name": "depth_height", "default": "480", "description": "''"},
    {"name": "depth_fps", "default": "30", "description": "''"},
    {"name": "depth_format", "default": "Y16", "description": "''"},
    {"name": "flip_depth", "default": "false", "description": "''"},
    {"name": "enable_infra", "default": "true", "description": "''"},
    {"name": "infra_width", "default": "640", "description": "''"},
    {"name": "infra_height", "default": "480", "description": "''"},
    {"name": "infra_fps", "default": "30", "description": "''"},
    {"name": "infra_format", "default": "Y16", "description": "''"},
    {"name": "flip_infra", "default": "false", "description": "''"},
    {"name": "color.roi.x", "default": "-1", "description": "''"},
    {"name": "color.roi.y", "default": "-1", "description": "''"},
    {"name": "color.roi.width", "default": "-1", "description": "''"},
    {"name": "color.roi.height", "default": "-1", "description": "''"},
    {"name": "depth.roi.x", "default": "-1", "description": "''"},
    {"name": "depth.roi.y", "default": "-1", "description": "''"},
    {"name": "depth.roi.width", "default": "-1", "description": "''"},
    {"name": "depth.roi.height", "default": "-1", "description": "''"},
    {"name": "depth.scale", "default": "1", "description": "''"},
    {"name": "uvc_camera.enable", "default": "false", "description": "use uvc camera"},
    {"name": "uvc_camera.vendor_id", "default": "0x2bc5", "description": "''"},
    {"name": "uvc_camera.product_id", "default": "0x0", "description": "''"},
    {"name": "uvc_camera.retry_count", "default": "100", "description": "retry times when failed to find/open uvc camera"},
    {"name": "image.QoS", "default": "service_default", "description": "''"},
    {"name": "point_cloud.QoS", "default": "service_default", "description": "''"},
    {"name": "camera_info.QoS", "default": "service_default", "description": "''"},
]


# fmt: on


