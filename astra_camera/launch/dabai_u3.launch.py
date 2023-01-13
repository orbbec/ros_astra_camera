from launch import LaunchDescription
import launch_ros
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml
import os
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

import pathlib
import sys

sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import launch_utils

local_default_parametes = [
    {"color_width": 640},
    {"color_height": 480},
    {"color_fps": 30},
    {"depth_width": 640},
    {"depth_height": 400},
    {"dpeth_fps": 30},
]


def declare_configurable_parameters(parameters):
    return [
        DeclareLaunchArgument(
            param["name"],
            default_value=param["default"],
            description=param["description"],
        )
        for param in parameters
    ]


def set_configurable_parameters(parameters):
    return dict(
        [(param["name"], LaunchConfiguration(param["name"])) for param in parameters]
    )


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)


def launch_setup(context, *args, **kwargs):
    _config_file = LaunchConfiguration("config_file").perform(context)
    params_from_file = {} if _config_file == "''" else yaml_to_dict(_config_file)
    log_level = "info"

    return [
        launch_ros.actions.Node(
            package="realsense2_camera",
            node_namespace=LaunchConfiguration("camera_name"),
            node_name=LaunchConfiguration("camera_name"),
            node_executable="astra_camera_node",
            prefix=["stdbuf -o L"],
            parameters=[
                set_configurable_parameters(launch_utils.configurable_parameters),
                params_from_file,
            ],
            output="screen",
            arguments=[
                "--ros-args",
                "--log-level",
                LaunchConfiguration("log_level"),
            ],
        )
    ]


def generate_launch_description():
    pass
