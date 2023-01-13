import sys
from launch import LaunchDescription
import launch_ros.actions
from ament_index_python import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import copy
from os import path
import yaml
from launch_ros.actions import Node


def generate_container_node(camera_name, params):
    return ComposableNodeContainer(
        name='astra_camera_container',
        namespace=camera_name,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::OBCameraNodeFactory',
                           name='camera',
                           parameters=[params],
                           namespace=camera_name),
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::PointCloudXyzNode',
                           namespace=camera_name,
                           name='point_cloud_xyz'),
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::PointCloudXyzrgbNode',
                           namespace=camera_name,
                           name='point_cloud_xyzrgb')
        ],
        output='screen')


def duplicate_params(general_params, posix, serial_number):
    local_params = copy.deepcopy(general_params)
    local_params["camera_name"] += posix
    local_params["serial_number"] = serial_number
    return local_params


def generate_launch_description():
    params_file = get_package_share_directory("astra_camera") + "/params/astra_mini_params.yaml"
    if not path.exists(params_file):
        print("path %s is not exists" % params_file)
        sys.exit(-1)
    with open(params_file, 'r') as file:
        default_params = yaml.safe_load(file)

    serial_number1 = "ADA611300CE"
    serial_number2 = "sn123456789"
    params1 = duplicate_params(default_params, "1", serial_number1)
    params2 = duplicate_params(default_params, "2", serial_number2)
    container1 = generate_container_node("camera1", params1)
    container2 = generate_container_node("camera2", params2)
    # dummy static transformation from camera1 to camera2
    dummy_tf_node = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "camera1_link",
            "camera2_link",
        ],
    )
    return LaunchDescription(
        [container1, container2, dummy_tf_node])
