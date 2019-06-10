# astra_camera

A ROS driver for Orbbec 3D cameras.

## Install

1. Pull the repository into your ROS workspace `git clone https://github.com/orbbec/ros_astra_camera`

2. Install dependences
    ```sh
    sudo apt install ros-$ROS_DISTRO-rgbd-launch ros-$ROS_DISTRO-libuvc ros-$ROS_DISTRO-libuvc-camera ros-$ROS_DISTRO-libuvc-ros
    ```

3. Go to catkin workspace and compile astra_camera
    `catkin_make --pkg astra_camera`

4. Create astra udev rule
    `sudo ./scripts/create_udev_rules`

### Filter Enable

Astra driver provides normal and filtering methods. With filter driver, we can get more precise depth data but it would cost more computing resources. If the program will be executed on embedded systems, we suggest to use normal method. You can use `-DFILTER=ON / OFF` to change the method as below.

`catkin_make --pkg astra_camera -DFILTER=OFF`

## Important Topics

* `*/image_raw`: depth/rgb/ir raw images
* `*/image_rect_raw`: images rectified by intrinsic/extrinsic parameters
* `*/camera_info`: camera intrinsic/extrinsic paramers
* `/camera/depth/points`: point cloud without color information
* `/camera/depth_registered/points`: Xyzrgb point cloud

## Run astra_camera

You will need to clone the launch files from [astra_launch repository](https://github.com/orbbec/ros_astra_launch) into your workspace.

### Examples

#### Use Astra

`roslaunch astra_launch astra.launch`

#### Use Astra Stereo S (w/ UVC)

`roslaunch astra_launch stereo_s.launch`

You can use **rviz** or **image_view** to verify the outputs.

## Useful Services

This package provides multiple [ros services](http://wiki.ros.org/Services) for users to get useful information and set up devices. To know more about using these services, please check [this tutorial](http://wiki.ros.org/rosservice).

* `/camera/get_device_type`: return a string containing astra device type
* `/camera/get_ir_exposure`: get exposure value of ir camera
* `/camera/get_ir_gain`: get gain value of ir camera
* `/camera/get_serial`: get serial number
* `/camera/get_uvc_exposure`: get exposure value of rgb camera
* `/camera/get_uvc_gain`: get gain value of rgb camera
* `/camera/get_uvc_white_balance`: get white balance value of rgb camera
* `/camera/reset_ir_exposure`: reset ir exposure to default value
* `/camera/reset_ir_gain`: reset ir gain to default value
* `/camera/set_ir_exposure`: set ir exposure to specific value
* `/camera/set_ir_gain`: set ir gain to specific value
* `/camera/set_laser`: turn on (**true**) or turn off (**false**) laser
* `/camera/set_uvc_exposure`: set uvc exposure. (set **0** indicating auto mode)
* `/camera/set_uvc_gain`: set uvc gain
* `/camera/set_uvc_white_balance`: set uvc white balance (set **0** indicating auto mode)
* `/camera/set_ir_flood`: turn on (**true**) or turn off (**false**) ir flood

### Examples

After launching an astra camera, you can get ir exposure by the following command
1. ir exposure
    ```sh
    rosservice call /camera/get_ir_exposure "{}"
    ```
    Next, you can change this value in this way
    ```sh
    rosservice call /camera/set_ir_exposure "{exposure: 50}"
    ```

2. turn on/off laser
    ```sh
    ###### turn on
    rosservice call /camera/set_laser "{enable: true}"
    ###### turn off
    rosservice call /camera/set_laser "{enable: false}"
    ```


For the other services, the usage is same as the above example.

## License

Copyright 2019 Orbbec Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

*Other names and brands may be claimed as the property of others*