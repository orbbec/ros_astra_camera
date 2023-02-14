# astra_camera

The ros_astra_camera package is an OpenNI2 ROS wrapper created for Orbbec 3D cameras.
This package allows the usage of Orbbec 3D cameras with ROS Kinetic, Melodic, and Noetic distributions

## Install dependencies

### ROS

- Please refer directly to ROS [wiki](http://wiki.ros.org/ROS/Installation).

### other dependencies

- Install dependencies (be careful with your ROS distribution)

  ```bash
  # Assuming you have sourced the ros environment, same below
  sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager\
  ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev
  ```

- Install libuvc.

  ```bash
  git clone https://github.com/libuvc/libuvc.git
  cd libuvc
  mkdir build && cd build
  cmake .. && make -j4
  sudo make install
  sudo ldconfig
  ```

## Getting start

- Create a ros workspace( if you don't have one).

```bash
mkdir -p ~/ros_ws/src
```

- Clone code from github.

```bash
 cd ~/ros_ws/src
 git clone https://github.com/orbbec/ros_astra_camera.git
```

- Build

```bash
cd ~/ros_ws
catkin_make
```

- Install udev rules.

```bash
cd ~/ros_ws
source ./devel/setup.bash
roscd astra_camera
./scripts/create_udev_rules
sudo udevadm control --reload && sudo  udevadm trigger
```

Start the camera

- In terminal 1

```bash
source ./devel/setup.bash 
roslaunch astra_camera astra.launch
```

- In terminal 2

```bash
source ./devel/setup.bash
rviz
```

Select the topic you want to display

- Check topics / services/ parameters (open a new terminal)

```bash
rostopic list
rosservice list
rosparam list
```

- Check camera extrinsic parameter(from depth to color)

```bash
rostopic echo /camera/extrinsic/depth_to_color

```

- Get camera parameter

```bash
rosservice call /camera/get_camera_params "{}"
```

- Check camera parameter, please refer to the ROS documentation for the meaning of the specific fields of the camera
  parameter [camera info](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)

```bash
rostopic echo /camera/depth/camera_info
rostopic echo /camera/color/camera_info
```

- Check device information

```bash
rosservice call /camera/get_device_info "{}" 
```

- Get the SDK version (Include firmware and OpenNI version )

```bash
rosservice call /camera/get_version "{}"
```

- Set/get (auto) exposure

```bash
rosservice call /camera/set_color_auto_exposure '{data: false}' 
rosservice call /camera/set_uvc_auto_exposure  '{data: false}'
rosservice call /camera/set_ir_auto_exposure  "{data: false}"
    
# Setting exposure values (Be careful with the data range, the following example may not be correct.)
rosservice call /camera/set_ir_exposure  "{data: 2000}"
roservice call /camera/set_color_exposure  "{data: 2000}"
rosservice call /camera/set_uvc_exposure  "{data: 2000}"
 # Get exposure
 rosservice call /camera/get_ir_exposure  "{}"
 rosservice call /camera/get_color_exposure "{}"
 rosservice call /camera/get_uvc_exposure "{}"
```

- Set/get gain

```bash
# Get Gain
rosservice call /camera/get_color_gain '{}' # OpenNI camera
rosservice call /camera/get_ir_gain '{}' # OpenNI camera
rosservice call /camera/get_uvc_gain "{}" # UVC camera
# Setting the gain (Be careful with the data range, the following example may not be correct.)
rosservice call /camera/set_color_gain  "{data: 200}"
rosservice call /camera/set_ir_gain "{data: 200}"
roservice call /camera/set_uvc_gain "{data: 200}"
```

- Set/get mirror

```bash
rosservice call /camera/set_color_mirror  "{data: true}"
rosservice call /camera/set_depth_mirror  "{data: true}"
rosservice call /camera/set_ir_mirror  "{data: true}"
rosservice call /camera/set_uvc_mirror  "{data: true}"
```

- Set/get (auto) white balance

```bash
rosservice call /camera/set_uvc_auto_white_balance  "{data: false}"
#(Be careful with the data range, the following example may not be correct.)
rosservice call /camera/set_uvc_white_balance  "{data: 200}"
```

- Turn on/off laser

```bash
rosservice call /camera/set_laser '{data: true}' # Turn on
rosservice call /camera/set_laser '{data: false}' # Turn off
```

- Turn on/off fans

```bash
 rosservice call /camera/set_fan  '{data: true}' # Turn on
 rosservice call /camera/set_fan  '{data: false}' # Turn off
```

- Turn on/off LDP

```bash
rosservice call /camera/set_ldp '{data: true}'
rosservice call /camera/set_ldp '{data: false}'
```

- Turn on/off sensors

```bash
rosservice call  /camera/toggle_ir "{data: true}"
rosservice call  /camera/toggle_color "{data: true}"
rosservice call  /camera/toggle_depth "{data: true}"
rosservice call  /camera/toggle_uvc_camera "{data : true}"
```

- Save image

```bash
rosservice call /camera/save_images "{}"
```

For dabai and dabai dcw (RGB camera is UVC protocol), run:

```bash
rosservice call /camera/save_uvc_image "{}" 
```

NOTE: The images are saved under ~/.ros/image and are only available when the sensor is on.

- Save point cloud

```bash
rosservice call /camera/save_point_cloud_xyz "{}" # xyz
rosservice call /camera/save_point_cloud_xyz_rgb "{}" # xyzrgb
```

NOTE: Point cloud are only available if it is running and saved under ~/.ros/point_cloud.

### **Multiple cameras**

- First, you need to enumerate the serial number of the camera, plug in the cameras,
and run the following command:

  ```bash
  roslaunch astra_camera list_devices.launch  
  ```

- **Set the parameter `device_num` to the number of cameras**

- Next, go to ros_astra_camera/launch/multi_xxx.launch and change the serial number. Currently, the different cameras can only be distinguished by the serial number.

```
  <launch>
    <!-- unique camera name-->
    <arg name="camera_name" default="camera"/>
    <!-- Hardware depth registration -->
    <arg name="3d_sensor" default="astra"/>
    <!-- stereo_s_u3, astrapro, astra -->
    <arg name="camera1_prefix" default="01"/>
    <arg name="camera2_prefix" default="02"/>
    <arg name="camera1_serila_number" default="AALXB1301YW"/>
    <arg name="camera2_serila_number" default="AD7J7230031"/>
    <arg name="device_num" default="2"/>
    <include file="$(find astra_camera)/launch/$(arg 3d_sensor).launch">
        <arg name="camera_name" value="$(arg camera_name)_$(arg camera1_prefix)"/>
        <arg name="serial_number" value="$(arg camera1_serila_number)"/>
        <arg name="device_num" value="$(arg device_num)"/>
    </include>

    <include file="$(find astra_camera)/launch/$(arg 3d_sensor).launch">
        <arg name="camera_name" value="$(arg camera_name)_$(arg camera2_prefix)"/>
        <arg name="serial_number" value="$(arg camera2_serila_number)"/>
        <arg name="device_num" value="$(arg device_num)"/>
    </include>
    <node pkg="tf2_ros" type="static_transform_publisher" name="camera_tf" args="0 0 0 0 0 0 camera01_link camera02_link"/>
</launch>

```

- The Astra camera will use a semaphore for process synchronization. If the camera fails to start, the semaphore file may be left in the /dev/shm, causing the next start to get stuck. To prevent this issue, please run the following command before launching:

```bash
rosrun astra_camera cleanup_shm_node 
```

This command will clean up all semaphore files in the /dev/shm directory, ensuring that the camera will not get stuck during the next start.

- Launch

``` bash
roslaunch astra_camera multi_astra.launch
```

## Use calibration camera parameter

- Set camera info uri, Go to `xxx.launch`

```xml

<launch>
    <!--...-->
    <arg name="ir_info_uri" default="file:///you_ir_camera_calib_path/depth_camera.yaml"/>
    <arg name="color_info_uri" default="file:///you_depth_camera_calib_path/rgb_camera.yaml"/>
    <!--...-->
</launch>
```

- calibration file should like

```yaml
image_width: 640
image_height: 480
# The camera name is fixed. The color camera is rgb_camera, the depth/IR camera name is ir_camera
camera_name: rgb_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [517.301, 0, 326.785, 0, 519.291, 244.563, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.41527, 0.31874, -0.00197, 0.00071, 0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [0.999973, 0.00612598, -0.00406652, -0.00610201, 0.999964, 0.00588094, 0.0041024, -0.00585596, 0.999974 ]
projection_matrix:
  rows: 3
  cols: 4
  data: [517.301, 0, 326.785, -25.3167, 0, 519.291, 244.563, 0.282065, 0, 0, 1, 0.0777703]

```

## Launch parameters

- `connection_delay`:The delay time for reopening the device in milliseconds. Some devices may take longer to initialize, such as the Astra mini. Reopening the device immediately may cause firmware crashes when hot plugging.
- `enable_point_cloud`: Whether to enable point cloud..
- `enable_point_cloud_xyzrgb`,:Whether to enable RGB point cloud.
- `enable_d2c_viewer`: Publish D2C overlay image(For testing only).
- `device_num`: The number of devices. You need to specify the number of devices when using multiple cameras.
- `enable_reconfigure`, Whether to enable ROS [dynamic configuration](http://wiki.ros.org/dynamic_reconfigure) changes,
  set to false means that the `Astra.cfg` configuration will not take effect. This is recommended for testing purposes only. Turn it off when in use.
  use. .
- `color_width`， `color_height`， `color_fps`:  Color stream resolution and frame rate.
- `ir_width`， `ir_height`， `ir_fps`:IR stream resolution and frame rate.
- `depth_width`， `depth_height`， `depth_fps`: Depth stream resolution and frame rate.
- `enable_color`: Whether to enable RGB camera. This parameter has no effect when the RGB camera is using the UVC protocol.
- `enable_depth` : Whether to enable the depth camera.
- `enable_ir`: Whether to enable the IR camera.
- `depth_align`: Enables hardware depth to color alignment, which is required when the RGB point cloud is enabled.
- `depth_scale`: Depth image zoom scale. For example, setting it to 2 means aligning a depth image of size 320x240 to an RGB image of size 640x480.
- `color_roi_x`， `color_roi_y`， `color_roi_width`， `color_roi_height`:: Whether to crop RGB images. The default is -1, which is only used when the RGB resolution is greater than the depth resolution and needs to be aligned. For example, if you need to align a depth image of size 640x400 to an RGB image of size 640x480, you need to set `color_roi_x` to 0, `color_roi_y` to 0, `color_roi_width` to 640, and `color_roi_height` to 400. This will crop the top 400 pixels of the RGB image with a corresponding depth ROI.
- `color_depth_synchronization`，Enable synchronization of RGB with depth
- `use_uvc_camera`: If the RGB camera is using the UVC protocol, set this parameter to true. UVC is the protocol that currently includes Dabai, Dabai_dcw, and so on.
- `uvc_product_id`:PID of the UVC camera.
- `uvc_camera_format`:Image format for the UVC camera.
- `uvc_retry_count` : Sometimes the UVC protocol camera does not reconnect successfully when hot plugging, requiring many retries.
- `enable_publish_extrinsic` Enable publishing camera extrinsic.
- `oni_log_level`:  Log levels for OpenNI: `verbose`, `info`, `warning`, `error`, or `none`.
- `oni_log_to_console`, Whether to output OpenNI logs to the console.
- `oni_log_to_file`:Whether to output OpenNI logs to a file. By default, it will be saved in the Log folder under the path of the currently running program.
- For special customer requirements:
  - `keep_alive`,  Whether to send heartbeat packets to the firmware. This is not enabled by default.
  - `keep_alive_interval`, The time interval in seconds between sending heartbeat packets.

## Frequently Asked Questions

- No image when multiple cameras
  - Maybe the the power supply is not sufficient, consider to connect the camera with a powered USB hub.
  - Maybe the the resolution is too high, lower the resolution to test
- Hot-plug image anomaly
  - The `connection_delay` parameter can be increased because some devices take longer to initialize and may not have completed the initialization of the device.
- No image when hot-plugging
  - Check if the data cable is plugged in properly.
  - Try connecting the camera to a powered USB hub. The ARM development board may have an unstable power supply, causing the device to fail to reopen.
- Launching the camera gets stuck:

  - t is highly likely that the camera failed to start the last time. The Astra camera uses a semaphore to do process synchronization. If it fails to start, the semaphore file may be left in the `/dev/shm`, resulting in the next start being stuck. Just run `rosrun astra_camera cleanup_shm_node`, and the problem should be resolved..

- The point cloud's frame rate is very low. Consider increasing it by adding a core recv buffer:

   ```bash
    sudo sysctl -w net.core.rmem_max=8388608 net.core.rmem_default=8388608
   ```

## License

Copyright 2023 Orbbec Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with
the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "
AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.

*Other names and brands may be claimed as the property of others*
