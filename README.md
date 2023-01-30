# astra_camera

A ROS driver for Orbbec 3D cameras. This package supports ROS Kinetic, Melodic and Noetic distributions

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

- First, you need to enumerate the serial number of the camera, plug in the cameras and run

  ```bash
  roslaunch astra_camera list_devices.launch  
  ```

- **Set the parameter `device_num` to the number of cameras**

- Go to the `ros_astra_camera/launch/multi_xxx.launch`   and change the serial number. Currently, different cameras can
  only be distinguished by the serial number,

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

- astra camera will use semaphore to do process synchronization,
  if the camera start fails, the semaphore file may be left in the `/dev/shm`, 
  causing the next start to be stuck.
  Before launch, please run

```bash
rosrun astra_camera cleanup_shm_node 
```
to clean up `/dev/shm`.
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

**The configuration file is under the `ros_astra_cameraparams/camera_params_template.yaml`, the param parameters in the
launch file can override the yaml values**

- `reconnection_delay`, The delay time for reopening the device in seconds. Some devices would take longer time to
  initialize, such as Astra mini, so reopening the device immediately would causes firmware crashes when hot plug.
- `enable_point_cloud`, Whether to enable point cloud.
- `enable_point_cloud_xyzrgb`, Whether to enable RGB point cloud.
- `enable_d2c_filter`, Publish D2C overlay image.
- `device_num`,The number of devices, You need to fill in the number of devices when you need multiple cameras.
- `enable_reconfigure`, Whether to enable ROS [dynamic configuration](http://wiki.ros.org/dynamic_reconfigure) changes,
  set to false `Astra.cfg` configuration will not take effect, recommended for testing purposes only, turn off when in
  use. .
- `color_width`， `color_height`， `color_fps`， color stream resolution and frame rate.
- `ir_width`， `ir_height`， `ir_fps`，IR stream resolution and frame rate
- `depth_width`， `depth_height`， `depth_fps` depth stream resolution and frame rate
- `enable_color`， Whether to enable RGB camera, this parameter has no effect when the RGB camera is UVC protocol
- `enable_depth` , Whether to enable depth camera
- `enable_ir`, Whether to enable IR camera
- `depth_align`, Enables hardware depth to color alignment, requires RGB point cloud to open
- `depth_scale`, Depth image zoom scale, e.g. set to 2 means aligning depth 320x240 to RGB 640x480
- `color_roi_x`， `color_roi_y`， `color_roi_width`， `color_roi_height`, Whether to crop RGB images, the default is -1,
  which is only used when the RGB resolution is greater than the depth resolution and needs to be aligned. For example,
  if you need to align the depth 640x400 to RGB 640x480, you need to set color_roi_x: 0, color_roi_y: 0,
  color_roi_width: 640, color_roi_height: 400. roi_height: 400, which will crop the top 400 pixels of the RGB with a
  corresponding depth ROI.
- `color_depth_synchronization`，Enable synchronization of RGB with depth
- `use_uvc_camera`，if the RGB camera is UVC protocol, setting as true, UVC is the protocol that currently includes
  dabai, dabai_dcw etc.
- `uvc_product_id`，pid of UVC camera
- `uvc_camera_format`，Image format for uvc camera
- `uvc_retry_count` sometimes the UVC protocol camera does not reconnect successfully when hot-plug, requiring many
  times to retry.
- `enable_publish_extrinsic` Enable publish camera extrinsic.
- `oni_log_level`, Log levels for OpenNI verbose/ info /warning/ error /none
- `oni_log_to_console`, Whether to output OpenNI logs to the console
- `oni_log_to_file`, Whether to output OpenNI logs to a file, by default it will save in Log folder under the path of
  the currently running program
- For Special customer Required
    - `keep_alive`, Whether to send heartbeat packets to the firmware, not enabled by default
    - `keep_alive_interval`, The time interval in seconds between sending heartbeat packets

## Frequently Asked Questions

- No image when multiple cameras
    - Maybe the the power supply is not sufficient, consider to connect the camera with a powered USB hub.
    - Maybe the the resolution is too high, lower the resolution to test
- Hot-plug image anomaly
    - The `reconnection_delay` parameter can be set to bigger, as some devices take longer time to initialize and may
      not have completed the initialization of the device.
- No image when hot-plugging
    - Check if the data cable is plugged in well
    - Try to connect to a powered usb hub, the ARM development board may have unstable power supply causing the device
      fail to repoen.
- launch camera get stuck.

    - It is very likely that the camera failed to start last time, astra camera will use the semaphore
      to do process synchronization, if it fails to start, the semaphore file may be left in the shm,
      resulting in the next start stuck, Just run `rosrun astra_camera cleanup_shm_node` problem should be resolved.

- The frame rate of a point cloud is very low, Consideration increased by adding an udp buffer

   ```bash
    sudo sysctl -w net.core.rmem_max=8388608 net.core.rmem_default=8388608
   ```

## License

Copyright 2022 Orbbec Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with
the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "
AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.

*Other names and brands may be claimed as the property of others*
