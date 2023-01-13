# OpenNI2 SDK for ROS2

## Environment configuration

### ROS2 environment configuration

● Please refer to the official [ROS 2 installation guide](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

### Install additional dependencies

● Install dependencies

```bash
sudo apt install libgflags-dev nlohmann-json3-dev \
ros-galactic-image-transport ros-galactic-image-publisher
```

● Install glog

```bash
wget -c https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz  -O glog-0.6.0.tar.gz
tar -xzvf glog-0.6.0.tar.gz
cd glog-0.6.0
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig  # Refreshing the link library
```

● Install magic_enum

```bash
wget -c https://github.com/Neargye/magic_enum/archive/refs/tags/v0.8.0.tar.gz -O  magic_enum-0.8.0.tar.gz

tar -xzvf magic_enum-0.8.0.tar.gz
cd magic_enum-0.8.0
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig # Refreshing the link library
```

● Install libuvc

```bash
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig # Refreshing the link library
```

### Getting Started

#### Use cases

● Create a new ros2 workspace

```bash
mkdir -p ~/ros2_ws/src
```

● Extract and copy openNISDk_ROS2_xxx.tar.gz to ~/ros2_ws/src/
● Install libusb rules

```bash
cd ~/ros2_ws/src/ros2_astra_camera/astra_camera/scripts
sudo bash install.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

● Compiling

```bash
 cd ~/ros2_ws
source /opt/ros/galactic/setup.bash 
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

Starting the camera

● In terminal 1

```bash
source /opt/ros/galactic/setup.bash 
source ./install/setup.bash 
ros2 launch astra_camera astra_mini.launch.py
```

● In terminal 2

```bash
source /opt/ros/galactic/setup.bash 
source ./install/setup.bash 
rviz2 -d src/ros2_astra_camera/astra_camera/rviz/pointcloud.rviz
```

● List topics / services/ parameters (open a new terminal)

```bash
source /opt/ros/galactic/setup.bash 
ros2 topic list
ros2 service list
ros2 param list
```

● Check extrinsic parameters for depth to color

```bash
 ros2 topic echo --qos-durability=transient_local /camera/extrinsic/depth_to_color  --qos-profile=services_default
```

● Get camera parameters

```bash
ros2 service call /camera/get_camera_info astra_camera_msgs/srv/GetCameraInfo '{}'
```

● Check camera parameters, please refer to the ROS documentation for the meaning of the specific fields of the camera parameters  [camera info](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)

```bash
ros2 topic echo /camera/depth/camera_info
ros2 topic echo /camera/color/camera_info
```

● Check device information

```bash
ros2 service call /camera/get_device_info astra_camera_msgs/srv/GetDeviceInfo '{}'
```

● Get the SDK version

```bash
ros2 service call /camera/get_sdk_version  astra_camera_msgs/srv/GetString "{}"
```

● Set/get (auto) exposure

```bash
 # Auto exposure switch. For setting the exposure manually, please turn off the auto exposure first
 ros2 service call /camera/set_color_auto_exposure std_srvs/srv/SetBool '{data: false}' 
 ros2 service call /camera/set_uvc_auto_exposure std_srvs/srv/SetBool '{data: false}'
 ros2 service call /camera/set_ir_auto_exposure std_srvs/srv/SetBool "{data: false}"
    
# Setting exposure values
ros2 service call /camera/set_ir_exposure astra_camera_msgs/srv/SetInt32 "{data: 2000}"
ros2 service call /camera/set_color_exposure astra_camera_msgs/srv/SetInt32 "{data: 2000}"
ros2 service call /camera/set_uvc_exposure astra_camera_msgs/srv/SetInt32 "{data: 2000}"
 # Get exposure
 ros2 service call /camera/get_ir_exposure astra_camera_msgs/srv/GetInt32 "{}"
 ros2 service call /camera/get_color_exposure astra_camera_msgs/srv/GetInt32 "{}"
 ros2 service call /camera/get_uvc_exposure astra_camera_msgs/srv/GetInt32 "{}"
```

● Set/get gain

```bash
# Get Gain
ros2 service call /camera/get_color_gain astra_camera_msgs/srv/GetInt32 '{}' # OpenNI camera
ros2 service call /camera/get_ir_gain astra_camera_msgs/srv/GetInt32 '{}' # OpenNI camera
ros2 service call /camera/get_uvc_gain astra_camera_msgs/srv/GetInt32 "{}" # UVC camera
 # Setting the gain
ros2 service call /camera/set_color_gain astra_camera_msgs/srv/SetInt32 "{data: 200}"
ros2 service call /camera/set_ir_gain astra_camera_msgs/srv/SetInt32 "{data: 200}"
ros2 service call /camera/set_uvc_gain astra_camera_msgs/srv/SetInt32 "{data: 200}"
```

● Set mirror mode

```bash
ros2 service call /camera/set_color_mirror std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/set_ir_mirror std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/set_depth_mirror std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/set_uvc_color_mirror std_srvs/srv/SetBool '{data: true}'
```

● Set/get (auto) white balance

```bash
ros2 service call /camera/get_color_auto_white_balance astra_camera_msgs/srv/GetInt32 '{}' # Return 0 or 1
ros2 service call /camera/set_color_auto_white_balance std_srvs/srv/SetBool '{data: false}'
```

● Turn on/off laser

```bash
ros2 service call /camera/set_laser_enable std_srvs/srv/SetBool '{data: true}' # Turn on
ros2 service call /camera/set_laser_enable std_srvs/srv/SetBool '{data: false}' #Turn off
```

● Turn on/off fan

```bash
 ros2 service call /camera/set_fan_mode std_srvs/srv/SetBool '{data: true}' #Turn on
 ros2 service call /camera/set_fan_mode std_srvs/srv/SetBool '{data: false}' # Turn off
```

● Turn LDP on/off

```bash
ros2 service call /camera/set_ldp_enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/set_ldp_enable std_srvs/srv/SetBool '{data: false}'
```

● Turn sensors on/off

```bash
ros2 service call  /camera/toggle_ir std_srvs/srv/SetBool "{data: true}"
ros2 service call  /camera/toggle_color std_srvs/srv/SetBool "{data: true}"
ros2 service call  /camera/toggle_depth std_srvs/srv/SetBool "{data: true}"
ros2 service call /camera/toggle_uvc_camera std_srvs/srv/SetBool "{data : true}"
```

### Multi-Camera

● First, get the serial number of the camera, plug in the camera and run

```bash
ros2 run astra_camera list_devices_node
```

● In the multi-camera launch file, please change to the current connected cameras' serial number

● Launch

```bash
ros2 launch astra_camera multi_astra_mini.launch.py
```

## Configuration file description

```bash
# Parameter setting template
camera_name : "camera" # # Camera name (unique)
color_width: 640 # RGB Wide
color_height: 480 # RGB High
color_fps: 30 # RGB frame rate
enable_color: true #  RGB switches

# IR camera width/height/FPS
ir_width: 640 
ir_height: 480
ir_fps: 30
enable_ir: false
# depth camera width/height/FPS
depth_width: 640
depth_height: 480
depth_fps: 30
enable_depth: true

# open depth to color alignment （hardware registration）
depth_align: false
# Device serial number, required for multiple cameras
serial_number: ""
number_of_devices: 1
# UVC camera , only for dabai , dabai DCW
uvc_camera.enable: false
uvc_camera.format: "mjpeg"
uvc_camera.vid: 0x0
uvc_camera.pid: 0x0
# Retry count when opening camera failed
uvc_camera.retry_count: 100
# Different resolution alignment requires clipping and zooming, so ROI may need to be set
color_roi.x : -1 # Top left x
color_roi.y : -1 # Top left x
color_roi.width: -1
color_roi.height: -1
depth_roi.x : -1
depth_roi.y : -1
depth_roi.width: -1
depth_roi.height: -1
# depth scales 
depth_scale : 1
publish_tf: true
tf_publish_rate: 10.0
reconnect_timeout: 6.0
```

● dabai Depth 640x400 aligned to color 640x480, the color needs to be cropped

```bash
color_roi.x : 0
color_roi.y : 0
color_roi.width: 640
color_roi.height: 400
```

● astra_mini Depth 320x240 aligned to color 640x480 requires depth scale change

```bash
depth_scale : 2
```

**● After changing the configuration file, you need to run `colcon build` to update it under `install`, because the ROS2 environment is found under `install`, not in the source code.**
**● Alignment must be turned on to produce colorized point clouds**

## Self-booting

For the start-up script, edit `ros2_astra_camera_start.sh` and add the following

```bash
ROS_DISTRO=galactic
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI=file:///etc/cyclonedds/config.xml # The cyclonedds configuration file, which can be left out if not available
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # Optional: rmw_cyclonedds_cpp / rmw_fastrtps_cpp

ENVFILE=/home/orbbec/ros2_ws/install/setup.bash #  The environment of ros2_astra_camera

if [ -f ${ENVFILE} ] ; then
  echo "Loading astra camera env..."
  source /opt/ros/${ROS_DISTRO}/setup.bash
  source ${ENVFILE}
  ros2 launch astra_camera dabai_dcw.launch.py # Dabai DCW as an example
else
  echo "ros astra camera ws not found"
fi
```

For the systemd script, edit `ros2_astra_camera.service` and add the following

```bash
[Unit]
Description=orbbec astra device ROS wrapper
After=local-fs.target
ConditionPathExists=/home/orbbec/ros2_ws/install/setup.bash # The environment of ros2_astra_camera

[Service]
Type=simple
User=toosimple
EnvironmentFile=
ExecStart=/home/orbbec/ros2_ws/ros2_astra_camera_start.sh # Where to put the startup script
ExecStop=/bin/kill ${MAINPID}
Restart=on-failure
StartLimitInterval=60
StartLimitBurst=3
KillMode=mixed

[Install]
WantedBy=multi-user.target
```

Copy ros2_astra_camera.service to the system

```bash
sudo cp ros2_astra_camera.service /etc/systemd/system
```

Enable ros2_astra_camera.service

```bash
sudo systemctl enable ros2_astra_camera.service
```

Manual start

```bash
sudo systemctl start ros2_astra_camera.service
```

Check status

```bash
sudo systemctl status ros2_astra_camera.service
```

View log

```bash
journalctl -u ros2_astra_camera.service
```

Disable ros2_astra_camera.service

```bash
sudo systemctl disable ros2_astra_camera.service
```

Delete ros2_astra_camera.service (**please be careful when entering commands**)

```bash
sudo rm /etc/systemd/system/ros2_astra_camera.service
```

## DDS Tuning

The default DDS settings (Galactic) may not be efficient for data transmission. Different DDS settings will have different performance. Here we take CycloneDDS as an example. For detailed information, please refer to[ROS DDS Tuning](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)。

● Edit cyclonedds configuration file

```bash
sudo gedit /etc/cyclonedds/config.xml
```

Add

```bash
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config
https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>lo</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Internal>
            <MinimumSocketReceiveBufferSize>16MB</MinimumSocketReceiveBufferSize>
        </Internal>
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <MaxAutoParticipantIndex>
                30</MaxAutoParticipantIndex>
            <Peers>
                <Peer address="localhost" />
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
```

● Set the environment variables, add to .zshrc or .bashrc

```bash
export ROS_DOMAIN_ID=42 # Numbers from 0 to 232
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI=file:///etc/cyclonedds/config.xml
```

Tip：to understand why the maximum ROS_DOMAIN_ID is 232, please visit [The ROS DOMAIN ID](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html)
● Increase UDP receive buffer size
Edit

```bash
/etc/sysctl.d/10-cyclone-max.conf
```

Add

```bash
net.core.rmem_max=2147483647
net.core.rmem_default=2147483647
```

## Frequently Asked Questions

● No point cloud or image displayed on Rviz2

- The default QoS for outgoing messages is sense_data, i.e. Relaiblity Policy = Best Effort, please check if the Relaiblity Policy is set to Best Effort on Rviz2, and also check if there is any problem with the QoS setting of the message via `ros2 topic info -v /xxx/topic`

● No picture from multiple cameras

- It is possible that the power supply is insufficient, do not connect all cameras to the same hub, and a powered hub is recommended

- It is also possible that the resolution is too high, try turning it lower
- If the color camera is an UVC camera, the serial number of the UVC may not be set
