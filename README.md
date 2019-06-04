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

`./scripts/create_udev_rules`

### Filter Enable

Astra driver provides normal and filtering methods. With filter driver, we can get more precise depth data but it would cost more computing resources. If the program will be executed on embedded systems, we suggest to use normal method. You can use `-DFILTER=ON / OFF` to change the method as below.

`catkin_make --pkg astra_camera -DFILTER=OFF`

## Run astra_camera

You will need to clone the launch files from [astra_launch repository](https://github.com/orbbec/ros_astra_launch) into your workspace.

### Examples

#### Use Astra

`roslaunch astra_launch astra.launch`

#### Use Astra Stereo S (w/ UVC)

`roslaunch astra_launch stereo_s.launch`

You can use **rviz** or **image_view** to verify the outputs.

# Windows 10 Subsystem

If you encountered `udev` error on Windows 10 Subsystem, try to remove `upstart` and `udev`.

```sh
apt-get remove upstart
apt-get remove udev
apt-get autoremove
```
