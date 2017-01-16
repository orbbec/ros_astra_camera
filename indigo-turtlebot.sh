#!/bin/bash

echo "[Update & upgrade the package]"
sudo apt-get update -qq
sudo apt-get upgrade -qq

echo "[Installing chrony and setting the ntpdate]"
sudo apt-get install -y chrony ros-indigo-astra*
sudo ntpdate ntp.ubuntu.com

echo "[Making the catkin workspace and testing the catkin_make]"
mkdir -p ~/turtlebot_ws/src
cd ~/turtlebot_ws/src
catkin_init_workspace
git clone https://github.com/turtlebot/turtlebot.git --branch indigo
cd ~/turtlebot_ws/
catkin_make

echo "[Setting the ROS evironment]"
sh -c "echo \"source ~/turtlebot_ws/devel/setup.bash\" >> ~/.bashrc"
sh -c "echo \"export TURTLEBOT_3D_SENSOR=astra\" >> ~/.bashrc"
sh -c "echo \"export TURTLEBOT_BATTERY=/sys/class/power_supply/BAT1\" >> ~/.bashrc"


echo "[Setting the UDEV rules]"
sudo wget https://raw.githubusercontent.com/orbbec/ros_astra_camera/master/56-orbbec-usb.rules -P /etc/udev/rules.d/
rosrun kobuki_ftdi create_udev_rules

echo "[Complete!!!]"

exec bash

exit 0
