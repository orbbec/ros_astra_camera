# astra camera

## 1) goto catkin workshop dir and compiler astra_camera
```
$ catkin_make --pkg astra_camera
```
note: astra driver provide two work method, normal and filter. with filter driver, get better quality depth 
data but need high-performance platform , like pc. if you work in the ARM, suggest to use normal method. you can use -DFILTER=ON / OFF to change the method.
```
$ catkin_make --pkg astra_camera -DFILTER=OFF
```

## 2a) create astra udev rule
```
$ roscd astra_camera && ./scripts/create_udev_rules
```
alternatively you can find it here: https://github.com/orbbec/ros_astra_camera/blob/master/56-orbbec-usb.rules

## 2b) configure astra with turtlebot
```
bash indigo-turtlebot.sh
```

## 3) run astra_camera
use astra 
```
$ roslaunch astra_launch astra.launch
```
use astra pro (uvc rgb )
```
$ roslaunch astra_launch astrapro.launch
```

## 4) you can use rviz or image_view to verify driver

about firmware upgrade
1, only for Astra and Astra S. /**********important*********/
2, run on x86, x64, arm, arm64 platform, and depend on 
zip/unzip, awk, tail, md5sum

