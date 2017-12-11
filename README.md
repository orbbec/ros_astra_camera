1. Go to catkin workshop dir and compiler astra_camera
```
$ catkin_make --pkg astra_camera
```

note: astra driver provide two work method, normal and filter. With filter driver, get better quality depth data but need high-performance platform , like pc. If you work in the ARM, suggest to use normal method. You can use -DFILTER=ON / OFF to change the method.

```
$ catkin_make --pkg astra_camera -DFILTER=OFF
```

2. Create astra udev rule
```
$ roscd astra_camera && ./scripts/create_udev_rules
```

3. Run astra_camera
(You will need to clone the launch files from [https://github.com/orbbec/ros_astra_launch](https://github.com/orbbec/ros_astra_launch) into your catkin src directory.)

use astra 
```
$ roslaunch astra_launch astra.launch
```
use astra pro (uvc rgb )
```
$ roslaunch astra_launch astrapro.launch
```

4. You can use rviz or image_view to verify driver


