# 相机控制ROS服务

* `ros2 service list` 查看所有服务。

* **如果在配置文件没打开相应的传感器，不会有相应的service**，（需求里面的topic 可配置）。

* 相关命令

  * 获取相机外参数

    ```bash
    ros2 topic echo --qos-durability=transient_local /camera/extrinsic/depth_to_color  --qos-profile=services_default
    ```

    

  * 获取相机参数

    ```bash
    ros2 service call /camera/get_camera_info astra_camera_msgs/srv/GetCameraInfo '{}'
    ```

    

  * 获取SDK版本

    ```bash
    ros2 service call /camera/get_sdk_version astra_camera_msgs/srv/GetString '{}'
    
    ```
  
    
  
  * 获取设备信息
  
    ```bash
    ros2 service call /camera/get_device_info astra_camera_msgs/srv/GetDeviceInfo '{}'
    ```
  
    
  
  * 设置/获取（自动）曝光
  
    ```bash
    # 自动曝光开关，如果手动设置曝光，需要先关闭自动曝光
    ros2 service call /camera/set_color_auto_exposure std_srvs/srv/SetBool '{data: false}' 
    ros2 service call /camera/set_uvc_auto_exposure std_srvs/srv/SetBool '{data: false}'
    ros2 service call /camera/set_ir_auto_exposure std_srvs/srv/SetBool "{data: false}"
    
    # 设置曝光值
    ros2 service call /camera/set_ir_exposure astra_camera_msgs/srv/SetInt32 "{data: 2000}"
    ros2 service call /camera/set_color_exposure astra_camera_msgs/srv/SetInt32 "{data: 2000}"
    ros2 service call /camera/set_uvc_color_exposure astra_camera_msgs/srv/SetInt32 "{data: 2000}"
    # 注意深度没有曝光
    ```

  * 设置/获取增益

    ```bash
    # 获取增益
    ros2 service call /camera/get_color_gain astra_camera_msgs/srv/GetInt32 '{}' # OpenNI camera
    ros2 service call /camera/get_ir_gain astra_camera_msgs/srv/GetInt32 '{}' # OpenNI camera
    ros2 service call /camera/get_uvc_gain astra_camera_msgs/srv/GetInt32 "{}" # UVC camera
    # 设置增益
    ros2 service call /camera/set_color_gain astra_camera_msgs/srv/SetInt32 "{data: 200}"
    ros2 service call /camera/set_ir_gain astra_camera_msgs/srv/SetInt32 "{data: 200}"
    ros2 service call /camera/set_uvc_color_gain astra_camera_msgs/srv/SetInt32 "{data: 200}"
    # 注意深度没有增益
    ```

    

  * 设置/获取镜像

    ```bash
    ros2 service call /camera/set_color_mirror std_srvs/srv/SetBool '{data: true}'
    ros2 service call /camera/set_ir_mirror std_srvs/srv/SetBool '{data: true}'
    ros2 service call /camera/set_depth_mirror std_srvs/srv/SetBool '{data: true}'
    ros2 service call /camera/set_uvc_color_mirror std_srvs/srv/SetBool '{data: true}'
    ```
  
    
  
  * 设置/获取(自动)白平衡
  
    ```bash
    ros2 service call /camera/get_color_auto_white_balance astra_camera_msgs/srv/GetInt32 '{}' # 返回 0 或 1
    ros2 service call /camera/set_color_auto_white_balance std_srvs/srv/SetBool '{data: false}'
    ```

    
  
  * 打开/关闭激光

    ```bash
    ros2 service call /camera/set_laser_enable std_srvs/srv/SetBool '{data: true}' # 打开
    ros2 service call /camera/set_laser_enable std_srvs/srv/SetBool '{data: false}' #关闭
    ```
  
    
  
  * 打开/关闭风扇
  
    ```bash
    ros2 service call /camera/set_fan_mode std_srvs/srv/SetBool '{data: true}' #打开
    ros2 service call /camera/set_fan_mode std_srvs/srv/SetBool '{data: false}' # 关闭
    
    ```
  
    
  
  * 打开/关闭LDP
  
    ```bash
    ros2 service call /camera/set_ldp_enable std_srvs/srv/SetBool '{data: true}'
    ros2 service call /camera/set_ldp_enable std_srvs/srv/SetBool '{data: false}'
    ```
  
    
  
  * 打开/关闭传感器
  
    ```bash
    ros2 service call  /camera/toggle_ir std_srvs/srv/SetBool "{data : true}"
    ros2 service call  /camera/toggle_color std_srvs/srv/SetBool "{data : true}"
    ros2 service call  /camera/toggle_depth std_srvs/srv/SetBool "{data : true}"
    ros2 service call /camera/toggle_uvc_camera std_srvs/srv/SetBool "{data : true}"
    ```
