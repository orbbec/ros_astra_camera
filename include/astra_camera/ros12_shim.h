#ifndef ROS12_SHIM_H_
#define ROS12_SHIM_H_

#include <stdio.h>
#include <iostream>

#ifndef ROS_INFO
  #define ROS_INFO(str, ...) printf(str "\n", ## __VA_ARGS__)
#endif
#ifndef ROS_DEBUG
  #define ROS_DEBUG(str, ...)
#endif
#ifndef ROS_WARN
  #define ROS_WARN(str, ...) printf(str "\n", ## __VA_ARGS__)
#endif
#ifndef ROS_ERROR
  #define ROS_ERROR(str, ...) printf(str "\n", ## __VA_ARGS__)
#endif
#ifndef ROS_FATAL
  #define ROS_FATAL(str, ...) printf(str "\n", ## __VA_ARGS__)
#endif

#ifndef ROS_INFO_STREAM
  #define ROS_INFO_STREAM(str) std::cout << str << std::endl
#endif
#ifndef ROS_DEBUG_STREAM
  #define ROS_DEBUG_STREAM(str)
#endif
#ifndef ROS_WARN_STREAM
  #define ROS_WARN_STREAM(str) std::cout << str << std::endl
#endif
#ifndef ROS_ERROR_STREAM
  #define ROS_ERROR_STREAM(str) std::cout << str << std::endl
#endif
#ifndef ROS_FATAL_STREAM
  #define ROS_FATAL_STREAM(str) std::cout << str << std::endl
#endif

#endif
