cmake_minimum_required(VERSION 3.5)
project(astra_camera)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

option(FILTER "OB Filter library" ON)

if( ${FILTER} STREQUAL "ON" )
set(obfilter "On")
else()
set(obfilter "Off")
endif()

include(ExternalProject)

ExternalProject_Add(astra_openni2
  PREFIX astra_openni2
  # SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/astra_openni2/OpenNI2
  GIT_REPOSITORY https://github.com/orbbec/OpenNI2.git
  GIT_TAG orbbec_ros
  CONFIGURE_COMMAND echo "no need to configure"
  #${CMAKE_CURRENT_SOURCE_DIR}/libantlr/configure --prefix=<INSTALL_DIR>
  BUILD_IN_SOURCE 1
  BUILD_COMMAND make release FILTER=${obfilter}
  INSTALL_DIR openni2
  INSTALL_COMMAND tar -xjf <SOURCE_DIR>/Packaging/Final/OpenNI-Linux-2.3.tar.bz2 -C <INSTALL_DIR> --strip 1 && mkdir -p <INSTALL_DIR>/include && ln -fs <INSTALL_DIR>/Include <INSTALL_DIR>/include/openni2
)

link_directories(${CMAKE_CURRENT_BINARY_DIR}/openni2/Redist)
include_directories(include
                    ${Boost_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_BINARY_DIR}/openni2/include
                    )

add_library(astra_wrapper
   src/astra_convert.cpp
   src/astra_device.cpp
   src/astra_device_info.cpp
   src/astra_timer_filter.cpp
   src/astra_frame_listener.cpp
   src/astra_device_manager.cpp
   src/astra_exception.cpp
   src/astra_video_mode.cpp
)
ament_target_dependencies(astra_wrapper
  "rclcpp"
  "builtin_interfaces"
  "sensor_msgs")
target_link_libraries(astra_wrapper -lOpenNI2Orbbec ${Boost_LIBRARIES})
add_dependencies(astra_wrapper astra_openni2)

add_library(astra_driver_lib src/astra_driver.cpp)
ament_target_dependencies(astra_driver_lib
  "rclcpp"
  "builtin_interfaces"
  "sensor_msgs")
target_link_libraries(astra_driver_lib astra_wrapper ${Boost_LIBRARIES})
add_dependencies(astra_driver_lib astra_openni2)

add_executable(astra_camera_node
   ros/astra_camera_node.cpp
)
target_link_libraries(astra_camera_node astra_driver_lib)
add_dependencies(astra_camera_node astra_openni2)

install(TARGETS astra_camera_node
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
)

install(FILES ${CMAKE_BINARY_DIR}/openni2/Redist/libOpenNI2Orbbec.so
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)

install(DIRECTORY ${CMAKE_BINARY_DIR}/openni2/Redist/OpenNI2
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
