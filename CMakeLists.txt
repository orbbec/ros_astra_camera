cmake_minimum_required(VERSION 2.8.3)
project(astra_camera)

find_package(catkin REQUIRED camera_info_manager dynamic_reconfigure image_transport  nodelet sensor_msgs roscpp message_generation)

find_package(Boost REQUIRED COMPONENTS system thread)


#MACHINE = $(shell uname -m)
execute_process(COMMAND uname -m  OUTPUT_VARIABLE  MACHINES )
execute_process(COMMAND getconf LONG_BIT  OUTPUT_VARIABLE  MACHINES_BIT )
MESSAGE(STATUS "ORRBEC Machine : ${MACHINES}")
MESSAGE(STATUS "ORRBEC Machine Bits : ${MACHINES_BIT}")
IF ( (${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64")  )
      set(HOST_PLATFORM  "x64")
# for schroot enviroment
ELSEIF ( (${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "32")  )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "x86" )
ELSEIF ( ${MACHINES} MATCHES "x86" )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "i686" )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "i386" )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "arm" )
      set(HOST_PLATFORM  "arm")
ELSEIF ( (${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64")  )
      set(HOST_PLATFORM  "arm64")
ELSEIF ( (${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "32")  )
      set(HOST_PLATFORM  "arm")
ENDIF ()

MESSAGE(STATUS "ORRBEC : ${HOST_PLATFORM}")

generate_dynamic_reconfigure_options(cfg/Astra.cfg)
add_service_files(FILES
  GetSerial.srv)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES astra_wrapper
  CATKIN_DEPENDS camera_info_manager dynamic_reconfigure image_transport nodelet sensor_msgs roscpp message_runtime
  DEPENDS libastra
)

set(ORRBEC_OPENNI2_REDIST  "${CMAKE_CURRENT_SOURCE_DIR}/include/openni2_redist/${HOST_PLATFORM}")
link_directories(${ORRBEC_OPENNI2_REDIST})

#MESSAGE(STATUS "ORRBEC : ${ORRBEC_OPENNI2_REDIST}")

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/openni2
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
target_link_libraries(astra_wrapper ${catkin_LIBRARIES} -lOpenNI2  -L${ORRBEC_OPENNI2_REDIST}
  ${Boost_LIBRARIES} )

add_library(astra_driver_lib
                src/astra_driver.cpp
)
target_link_libraries(astra_driver_lib astra_wrapper ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(astra_driver_lib ${PROJECT_NAME}_gencfg)

add_library(astra_camera_nodelet
   ros/astra_camera_nodelet.cpp
)
target_link_libraries(astra_camera_nodelet astra_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(astra_camera_nodelet ${PROJECT_NAME}_gencfg)

add_executable(astra_camera_node
   ros/astra_camera_node.cpp
)
target_link_libraries(astra_camera_node astra_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(astra_camera_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(astra_list_devices
   src/list_devices.cpp
)
target_link_libraries(astra_list_devices astra_wrapper)

add_executable(astra_test_wrapper test/test_wrapper.cpp )
target_link_libraries(astra_test_wrapper astra_wrapper ${Boost_LIBRARIES})
if (UNIX AND NOT APPLE)
  add_executable(astra_usb_reset src/usb_reset.c)
  set(ADDITIONAL_EXECUTABLES "astra_usb_reset")
endif()

install(TARGETS astra_wrapper astra_camera_nodelet astra_camera_node astra_list_devices astra_driver_lib ${ADDITIONAL_EXECUTABLES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# add xml file
install(FILES astra_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#copy runtime file to bin, not support wildcard *.*
#execute_process(COMMAND  cp -R ${CMAKE_CURRENT_SOURCE_DIR}/include/openni2_redist/OpenNI2   ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/
#									COMMAND  cp -R ${CMAKE_CURRENT_SOURCE_DIR}/include/openni2_redist/libOpenNI2.jni.so   ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/
#									COMMAND  cp -R ${CMAKE_CURRENT_SOURCE_DIR}/include/openni2_redist/libOpenNI2.so   ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/
#									COMMAND  cp -R ${CMAKE_CURRENT_SOURCE_DIR}/include/openni2_redist/OpenNI.ini  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/
#									COMMAND  cp -R ${CMAKE_CURRENT_SOURCE_DIR}/include/openni2_redist/org.openni.jar  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/
#)



