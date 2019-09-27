cmake_minimum_required(VERSION 2.8.3)
project(farquaad)

# name of current executable
set(yeet test012c)
#set(yeet droneCtrlBackend)
#set(yeet droneCtrlFrontend)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ardrone_autonomy
  cv_bridge
  image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES farquaad
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

set(OpenCV_LIBRARIES
    /home/eric/ros_catkin_ws/install_isolated/lib/libopencv_core3.so
    /home/eric/ros_catkin_ws/install_isolated/lib/libopencv_videoio3.so
    /home/eric/ros_catkin_ws/install_isolated/lib/libopencv_imgproc3.so
    /home/eric/ros_catkin_ws/install_isolated/lib/libopencv_imgcodecs3.so
    /home/eric/ros_catkin_ws/install_isolated/lib/libopencv_highgui3.so
)




include_directories(
  /home/eric/quad_ws/src/ardrone_autonomy/include
  /home/eric/ros_catkin_ws/install_isolated/include/opencv-3.3.1-dev
  ${catkin_INCLUDE_DIRS}
)

add_executable(${yeet} src/${yeet}.cpp)
target_link_libraries(${yeet} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${yeet} ardrone_autonomy_generate_messages_cpp)

add_executable(keyinfo src/keyinfo.cpp)
target_link_libraries(keyinfo ${catkin_LIBRARIES})
add_dependencies(keyinfo ardrone_autonomy_generate_messages_cpp)

add_executable(updown src/updown.cpp)
target_link_libraries(updown ${catkin_LIBRARIES})
add_dependencies(updown ardrone_autonomy_generate_messages_cpp)

add_executable(test003 src/test003.cpp)
target_link_libraries(test003 ${catkin_LIBRARIES})
add_dependencies(test003 ardrone_autonomy_generate_messages_cpp)

add_executable(emergency src/emergency.cpp)
target_link_libraries(emergency ${catkin_LIBRARIES})
add_dependencies(emergency ardrone_autonomy_generate_messages_cpp)

