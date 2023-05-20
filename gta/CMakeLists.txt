cmake_minimum_required(VERSION 3.0.2)
project(gta)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  genmsg
  geometry_msgs
  image_transport
  livox_ros_driver
  mavros_msgs
  message_filters
  message_generation
  message_runtime
  nav_msgs
  nodelet
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV 4  REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package (yaml-cpp REQUIRED )

set(ncnn_DIR "/home/luo/gtanodelet/src/GTA/gta/lib/cmake/ncnn")
find_package(ncnn REQUIRED)
if (${ncnn_FOUND})
  message("-- NCNN found.")
  message("-- NCNN_VULKAN flag is ${NCNN_VULKAN}")
  if (${NCNN_VULKAN})
    message("-- AUTO-ENABLING GPU_SUPPORT")
    set(GPU_SUPPORT OFF)
  endif()
  include_directories(${ncnn_INCLUDE})
endif()



## Declare ROS messages
add_message_files(
 FILES
 obj.msg
 vertice.msg
)


generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offb
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs message_generation message_runtime
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${YAML_CPP_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(gta
  src/gta_ydl.cpp
  src/visualization.cpp
  src/To_odometry.cpp
  src/record.cpp

  #vis
)

add_dependencies(gta 
        gta_generate_messages_cpp
        ${${PROJECT_NAME}_EXPORTED_TARGETS} 
        ${catkin_EXPORTED_TARGETS})

target_link_libraries(gta
  ${catkin_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
  ncnn
)