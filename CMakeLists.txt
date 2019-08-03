cmake_minimum_required(VERSION 2.8.3)
project(3d_ar_pointcloud_extraction)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET(CMAKE_CXX_FLAGS "-std=c++14 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_extraction
  src/pointcloud_extraction.cpp
  src/pointcloud_extraction_node.cpp
)

target_link_libraries(pointcloud_extraction
  ${catkin_LIBRARIES}
)
#############
## Install ##
#############

set(EXEC_FILES
  pointcloud_extraction
)

set(DIRECTORIES
  launch
)
foreach(nodes ${EXEC_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(nodes)

# Install include directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install all files
foreach(dirs ${DIRECTORIES})
  install(DIRECTORY ${dirs}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach(dirs)
#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()