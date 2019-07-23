cmake_minimum_required(VERSION 2.8.3)
project(plane_detector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

 
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  compressed_depth_image_transport
  cv_bridge
  dynamic_reconfigure
  image_transport
  message_filters
  rosbag
  roscpp
  rospy
  sensor_msgs
  tf
  visualization_msgs
)

#find_package(Eigen3 REQUIRED)
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)


SET(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Release)

#Dynamic reconfigure stuff
generate_dynamic_reconfigure_options(cfg/Detector.cfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS compressed_depth_image_transport dynamic_reconfigure  image_transport message_filters message_runtime rosbag   roscpp   rospy   sensor_msgs   std_msgs   tf visualization_msgs
#  LIBRARIES ${PROJECT_NAME}
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}, ${EIGEN3_INCLUDE_DIR} , ${OpenCV2_INCLUDE_DIRS}, ${Boost_INCLUDE_DIRS}, include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(test_plane src/test_plane.cpp)
target_link_libraries(test_plane ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_detector_bag src/test_plane_detector_bag.cpp)
target_link_libraries(test_detector_bag ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
# 
add_executable(test_detector src/test_plane_detector.cpp)
target_link_libraries(test_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
# 
add_executable(test_detector_marker src/test_plane_marker.cpp)
target_link_libraries(test_detector_marker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

# 
add_executable(test_floor_detector src/test_floor_detector.cpp)
target_link_libraries(test_floor_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(test_detector_bag ${PROJECT_NAME}_gencfg )
add_dependencies(test_detector ${PROJECT_NAME}_gencfg )
add_dependencies(test_detector_marker ${PROJECT_NAME}_gencfg )
add_dependencies(test_floor_detector ${PROJECT_NAME}_gencfg )

## Install stuff
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

