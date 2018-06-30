cmake_minimum_required(VERSION 2.8.3)
project(plane_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  compressed_depth_image_transport
  cv_bridge
  dynamic_reconfigure
  image_transport
  message_filters
  message_generation
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_msgs
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

add_message_files(
  FILES
  WallInfo.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS compressed_depth_image_transport dynamic_reconfigure  image_transport message_filters message_runtime rosbag   roscpp   rospy   sensor_msgs   std_msgs   tf visualization_msgs
  DEPENDS eigen3 OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}, ${EIGEN3_INCLUDE_DIR} , ${OpenCV2_INCLUDE_DIRS}, ${Boost_INCLUDE_DIRS} , include
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
add_executable(test_detector_bag src/test_plane_detector_bag.cpp)
target_link_libraries(test_detector_bag ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
# 
add_executable(test_detector src/test_plane_detector.cpp)
target_link_libraries(test_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
# 
add_executable(test_detector_marker src/test_plane_marker.cpp)
target_link_libraries(test_detector_marker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_floor_detector src/test_floor_detector.cpp)
target_link_libraries(test_floor_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_floor_detector_bag src/test_floor_detector_bag.cpp)
target_link_libraries(test_floor_detector_bag ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_wall_detector src/test_wall_detector.cpp)
target_link_libraries(test_wall_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})


#add_executable(test_hole_detector src/test_hole_detector.cpp)
#target_link_libraries(test_hole_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})



add_dependencies(test_wall_detector ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} plane_detector_generate_messages_cpp)
add_dependencies(test_detector_bag ${PROJECT_NAME}_gencfg )
add_dependencies(test_detector ${PROJECT_NAME}_gencfg )
add_dependencies(test_detector_marker ${PROJECT_NAME}_gencfg )
add_dependencies(test_floor_detector ${PROJECT_NAME}_gencfg )
add_dependencies(test_floor_detector_bag ${PROJECT_NAME}_gencfg )
add_dependencies(test_wall_detector ${PROJECT_NAME}_gencfg )



## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/plane_detector.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/plane_detector_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_plane_detector.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
