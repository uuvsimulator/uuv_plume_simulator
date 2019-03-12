cmake_minimum_required(VERSION 2.8.3)
project(uuv_cpc_sensor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  sensor_msgs
  uuv_plume_msgs
  geometry_msgs
  tf2_ros
  roscpp)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uuv_cpc_ros_sensor
  CATKIN_DEPENDS
    std_msgs
    nav_msgs
    sensor_msgs
    uuv_plume_msgs
    geometry_msgs
    tf2_ros
    roscpp
  DEPENDS Boost 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

# Particle concentration sensor node
add_library(uuv_cpc_ros_sensor src/CPCSensor.cc)
add_dependencies(uuv_cpc_ros_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uuv_cpc_ros_sensor ${catkin_LIBRARIES})

add_executable(uuv_cpc_ros_sensor_node src/CPCSensorNode.cc)
add_dependencies(uuv_cpc_ros_sensor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uuv_cpc_ros_sensor_node ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS uuv_cpc_ros_sensor_node uuv_cpc_ros_sensor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN ".~" EXCLUDE
)
