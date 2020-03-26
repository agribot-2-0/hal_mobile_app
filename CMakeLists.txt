#############################################################################
## Set minimum required version of cmake, project name and compile options ##
#############################################################################
cmake_minimum_required(VERSION 2.8.3)
project(hal_mobile_app)

#############################################################################
## Find catkin packages and libraries for catkin and system dependencies   ##
#############################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
  agribot_common
  agribot_msgs
  roscpp
  rospy
  tf2
  tf2_ros
  std_msgs
  sensor_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hal_agribot_control
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

#############################################################################
## Build                                                                   ##
#############################################################################

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${agribot_common_SOURCE_DIR}/include
)

link_directories("${PROJECT_SOURCE_DIR}")
add_executable(mobile_app_node src/mobile_app_node.cpp src/MobileAppHandler.cpp)
target_link_libraries(mobile_app_node ${catkin_LIBRARIES})
add_dependencies(mobile_app_node ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

#############
## Testing ##
#############
