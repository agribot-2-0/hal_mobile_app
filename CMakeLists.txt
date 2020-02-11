#############################################################################
## Set minimum required version of cmake, project name and compile options ##
#############################################################################
cmake_minimum_required(VERSION 2.8.3)
project(hal_mobile_app)

#############################################################################
## Find catkin packages and libraries for catkin and system dependencies   ##
#############################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  agribot_msgs
  agribot_common
)

################################################
## Declare ROS messages, services and actions ##
################################################

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
add_executable(hal_mobile_app src/MobileAppHandler.cpp src/mobile_app_node.cpp)
target_link_libraries(hal_mobile_app ${agribot_common_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(hal_mobile_app ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

#############
## Testing ##
#############
