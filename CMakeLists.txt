cmake_minimum_required(VERSION 2.8.3)
project(AutoPinball)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  roscpp
  std_msgs
)

## OpenCV
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  flip_flipper.msg
  override_light.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  get_switch.srv
  get_light.srv
)

## Generate added messages and services with any dependencies listed here
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
#  LIBRARIES AutoPinball
#  CATKIN_DEPENDS message_generation rospy
   DEPENDS opencv
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

#############
## Install ##
#############

#############
## Testing ##
#############

#################
## Executables ##
#################
add_executable(track_metal src/Track/track_metal.cpp)
target_link_libraries(track_metal ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(track_metal AutoPinball_generate_messages_cpp)
