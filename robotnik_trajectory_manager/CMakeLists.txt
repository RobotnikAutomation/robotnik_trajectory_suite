cmake_minimum_required(VERSION 2.8.3)
project(robotnik_trajectory_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  robotnik_msgs
  genmsg
)

#
catkin_python_setup()


## Generate services in the 'srv' folder
add_service_files(
   FILES
   SaveState.srv
   LoadState.srv
   GetStates.srv
   GetJointsState.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotnik_trajectory_manager
#  CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

