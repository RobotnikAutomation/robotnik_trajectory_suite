cmake_minimum_required(VERSION 2.8.3)
project(robotnik_trajectory_pad)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  diagnostic_msgs
  robotnik_msgs
  diagnostic_updater
  robotnik_trajectory_planner
  genmsg
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   State.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   robotnik_msgs
)

catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(robotnik_trajectory_pad_node src/robotnik_trajectory_pad_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(robotnik_trajectory_pad_node robotnik_trajectory_pad_generate_messages_cpp robotnik_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robotnik_trajectory_pad_node
   ${catkin_LIBRARIES}
 )

