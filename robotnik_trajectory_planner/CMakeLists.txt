cmake_minimum_required(VERSION 2.8.3)
project(robotnik_trajectory_planner)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  std_msgs
  trajectory_msgs
  actionlib
  control_msgs
  std_msgs
  robotnik_msgs
  genmsg
)


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   State.msg
   JointByJoint.msg
   CartesianEuler.msg
   PointTraj.msg
   Trajectory.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   SelectGroup.srv
   SetControlMode.srv
   GetJointsGroup.srv
   GetGroups.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   robotnik_msgs#   trajectory_msgs
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
  include
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(robotnik_trajectory_planner_node src/rt_traj_planner.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(robotnik_trajectory_planner_node robotnik_msgs_generate_messages_cpp)
add_dependencies(robotnik_trajectory_planner_node robotnik_trajectory_planner_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robotnik_trajectory_planner_node
   ${catkin_LIBRARIES}
)

