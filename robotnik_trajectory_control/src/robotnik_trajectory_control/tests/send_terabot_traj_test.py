#! /usr/bin/env python

import roslib; roslib.load_manifest('robotnik_trajectory_control')
import rospy
import actionlib
import time
import sys

from actionlib_msgs.msg import *
from trajectory_msgs.msg import *

from control_msgs.msg import *

if __name__ == '__main__':
	rospy.init_node('follow_trajectory_test')
	client = actionlib.SimpleActionClient('rt_traj_exe/follow_joint_trajectory', FollowJointTrajectoryAction)
	client.wait_for_server()
	
	if len(sys.argv) < 2:
		option = 1
		print 'Selected option by default %d'%option
	else:
		option = int(sys.argv[1])
		print 'Selected option %d'%option
	
	goal = FollowJointTrajectoryGoal()
	goal.trajectory.header.stamp = rospy.Time()
	#goal.trajectory.joint_names = ['right_arm_4_joint', 'right_arm_2_joint', 'right_arm_1_joint', 'right_arm_3_joint']
	goal.trajectory.joint_names = ['terabot_8_joint', 'terabot_5_joint']
	tpoint1 = JointTrajectoryPoint()
	tpoint1.positions = [-0.2, 0.1]
	tpoint1.velocities = [0.1, 0.1]
	tpoint1.accelerations = [0.1, 0.1]
	tpoint1.time_from_start = rospy.Duration.from_sec(5.0)
	
	tpoint2 = JointTrajectoryPoint()
	tpoint2.positions = [-0.6, 0.3]
	tpoint2.velocities = [0.1, 0.11]
	tpoint2.accelerations = [0.1, 0.1]
	tpoint2.time_from_start = rospy.Duration.from_sec(5.0)
	
	
	goal.trajectory.points = [tpoint1, tpoint2]
	
	# Sends 3 trajs
	if option == 1:
		
		rospy.loginfo('OPTION 1: sending trajectory 1')
		# Fill in the goal here
		client.send_goal(goal)
		rospy.loginfo('waiting for result')
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
		
		
			
		print 'Result is %s'%client.get_result()

	# Sends 3 trajs
	if option == 2:
		
		tpoint1.positions = [-0.3, 0.2]
		tpoint2.positions = [0.0, 0.0]
		rospy.loginfo('OPTION 1: sending trajectory 1')
		# Fill in the goal here
		client.send_goal(goal)
		rospy.loginfo('waiting for result')
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
		
		
			
		print 'Result is %s'%client.get_result()
