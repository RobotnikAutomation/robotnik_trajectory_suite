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
	goal.trajectory.joint_names = ['mico_joint_1', 'mico_joint_2', 'mico_joint_3', 'mico_joint_4', 'mico_joint_5', 'mico_joint_6']
	tpoint1 = JointTrajectoryPoint()
	tpoint1.positions = [-1.6554960347361036, -1.6516461690642847, 0.1944244748554111, -1.0983675283814525, 1.679086049355918, 3.40815204236352]		                    
	tpoint1.velocities = [0.3, 0.1, 0.1, 0.1, 0.1, 0.1]
	tpoint1.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
	tpoint1.time_from_start = rospy.Duration.from_sec(5.0)
	
	tpoint2 = JointTrajectoryPoint()
	tpoint2.positions = [-0.8229356852782783, -1.197347106087562, -0.505311381024976, -1.5505664225225386, 2.2443348001903303, 3.453371851882796]	
	tpoint2.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
	tpoint2.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
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

		time.sleep(2.0)		
		tpoint1.positions = [-1.6554960347361036, -1.6516461690642847, 0.1944244748554111, -1.0983675283814525, 1.679086049355918, 3.40815204236352]		                    
		tpoint2.positions = [-0.8229356852782783, -1.197347106087562, -0.505311381024976, -1.5505664225225386, 2.2443348001903303, 3.453371851882796]

		rospy.loginfo('sending trajectory 2')
		# Fill in the goal here
		client.send_goal(goal)
		rospy.loginfo('waiting for result')
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
		print 'Result is %s'%client.get_result()
		
		time.sleep(2.0)
		
		tpoint1.positions = [-1.6554960347361036, -1.6516461690642847, 0.1944244748554111, -1.0983675283814525, 1.679086049355918, 3.40815204236352]		                    
		tpoint2.positions = [-0.8229356852782783, -1.197347106087562, -0.505311381024976, -1.5505664225225386, 2.2443348001903303, 3.453371851882796]
				
		rospy.loginfo('sending trajectory 3')
		# Fill in the goal here
		client.send_goal(goal)
		rospy.loginfo('waiting for result')
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
		print 'Result is %s'%client.get_result()
	# Sends 1 traj and owerwrite it
	elif option == 2:
		rospy.loginfo('OPTION 2: sending trajectory 1')
		# Fill in the goal here
		client.send_goal(goal)
		time.sleep(1)
		tpoint1.positions = [-1.6554960347361036, -1.6516461690642847, 0.1944244748554111, -1.0983675283814525, 1.679086049355918, 3.40815204236352]		                    
		tpoint2.positions = [-0.8229356852782783, -1.197347106087562, -0.505311381024976, -1.5505664225225386, 2.2443348001903303, 3.453371851882796]
				
		rospy.loginfo('overwrite')
		client.send_goal(goal)
		
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
	
	# Sends 1 traj and cancell it
	elif option == 3:
		rospy.loginfo('OPTION 3: sending trajectory 1')
		# Fill in the goal here
		client.send_goal(goal)
		time.sleep(1)
		rospy.loginfo('cancel')
		client.cancel_goal()
		
	# only 1 point
	elif option == 4:		
		tpoint1.positions = [-1.6554960347361036, -1.6516461690642847, 0.1944244748554111, -1.0983675283814525, 1.679086049355918, 3.40815204236352]		                    
		tpoint1.velocities = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
		tpoint1.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		goal.trajectory.points = [tpoint1]
		client.send_goal(goal)
		
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
	
	# one joint at high velocity
	elif option == 5:
		print 'EXECUTING OPTION %d'%option
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.header.stamp = rospy.Time()
		goal.trajectory.joint_names = ['right_arm_1_joint']
		tpoint1 = JointTrajectoryPoint()
		tpoint1.positions = [-1.3455]
		tpoint1.velocities = [0.3]
		tpoint1.accelerations = [0.1]
		tpoint1.time_from_start = rospy.Duration.from_sec(5.0)
	
		goal.trajectory.points = [tpoint1]
		client.send_goal(goal)
		
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
	
		time.sleep(2.0)
		tpoint1.positions = [-1.3455]
		tpoint1.velocities = [0.3]
		tpoint1.accelerations = [0.1]
	
		client.send_goal(goal)
		
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
			
	# Modifying joint values at high frequency
	# elif option == 6:

	# Moving the HAND
		
			
