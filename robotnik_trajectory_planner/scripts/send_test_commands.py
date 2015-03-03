#! /usr/bin/env python

import roslib; roslib.load_manifest('robotnik_torso_planner')
import rospy
import actionlib
import time
import sys

from actionlib_msgs.msg import *
from trajectory_msgs.msg import *
from robotnik_torso_planner.msg import *

from control_msgs.msg import *

if __name__ == '__main__':
	rospy.init_node('planner_command_tests')
	
	
	if len(sys.argv) < 2:
		option = 1
		print 'Selected option by default %d'%option
	else:
		option = int(sys.argv[1])
		print 'Selected option %d'%option
	
	command_joint_pub = rospy.Publisher('rt_traj_planner/commands/joint_by_joint', JointByJoint)
	command_cartesian_pub = rospy.Publisher('rt_traj_planner/commands/cartesian_euler', CartesianEuler)
	
	msg_joint = JointByJoint()
	msg_cartesian = CartesianEuler()
	
	# POSITION CONTROL JOINT BY JOINT
	if option == 1:
		
		rospy.loginfo('OPTION 1: JOINTBYJOINT POSITION')
		
		#msg_joint.joints = ['right_arm_1_joint', 'right_arm_3_joint']
		msg_joint.joints = ['right_arm_1_joint']
		msg_joint.values = [0.0]
		msg_joint.time_plan = 4.0
		time.sleep(1.0)
		
		command_joint_pub.publish(msg_joint)
		
		'''rospy.loginfo('OPTION 1: sleep')
		time.sleep(4.0)
		
		msg_joint.values = [0.0, 0.0]
		command_joint_pub.publish(msg_joint)
		'''
	
	# VELOCITY CONTROL JOINT BY JOINT	
	elif option == 2:
		rospy.loginfo('OPTION 2: JOINTBYJOINT VELOCITY')
		
		#msg_joint.joints = ['right_arm_2_joint', 'right_arm_1_joint']
		msg_joint.joints = ['right_arm_1_joint']
		#msg_joint.values = [0.01, 0.1]
		msg_joint.values = [0.1]
		time.sleep(1.0)
		for i in range(200):
			command_joint_pub.publish(msg_joint)
			time.sleep(0.05)
		
			
	# VELOCITY CONTROL CARTESIAN
	elif option == 3:
		rospy.loginfo('OPTION 3: CARTESIAN VELOCITY')
		
		msg_cartesian.x = 0.0
		msg_cartesian.y = 0.00
		msg_cartesian.z = 0.05
		msg_cartesian.pitch = 0.00
		msg_cartesian.roll = 0.00
		 
		time.sleep(1.0)
		for i in range(250):
			command_cartesian_pub.publish(msg_cartesian)
			time.sleep(0.05)
		
			
		
	# Sends 1 traj and owerwrite it
	'''elif option == 2:
		
	elif option == 3:
		
	elif option == 4:
		
	elif option == 5:
		
	elif option == 6:'''

	
