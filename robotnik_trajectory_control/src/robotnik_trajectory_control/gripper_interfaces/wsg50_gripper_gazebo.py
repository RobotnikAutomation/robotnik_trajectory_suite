#!/usr/bin/env python
"""
	Software License Agreement (BSD License)

	Copyright (c) 2015 Robotnik Automation SLL. All Rights Reserved.

	Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

	2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
	   in the documentation and/or other materials provided with the distribution.

	3. The name of the author may not be used to endorse or promote products derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY ROBOTNIK AUTOMATION SLL "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
	AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
	OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
	AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
	EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import roslib; roslib.load_manifest('robotnik_trajectory_control')
import rospy
import actionlib

from control_msgs.msg import *
from actionlib_msgs.msg import *
from gripper_interface import StandardGripperInterface
from std_msgs.msg import Float64

class Wsg50GripperGazeboInterface(StandardGripperInterface):
	"""
		Class to set the position of the Gripper
		Moves the joint through a postion command topic read by Gazebo ros_control
		By default the type of controller is position_controllers/JointPositionController
	"""
	def __init__(self, joints):
		self.joints = joints
		self.min_opening_ = rospy.get_param('~min_opening', 0.0)
		self.max_opening_ = rospy.get_param('~max_opening', 0.10)
		self.f1_origin_ = rospy.get_param('~f1_origin', 0.0)
		self.f2_origin_ = rospy.get_param('~f2_origin', 0.0)
		self.f1_direction_ = rospy.get_param('~f1_direction', -1.0)
		self.f2_direction_ = rospy.get_param('~f2_direction', 1.0)
		# Timeout for any action (secs)
		self.timeout_action_ = rospy.get_param('~timeout_action', 10.0)
		self.f1_command_topic_ = rospy.get_param('~f1_command_topic', "/wsg50_f1/command")
		self.f2_command_topic_ = rospy.get_param('~f2_command_topic', "/wsg50_f2/command")
		
		self.center = rospy.get_param('~center', 0.0)
		
		if len(self.joints)!= 2:
			rospy.logerr('Wsg50GripperGazeboInterface: incorrect number of joints %s'%str(self.joints))
			exit()
	
	def setup(self):
		"""
			Setups the component.
			Tries to connect the action server
			@return 0 if OK
			@return -1 if ERROR
		"""
		# Creates publisher to set the position
		self.f1_position_publisher = rospy.Publisher(self.f1_command_topic_, Float64)
		self.f2_position_publisher = rospy.Publisher(self.f2_command_topic_, Float64)
		# Creates subscribers to read the current position
		#self.f1_position_subscriber = rospy.Publisher(self.f1_state_topic_, Float64)
		#self.f2_position_subscriber = rospy.Publisher(self.f2_state_topic_, Float64)
		
		
		return 0
		
		
	def setPositionGoal(self, position_goal):
		"""
			Sets the position of the gripper
			@param postion as GripperCommand
			@return 0 if it's send successfully
			@return -1 if ERROR		
		"""
		desired_position = position_goal.command.position
		if position_goal.command.position < self.min_opening_:
			desired_position = self.min_opening_
		elif position_goal.command.position > self.max_opening_:
			desired_position = self.max_opening_
		
		#joints = self.convertGripperDistanceToJointValues(desired_position)
		j1_pos = self.f1_direction_*desired_position/2.0
		j2_pos = self.f2_direction_*desired_position/2.0
		self.f1_position_publisher.publish(j1_pos)
		self.f2_position_publisher.publish(j2_pos)
		
		
		return 0
		
		
	def isGoalReached(self):
		"""
			Returns 0 if the position Goal is reached
			Returns -1 if the position hasn't been reached yet
			Returns -2 if it has been an error
		"""
		
		return 0	
		
		
	def cancelGoal(self):
		"""
			Cancels the current position
		"""
		
		
		return 0
	
		
		
