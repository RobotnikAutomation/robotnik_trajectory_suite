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

import yaml
import os
import roslib; roslib.load_manifest('robotnik_trajectory_manager')
import rospy
from threading import Thread, Timer
import string
import time
import sys
import rospkg

from robotnik_msgs.msg import State
from robotnik_trajectory_planner.msg import State as PlannerState
from robotnik_trajectory_planner.msg import Trajectory, PointTraj

import std_msgs.msg as std_msgs
from robotnik_trajectory_manager.srv import SaveState, LoadState, GetStates, GetJointsState
from rospy.exceptions import *

import moveit_commander
import moveit_msgs.msg

DEFAULT_FREQ = 5.0
MAX_FREQ = 10.0


def get_param(name, value=None):
	'''
		Function from joint_state_publisher
		@param name: name of the param toget
		@return the param or the default value
	'''
	
	private = "~%s" % name
	if rospy.has_param(private):
		return rospy.get_param(private)
	elif rospy.has_param(name):
		return rospy.get_param(name)
	else:
		return value
			
	
# Class to simulate the trajectory execution
class TrajManager:
	'''
		Simulates join_state_publisher
		Offers the action service FollowJointTrajectoryAction
	'''
		
	def __init__(self, args):
		
		self.node_name = rospy.get_name().replace('/','')
		self.desired_freq = args['desired_freq'] 
		# Checks value of freq
		if self.desired_freq <= 0.0 or self.desired_freq > MAX_FREQ:
			rospy.loginfo('%s::init: Desired freq (%f) is not possible. Setting desired_freq to %f'%(self.node_name,self.desired_freq, DEFAULT_FREQ))
			self.desired_freq = DEFAULT_FREQ
	
		# Topic where component rt_traj_planner will publish its state
		self.planner_topic_state_string = args['planner_topic_state']
		# Filename of the the config file where it'll read the stored positions
		self.stored_state_filename = args['states_file']
		# Topic where the component will publish the commands
		self.planner_topic_command_string = args['planner_topic_command']
		
		
		self.real_freq = 0.0
		
		# Saves the state of the component
		self.state = State.INIT_STATE
		# Saves the previous state
		self.previous_state = State.INIT_STATE
		
		# flag to control the initialization of the component
		self.initialized = False
		# flag to control the initialization of ROS stuff
		self.ros_initialized = False
		# flag to control that the control loop is running
		self.running = False
		# Variable used to control the loop frequency
		self.time_sleep = 1.0 
		# State msg to publish
		self.msg_state = State()
		# Timer to publish state
		self.publish_state_timer = 1
		
		# Save all the available states
		# { 'id': {joints: ['j1', 'j2', ...], values: [1.3321, 0.1234111, ...], group: 'left_arm'}, 'id2': ....}
		self.dict_states = {}
		
		# Gets the system path to the config file
		rp = rospkg.RosPack()
		self.stored_states_path = os.path.join(rp.get_path('robotnik_trajectory_manager'), 'config', self.stored_state_filename)
		
		# Tries to load the config file in memory
		if self.loadYaml() == 0:
			rospy.loginfo('%s::init: Config file loaded successfully'%(self.node_name))
		else:
			rospy.logerr('%s::init: Config file not loaded'%(self.node_name))
			
		self.t_publish_state = Timer(self.publish_state_timer, self.publishROSstate)
			
		
	def setup(self):
		'''
			Initializes de hand
			@return: True if OK, False otherwise
		'''
		self.initialized = True
		
		return 0
		
	def rosSetup(self):
		'''
			Creates and inits ROS components
		'''
		if self.ros_initialized:
			return 0
		
		# PUBLISHERS
		self._state_publisher = rospy.Publisher('%s/state'%self.node_name, State)
		self._planner_command_publisher = rospy.Publisher(self.planner_topic_command_string, Trajectory)
		# SERVICES
		self._save_state_service = rospy.Service('%s/save_state'%self.node_name, SaveState, self.saveStateServiceCb)
		self._load_state_service = rospy.Service('%s/load_state'%self.node_name, LoadState, self.loadStateServiceCb)
		self._get_states_service = rospy.Service('%s/get_states'%self.node_name, GetStates, self.getStatesServiceCb)
		self._get_joints_state_service = rospy.Service('%s/get_joints_state'%self.node_name, GetJointsState, self.getJointsStateServiceCb)
		
		# Creates an interface with moveit commander
		self.moveit_commander_iface = moveit_commander.RobotCommander()
		
		
		self.ros_initialized = True
			
		self.publishROSstate()
		
		return 0
		
		
	def shutdown(self):
		'''
			Shutdowns device
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.initialized:
			return -1
		rospy.loginfo('%s::shutdown'%self.node_name)
		
		# Cancels current timers
		self.t_publish_state.cancel()
		
		self.initialized = False
		
		return 0
	
	
	def rosShutdown(self):
		'''
			Shutdows all ROS components
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.ros_initialized:
			return -1
		
		# Performs ROS topics & services shutdown
		self._state_publisher.unregister()
		self._planner_command_publisher.unregister()
		self._save_state_service .shutdown()
		self._load_state_service .shutdown()
		
		self.ros_initialized = False
		
		return 0
			
	
	def stop(self):
		'''
			Creates and inits ROS components
		'''
		self.running = False
		
		return 0
	
	
	def start(self):
		'''
			Runs ROS configuration and the main control loop
			@return: 0 if OK
		'''
		
		if self.running:
			return 0
			
		self.running = True
		
		self.controlLoop()
		
		return 0
	
	
	def controlLoop(self):
		'''
			Main loop of the component
			Manages actions by state
		'''
		
		while self.running and not rospy.is_shutdown():
			t1 = time.time()
			
			if self.state == State.INIT_STATE:
				self.initState()
				
			elif self.state == State.STANDBY_STATE:
				self.standbyState()
				
			elif self.state == State.READY_STATE:
				self.readyState()
				
			elif self.state == State.EMERGENCY_STATE:
				self.emergencyState()
				
			elif self.state == State.FAILURE_STATE:
				self.failureState()
				
			elif self.state == State.SHUTDOWN_STATE:
				self.shutdownState()
				
			self.allState()
			
			t2 = time.time()
			tdiff = (t2 - t1)
			
			
			t_sleep = self.time_sleep - tdiff
			
			if t_sleep > 0.0:
				rospy.sleep(t_sleep)
			
			t3= time.time()
			self.real_freq = 1.0/(t3 - t1)
		
		self.running = False
		# Performs component shutdown
		self.shutdownState()
		# Performs ROS shutdown
		self.rosShutdown()
		rospy.loginfo('%s::controlLoop: exit control loop'%self.node_name)
		
		return 0
		
	
	
	def rosPublish(self):
		'''
			Publish topics at standard frequency
		'''
		
		
		return 0
		
	
	
	def initState(self):
		'''
			Actions performed in init state
		'''
		
		if not self.initialized:
			self.setup()
		
		if not self.ros_initialized:
			self.rosSetup()
			
		if self.initialized and self.ros_initialized:
			
			try:
				# Tries to get one message from planner to verify that the component is running
				planner_state = rospy.wait_for_message(self.planner_topic_state_string, PlannerState, 2)
				
				#if planner_state.state.state != State.INIT_STATE:
				self.switchToState(State.STANDBY_STATE)
				#else:
				#rospy.loginfo('%s::initState: planner on init state'%(self.node_name))
			except rospy.ROSException, e:
				rospy.logerr('%s::initState: No response from planner: %s'%(self.node_name,e))
			except rospy.ROSInterruptException, e:
				rospy.logerr('%s::initState: Received interruption: %s'%(self.node_name,e))
		
		return

	
	def standbyState(self):
		'''
			Actions performed in standby state
		'''
		self.switchToState(State.READY_STATE)
		
		return

	
	def readyState(self):
		'''
			Actions performed in ready state
		'''
		
				
		return
	
	
	def shutdownState(self):
		'''
			Actions performed in shutdown state 
		'''
		if self.shutdown() == 0:
			self.switchToState(State.INIT_STATE)
		
		return
	
	
	def emergencyState(self):
		'''
			Actions performed in emergency state
		'''
		
		return
	
	
	def failureState(self):
		'''
			Actions performed in failure state
		'''
		
			
		return
	
	
	def switchToState(self, new_state):
		'''
			Performs the change of state
		'''
		if self.state != new_state:
			self.previous_state = self.state
			self.state = new_state
			rospy.loginfo('%s::switchToState: %s'%(self.node_name, self.stateToString(self.state)))
			if self.previous_state == State.INIT_STATE:
				self.time_sleep = 1.0 / self.desired_freq
		
		return
		
	
	def allState(self):
		'''
			Actions performed in all states
		'''
		if self.ros_initialized:
			self.rosPublish()
		
		return
	
	
	def stateToString(self, state):
		'''
			@param state: state to convert
			@type state: State
			@returns the equivalent string of the state
		'''
		if state == State.INIT_STATE:
			return 'INIT_STATE'
				
		elif state == State.STANDBY_STATE:
			return 'STANDBY_STATE'
			
		elif state == State.READY_STATE:
			return 'READY_STATE'
			
		elif state == State.EMERGENCY_STATE:
			return 'EMERGENCY_STATE'
			
		elif state == State.FAILURE_STATE:
			return 'FAILURE_STATE'
			
		elif state == State.SHUTDOWN_STATE:
			return 'SHUTDOWN_STATE'
		else:
			return 'UNKNOWN_STATE'
	
	
	def substateToString(self, substate):
		'''
			@param substate: substate to switch to
			@type substate: int
			@returns the equivalent string of the substate
		'''
		if substate == ACTIVE_SUBSTATE:
			return 'ACTIVE_SUBSTATE'
				
		elif substate == PAUSE_SUBSTATE:
			return 'PAUSE_SUBSTATE'
			
		elif substate == CANCEL_SUBSTATE:
			return 'CANCEL_SUBSTATE'
			
		elif substate == IDLE_SUBSTATE:
			return 'IDLE_SUBSTATE'
		else:
			return 'UNKNOWN_SUBSTATE'
			
	
		
	def publishROSstate(self):
		'''
			Publish the State of the component at the desired frequency
		'''
		
		self.msg_state.state = self.state
		self.msg_state.state_description = self.stateToString(self.state)
		self.msg_state.desired_freq = self.desired_freq
		self.msg_state.real_freq = self.real_freq
		self._state_publisher.publish(self.msg_state)
		
		self.t_publish_state = Timer(self.publish_state_timer, self.publishROSstate)
		self.t_publish_state.start()
				
	
	def saveStateServiceCb(self, req):
		'''
			ROS service to perform the storage the current state (joint values) of the robot or a group 
			@param req: Requested service
			@type req: robotnik_torso_manager/SaveState
		'''
		
		if self.state == State.INIT_STATE:
			rospy.loginfo('%s:saveStateServiceCb: the component cannot process request while in INIT_STATE'%self.node_name)
			return False
		else:	
			if req.group_name == '':
				rospy.logerr('%s:saveStateServiceCb: Group name not set'%self.node_name)
				return False
			elif req.id == '':
				rospy.logerr('%s:saveStateServiceCb: ID not set'%self.node_name)
				return False
			else:
				if self.moveit_commander_iface.has_group(req.group_name):
					
					try:
						group = moveit_commander.MoveGroupCommander(req.group_name)
						values = group.get_current_joint_values()
						joints = group.get_joints()
						
						if len(values) != len(joints) and len(values) > 0:
							rospy.logerr('%s:saveStateServiceCb: Request cannot be processed due to inconsistency on joint values'%self.node_name)
						else:
							# Inserts the state in the table
							self.dict_states[req.id] = {'joints': joints, 'values': values, 'group': req.group_name}
							rospy.loginfo('%s:saveStateServiceCb: saving state of the group %s with ID = %s'%(self.node_name, req.group_name, req.id))
							#print self.dict_states
							# Saves to file
							if self.saveYaml() != 0:
								rospy.logerr('%s:saveStateServiceCb: error saving states into file'%self.node_name)
								return False
					except RuntimeError, e:
						rospy.logerr('%s:saveStateServiceCb: error calling moveit_commander(%s): %s'%(self.node_name,req.group_name, e))
						return False
						
				else:
					rospy.logerr('%s:saveStateServiceCb: Selected group name %s is not available'%(self.node_name, req.group_name))
				
				#print self.moveit_commander_iface.get_current_variable_values()
				#print self.moveit_commander_iface.get_group_names()
				
			
		return True
		
	
	def loadStateServiceCb(self, req):
		'''
			ROS service to perform the load of a saved previous state (joint values) of a group
			@param req: Requested service
			@type req: robotnik_torso_manager/LoadState
		'''
		
		if self.state == State.INIT_STATE:
			rospy.loginfo('%s:loadStateServiceCb: the component cannot process request while in INIT_STATE'%self.node_name)
			return False
		else:	
			if req.id == '':
				rospy.logerr('%s:loadStateServiceCb: ID not set'%self.node_name)
				return False
			else:
				if not self.dict_states.has_key(req.id):
					rospy.logerr('%s:loadStateServiceCb: Requested id (%s) doest not exist'%(self.node_name, req.id))
					return False
				else:
					rospy.loginfo('%s:loadStateServiceCb: Going to state %s'%(self.node_name, req.id))
					self.sendCommandTraj(req.id)
					#print self.dict_states[req.id]
				
			
		return True
		
		
	def getStatesServiceCb(self, req):
		'''
			ROS service to get the available predefined/loaded states of the robot
			@param req: Requested service
			@type req: robotnik_torso_manager/GetStates
		'''
		
		
		if self.state == State.INIT_STATE:
			rospy.loginfo('%s:getStatesServiceCb: the component cannot process request while in INIT_STATE'%self.node_name)
		else:
			
			return [self.dict_states.keys()]
			# Returs the id's of all the states	
			
			
		return []
				
		
	def getJointsStateServiceCb(self, req):
		'''
			ROS service to get the available predefined/loaded states of the robot
			@param req: Requested service
			@type req: robotnik_torso_manager/GetStates
		'''
		if req.id == '':
			rospy.logerr('%s:getJointsStateServiceCb: ID not set'%self.node_name)
			return False, [], []
		else:
			if not self.dict_states.has_key(req.id):
				rospy.logerr('%s:getJointsStateServiceCb: Requested id (%s) doest not exist'%(self.node_name, req.id))
				return False, [], []
			else:
				return True, self.dict_states[req.id]['joints'], self.dict_states[req.id]['values']
				
				
		
	def loadYaml(self):
		'''
			Loads the saved positions/states from the config file into memory
			@return 0 if OK
			@return -1 if ERROR
		'''
		try:
			# opens the file in read mode
			f = open(self.stored_states_path, 'r')
			self.dict_states = yaml.safe_load(f)
			if self.dict_states is None:
				self.dict_states = {}
			#rospy.loginfo('loadYaml: dict = %s'%self.dict_states)
			f.close()
			
		except IOError, e:
			rospy.logerr('%s:loadYaml:  error openning file %s: %s'%(self.node_name, self.stored_states_path, e))
			return -1
		
		except yaml.scanner.ScannerError, e:
			rospy.logerr('%s:loadYaml:  error parsing file %s: %s'%(self.node_name, self.stored_states_path, e))
			return -1
		
		return 0
		
	
	def saveYaml(self):
		'''
			Saves positions/states from memory to file
			@return 0 if OK
			@return -1 if ERROR
		'''
		try:
			# opens the file in read mode
			f = open(self.stored_states_path, 'w')
			f.write( yaml.dump(self.dict_states, default_flow_style=False))
			f.close()
			
		except IOError, e:
			rospy.logerr('%s:loadYaml:  error openning file %s: %s'%(self.node_name, self.stored_states_path, e))
			return -1
			
		return 0
		
		
	def sendCommandTraj(self, id):
		'''
			Sends a trajectory command to the planner
			@param id: id of the state to send
			@type id: string
		'''
		# Creates the msg
		msg = Trajectory()
		point = PointTraj()
		point.id = id
		point.group = self.dict_states[id]['group']
		point.joint_names = self.dict_states[id]['joints']
		point.joint_values = self.dict_states[id]['values']
		
		# Sends only one point
		msg.points = [point]
		
		self._planner_command_publisher.publish(msg)
		
		
		
			
def main():
	
	moveit_commander.roscpp_initialize([])
	
	rospy.init_node("rt_traj_manager")
	
						
	_name = rospy.get_name().replace('/','')
	
	arg_defaults = {
	  'topic_state': 'state',
	  'desired_freq': DEFAULT_FREQ,
	  'planner_topic_state': '/rt_traj_planner/state', 
	  'states_file': 'robot_state_positions.yaml', 
	  'planner_topic_command': 'rt_traj_planner/commands/trajectory'
	}
	
	args = {}
	
	for name in arg_defaults:
		try:
			if rospy.search_param(name): 
				args[name] = rospy.get_param('%s/%s'%(_name, name)) # Adding the name of the node, because the para has the namespace of the node
			else:
				args[name] = arg_defaults[name]
			#print name
		except rospy.ROSException, e:
			rospy.logerror('%s: %s'%(e, _name))
			
	
	rtmanager_node = TrajManager(args)
	
	rospy.loginfo('%s: starting'%(_name))

	rtmanager_node.start()


if __name__ == "__main__":
	main()
	exit()
