#!/usr/bin/env python
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


DEFAULT_FREQ = 0.5
MAX_FREQ = 1.0


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
			
	
# 
class TrajManagerLoop:
	'''
		Sends positions to trajectory manager
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
		self.manager_topic_command_string = args['manager_topic_command']
		
		
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
		# State of the planner
		self.planner_state = PlannerState()
		
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
		self._planner_state_subscriber = rospy.Subscriber(self.planner_topic_state_string, PlannerState, self.plannerStateCb)
		self._manager_command_service = rospy.ServiceProxy(self.manager_topic_command_string, LoadState)
		
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
		#self._manager_command_service.unregister()
	
		
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
				self._manager_command_service.wait_for_service(timeout = 10)
				
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
		
		if self.planner_state.state.state == State.STANDBY_STATE:
			self.switchToState(State.READY_STATE)
		
		return

	
	def readyState(self):
		'''
			Actions performed in ready state
		'''
		if self.planner_state.state.state == State.STANDBY_STATE:
			
			if len(self.dict_keys) > 0:
				print self.dict_states[self.dict_keys[self.current_key]]
				self.sendPosition(self.dict_states[self.dict_keys[self.current_key]])

				self.current_key = (self.current_key+ 1)%len(self.dict_keys)
		else:
			self.switchToState(State.STANDBY_STATE)	
				
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
			f.close()
			self.dict_keys = self.dict_states.keys()
			self.current_key = 0
			
			#print 'Dict = %d, %s'%(len(self.dict_states), self.dict_states)
			
		except IOError, e:
			rospy.logerr('%s:loadYaml:  error openning file %s: %s'%(self.node_name, self.stored_states_path, e))
			return -1
		
		except yaml.scanner.ScannerError, e:
			rospy.logerr('%s:loadYaml:  error parsing file %s: %s'%(self.node_name, self.stored_states_path, e))
			return -1
		
		return 0
		
	
		
	def sendPosition(self, id):
		'''
			Sends a position to the manager
			@param id: id of the position to send
			@type id: string
		'''
		
		ret = self._manager_command_service.call(id)
		
		if not ret:
			rospy.logerr('%s:sendPosition: error sending position %s'%(self.node_name, id))
	
	
	def plannerStateCb(self, msg):
		'''
			Receives the state of the Planner
		'''
		self.planner_state = msg
		
			
def main():
	
	rospy.init_node("rt_traj_manager_loop")
	
						
	_name = rospy.get_name().replace('/','')
	
	arg_defaults = {
	  'topic_state': 'state',
	  'desired_freq': DEFAULT_FREQ,
	  'planner_topic_state': '/rt_traj_planner/state', 
	  'states_file': 'robot_loop_positions.yaml', 
	  'manager_topic_command': 'rt_traj_manager/load_state'
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
			
	
	rtmanager_node = TrajManagerLoop(args)
	
	rospy.loginfo('%s: starting'%(_name))

	rtmanager_node.start()


if __name__ == "__main__":
	main()
	exit()
