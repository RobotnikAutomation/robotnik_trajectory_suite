/*! \class RtTrajPlanner
 *  \file RtTrajPlanner.cpp
 *	\author Robotnik Automation S.L.L
 *	\version 1.0.0
 *	\date 2015
 *  \brief Class to plan and control trajectories
 * 
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
 * 
 */

#include <robotnik_trajectory_planner/rt_traj_planner.h>



/*! \fn RtTrajPlanner::RtTrajPlanner()
 *  \brief Constructor by default
 *	\param hz as double, sets the desired frequency of the controlthread
 *	\param h as ros::NodeHandle, ROS node handle
*/
RtTrajPlanner::RtTrajPlanner(double hz, ros::NodeHandle h):nh(h), pnh("~"), desired_freq(hz){
	// Set main flags to false
	ros_initialized = initialized = running = false;
	
	if(desired_freq <= 0.0)
		desired_freq = DEFAULT_THREAD_DESIRED_HZ;
	
	state = robotnik_msgs::State::INIT_STATE;
	// Realizar para cada una de las clases derivadas
	component_name.assign("RtTrajPlanner");
	
	// Values by default
	control_mode = POSITION;
	//control_mode = VELOCITY;
	control_type = JOINTBYJOINT;
	goal_active = false;
	// By default we need to init arms
	substate_init = INIT_SETUP;
	initialization_request = false;
	
	// Initializes received messages
	joint_by_joint_msg.processed = cartesian_euler_msg.processed = trajectory_msg.processed = true;
	// Initializes time to control the control state reception
	t_received_control_state = ros::Time::now();
}

/*! \fn RtTrajPlanner::~RtTrajPlanner()
 * Destructor by default
*/
RtTrajPlanner::~RtTrajPlanner(){
	
	//delete current_move_group;
}

/*! \fn int RtTrajPlanner::setup()
 * Configures and initializes the component
 * \return OK
 * \return INITIALIZED if the component is already intialized
 * \return ERROR
*/
int RtTrajPlanner::setup(){
	// Checks if has been initialized
	if(initialized){
		ROS_INFO("%s::Setup: Already initialized",component_name.c_str());
		
		return INITIALIZED;
	}
	
	//
	///////////////////////////////////////////////////
	// Setups the component or another subcomponents if it's necessary //
	///////////////////////////////////////////////////
	

	initialized = true;

	return OK;
}

/*! \fn int RtTrajPlanner::shutDown()
 * Closes and frees the reserved resources
 * \return OK
 * \return ERROR if fails when closes the devices
 * \return RUNNING if the component is running
 * \return NOT_INITIALIZED if the component is not initialized
*/
int RtTrajPlanner::shutdown(){
	
	if(running){
		ROS_INFO("%s::Shutdown: Impossible while thread running, first must be stopped",component_name.c_str());
		return THREAD_RUNNING;
	}
	if(!initialized){
		ROS_INFO("%s::Shutdown: Impossible because of it's not initialized", component_name.c_str());
		return NOT_INITIALIZED;
	}
	
	//
	///////////////////////////////////////////////////////
	// ShutDowns another subcomponents if it's necessary //
	///////////////////////////////////////////////////////
	
	
	initialized = false;

	return OK;
}


/*! \fn int RtTrajPlanner::start()
 * Starts the control thread of the component and its subcomponents
 * \return OK
 * \return RUNNING if it's already running
 * \return NOT_INITIALIZED if the component is not initialized
*/
int RtTrajPlanner::start(){
	// Performs ROS setup
	rosSetup();
	
	if(running){
		ROS_INFO("%s::start: the component's thread is already running", component_name.c_str());
		return THREAD_RUNNING;
	}
	
	ROS_INFO("%s started", component_name.c_str());
	
	running = true;
	
	// Executes the control loop
	controlLoop();
	
	return OK;

}

/*! \fn int RtTrajPlanner::stop()
 * Stops the control thread of the Motors
 * \return OK
 * \return ERROR if it can't be stopped
 * \return THREAD_NOT_RUNNING if the thread is not running
*/
int RtTrajPlanner::stop(){
	
	if(!running){
		ROS_INFO("%s::stop: Thread not running", component_name.c_str());
	
		return THREAD_NOT_RUNNING;
	}
	//
	///////////////////////////////////////////////////
	// Stops another subcomponents, if it's necessary //
	///////////////////////////////////////////////////
	//
	ROS_INFO("%s::Stop: Stopping the component", component_name.c_str());
	
	running = false;

	usleep(100000);

	return OK;
}

/*!	\fn void RtTrajPlanner::controlLoop()
 *	\brief All core component functionality is contained in this thread.
*/
void RtTrajPlanner::controlLoop(){
	ROS_INFO("%s::controlLoop(): Init", component_name.c_str());
	ros::Rate r(desired_freq);  
	ros::AsyncSpinner spinner(4);
	ros::Time t1, t2;
	
	//ros::spinOnce();
	spinner.start();
	while(running && ros::ok()) {
		t1 = ros::Time::now();
		switch(state){
			
			case robotnik_msgs::State::INIT_STATE:
				initState();
			break;
			
			case robotnik_msgs::State::STANDBY_STATE:
				standbyState();
			break;
			
			case robotnik_msgs::State::READY_STATE:
				readyState();
			break;
			
			case robotnik_msgs::State::SHUTDOWN_STATE:
				shutdownState();
			break;
			
			case robotnik_msgs::State::EMERGENCY_STATE:
				emergencyState();
			break;
			
			case robotnik_msgs::State::FAILURE_STATE:
				failureState();
			break;
		
		}
		
		allState();
		
		//ros::spinOnce();
		
		r.sleep();
		t2 = ros::Time::now();
		
		real_freq = 1.0/(t2 - t1).toSec();
	}
	
	shutdownState();
	// Performs ROS Shutdown
	rosShutdown();

	ROS_INFO("%s::controlLoop(): End", component_name.c_str());

}

/*!	\fn void RtTrajPlanner::initState()
 *	\brief Actions performed on initial 
 * 	Setups the component
*/
void RtTrajPlanner::initState(){
	/*if(initialization_request){
		
		switch(substate_init){
			case INIT_SETUP:
				// If component setup is successful goes to STANDBY (or READY) state
				if(setup() != ERROR){
					
					if(ac_follow_joint_traj->waitForServer(ros::Duration(1.0))){
						ROS_INFO("%s::initState: Connected to service %s", component_name.c_str(), follow_joint_traj_name.c_str());
						
						switchToSubstate(INIT_INIT_ARMS);
						
						//TEST
						//sendActionToControl(robotnik_trajectory_msgs::Actions::INIT_FINISHED);
						switchToState(robotnik_msgs::State::STANDBY_STATE);
						initialization_request = false;
					}else{
						ROS_ERROR("%s::initState: Waiting for service %s", component_name.c_str(), follow_joint_traj_name.c_str());
					}	
				}
			break;
		}
			
			
	}*/
	switchToState(robotnik_msgs::State::STANDBY_STATE);
}

/*!	\fn void RtTrajPlanner::shutdownState()
 *	\brief Actions performed on Shutdown state
*/
void RtTrajPlanner::shutdownState(){
	
	if(shutdown() == OK){
		switchToState(robotnik_msgs::State::INIT_STATE);
	}
}

/*!	\fn void RtTrajPlanner::standbyState()
 *	\brief Actions performed on Standby state
*/
void RtTrajPlanner::standbyState(){
	
	// Checks whether there are new messages or not, and sets the control type
	ros::Time t_now = ros::Time::now();
	//static double t_new = 1.0/desired_freq;
	
	// If reinitialization is requested
	if(initialization_request){
		switchToSubstate(INIT_INIT_ARMS);
		switchToState(robotnik_msgs::State::INIT_STATE);
	}
	
	// JointByJoint?
	if(!joint_by_joint_msg.processed  and (t_now - joint_by_joint_msg.t).toSec() <= WATCHDOG_COMMAND ){
		ROS_INFO("%s::standbyState: Received new JointByJoint message", component_name.c_str());
		control_type = JOINTBYJOINT;
		if(control_mode == VELOCITY){
			// Updates auxiliar kinematic state before switching to ready
			//aux_kinematic_state->getJointStateGroup(s_robot_group)->setVariableValues(move_group_robot->getCurrentJointValues());
			
			//joint_model_group->setVariableValues(move_group_robot->getCurrentJointValues());
			aux_kinematic_state->setJointGroupPositions(groups2joint_model_group[current_move_group->name_group], move_group_robot->getCurrentJointValues());
		}
		switchToState(robotnik_msgs::State::READY_STATE);
	}
	
	// Cartesian Euler?
	else if(!cartesian_euler_msg.processed  and (t_now - cartesian_euler_msg.t).toSec() <= WATCHDOG_COMMAND ){
		ROS_INFO("%s::standbyState: Received new CartesianEuler message", component_name.c_str());
		control_type = CARTESIAN_EULER;
		if(control_mode == VELOCITY){
			// Updates auxiliar kinematic state before switching to ready
			//aux_kinematic_state->getJointStateGroup(s_robot_group)->setVariableValues(move_group_robot->getCurrentJointValues());
			aux_kinematic_state->setJointGroupPositions(groups2joint_model_group[current_move_group->name_group], move_group_robot->getCurrentJointValues());
		}
		switchToState(robotnik_msgs::State::READY_STATE);
	}
	
	// Trajectory ?
	else if(!trajectory_msg.processed){
		ROS_INFO("%s::standbyState: Received new Trajectory message", component_name.c_str());
		control_type = TRAJECTORY;
		switchToState(robotnik_msgs::State::READY_STATE);
	}
	
}


/*!	\fn void RtTrajPlanner::readyState()
 *	\brief Actions performed on ready state
*/
void RtTrajPlanner::readyState(){
	
	
	if(control_type == CARTESIAN_EULER){
			
		//
		// CONTROL POSITION
		if(control_mode == POSITION){
			sendCartesianEulerPosition();
			
		}else{
		// 
		// CONTROL VELOCITY
			sendCartesianEulerVelocity();
		}
		
		if(not goal_active)
			switchToState(robotnik_msgs::State::STANDBY_STATE);
	}
	
	//
	//	JOINTBYJOINT
	//
	else if(control_type == JOINTBYJOINT){
		//
		// CONTROL POSITION
		if(control_mode == POSITION){
			sendJointByJointPosition();	
		}else{
		// 
		// CONTROL VELOCITY
			sendJointByJointVelocity();
		}
		
		if(not goal_active)
			switchToState(robotnik_msgs::State::STANDBY_STATE);
	}
	
	//
	//	TRAJECTORY
	//
	else if(control_type == TRAJECTORY){
		int ret_traj = sendTrajectory();
		if(ret_traj == 0 or ret_traj == -2 or ret_traj == -4 or ret_traj == -3)
			switchToState(robotnik_msgs::State::STANDBY_STATE);		
	}
	
}


/*!	\fn int RtTrajPlanner::sendCartesianEulerPosition
 *	\brief Executes the Cartesian-Euler Type control in position control
 * 			Uses the class attribute catersianeuler_msg 
 *  \return 0 

*/
int RtTrajPlanner::sendCartesianEulerPosition(){
	std::vector<double> current_joint_values, robot_joint_values;
	std::vector<std::string> current_group_joints;
	static double t_planning = 0.5;
	moveit::planning_interface::MoveGroup::Plan my_plan;
	collision_detection::CollisionResult collision_result;
	double dx = 0.0, dy = 0.0, dz = 0.0, dpitch = 0.0, droll = 0.0, dyaw = 0.0;
	bool found_ik = false;
	double dist = 0.0;
	static double dt = (1.0/desired_freq);
	double max_diff = 0.0;
	//ROS_INFO("TDIFF = %.3f",(ros::Time::now() - cartesian_euler_msg.t).toSec());
		
	// local copy of class attribute to avoid overwritte 
	CartesianEulerMsg cartesian_euler_msg = this->cartesian_euler_msg;
	
	if(!cartesian_euler_msg.processed){
		dx = cartesian_euler_msg.msg.x;
		dy = cartesian_euler_msg.msg.y;
		dz = cartesian_euler_msg.msg.z;
		dpitch = cartesian_euler_msg.msg.pitch;
		droll = cartesian_euler_msg.msg.roll;
		dyaw = cartesian_euler_msg.msg.yaw;
		
		if( dx != 0.0 or dy != 0.0 or dz != 0.0 or dpitch != 0.0 or droll != 0.0 or dyaw != 0.0){
			
			this->m_transform << 1.0, 0.0, 0.0, dx,
				0.0, 1.0, 0.0,  dy,
				0.0, 0.0, 1.0,  dz,
				0.0, 0.0, 0.0, 1.0;

			Eigen::Affine3d rotMatPitch( Eigen::AngleAxisd( dpitch, ( Eigen::Vector3d() << 0.0, 1.0, 0.0 ).finished() ) );
			this->m_transform = this->m_transform * rotMatPitch.matrix();
			Eigen::Affine3d rotMatRoll( Eigen::AngleAxisd( droll, ( Eigen::Vector3d() << 1.0, 0.0, 0.0 ).finished() ) );
			this->m_transform = this->m_transform * rotMatRoll.matrix();
			Eigen::Affine3d rotMatYaw( Eigen::AngleAxisd( dyaw, ( Eigen::Vector3d() << 0.0, 0.0, 1.0 ).finished() ) );
			this->m_transform = this->m_transform * rotMatYaw.matrix();
			
			// Gets current joint values from move_group interface
			robot_joint_values = move_group_robot->getCurrentJointValues();
			current_joint_values = current_move_group->mg->getCurrentJointValues();
			current_group_joints = current_move_group->joint_names;
			// Updates Kinematic state of the whole robot
			//if(not kinematic_state->getJointStateGroup(s_robot_group)->setVariableValues(robot_joint_values))
			//	ROS_ERROR("%s::readyState: ERROR setting variable values for current kinematic state(CARTESIAN-EULER:POSITION)", component_name.c_str());
			//kinematic_state->setVariablePositions(robot_joint_values);
			kinematic_state->setJointGroupPositions(joint_robot_model_group, robot_joint_values);
			
			const Eigen::Affine3d &end_effector_state = kinematic_state->getFrameTransform(current_move_group->mg->getEndEffectorLink().c_str());	
			
			// Applying transformation
			Eigen::Affine3d next_end_effector_state = end_effector_state;
			next_end_effector_state = next_end_effector_state * this->m_transform;
			
			// Calcs Inverse Kinematics
			//found_ik = kinematic_state->getJointStateGroup(current_move_group->name_group)->setFromIK(next_end_effector_state, 5.0, 0.1);
			found_ik = kinematic_state->setFromIK(groups2joint_model_group[current_move_group->name_group], next_end_effector_state, 5.0, 0.1);
			
			if(found_ik){
				if(checkCollision(&collision_result, kinematic_state)){
					ROS_ERROR("%s::readyState: Robot in collision (CARTESIAN-EULER:POSITION)", component_name.c_str());
				}else{
				// NO-COLLISION
					if(!kinematic_state->satisfiesBounds()){
						ROS_ERROR("%s::readyState: Robot does not satisfies bounds (JOINTBYJOINT:POSITION)", component_name.c_str());
						kinematic_state->enforceBounds();
					}
					// SATIFIES BOUNDS
					std::vector<double> transformed_joint_values;
					double jump_constant = 0.2;
					bool bJump = false;
					
					// Check that next joint values doesn't produce a jump
					//kinematic_state->getJointStateGroup(current_move_group->name_group)->getVariableValues(transformed_joint_values);
					//kinematic_state->setVariablePositions(transformed_joint_values);
					const std::vector<int> current_group_joint_indexes = groups2joint_model_group[current_move_group->name_group]->getVariableIndexList();
					const double * current_joint_positions = kinematic_state->getVariablePositions();
					
					for(int i = 0; i < current_group_joint_indexes.size(); i++){
						transformed_joint_values.push_back(current_joint_positions[current_group_joint_indexes[i]]);
					}
					
					for(std::size_t i = 0; i < transformed_joint_values.size(); ++i)
					{
						dist = fabs ( transformed_joint_values[i] - current_joint_values[i] );
						//ROS_INFO("Diff on joint %d =  %5.2f", (int)i+1, dist);
						if (dist > jump_constant) {   // TODO - this limit as parameter, it could be a function of speed
							ROS_INFO("%s::readyState: JUMP in joint (%s) -> %5.2lf", component_name.c_str(), current_group_joints[i].c_str(), dist);
							bJump = true;
						}
					} 
					//bJump = false;
					if(!bJump){
						current_move_group->mg->clearPoseTarget();
						current_move_group->mg->setPoseTarget(next_end_effector_state);
						current_move_group->mg->setPlanningTime(DEFAULT_T_PLAN);	// TODO: Specify velocity
						// Gets the plan
						bool success = current_move_group->mg->plan(my_plan);
						if(!success){
							ROS_ERROR("%s::readyState: Error getting the plan (CARTESIAN:POSITION)", component_name.c_str());
						}else{
							// Sending via action client
							control_msgs::FollowJointTrajectoryGoal goal_;
							// Control max velocity restrictions for every point
							for(int i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); i++){
								controlMaxVelocities(my_plan.trajectory_.joint_trajectory.points[i].velocities, MAX_JOINT_VEL_CARTESIAN);
							}
							goal_.trajectory = my_plan.trajectory_.joint_trajectory;
							
							goal_active = true;
							//ROS_INFO("%s::readyState: Sending trajectory! (JOINTBYJOINT:POSITION)", component_name.c_str());
							// If there's any other active trajectory, overwrites it
							this->ac_follow_joint_traj->sendGoal(goal_);
						}
						/*// Prepare msg
						trajectory_msgs::JointTrajectoryPoint p;
					  
						p.positions = current_joint_values;
						
						vector<double> velocities(current_joint_values.size(), DEFAULT_JOINT_VEL);
						
						p.velocities = velocities;
						
						
						// Acceleration constant
						vector<double> accelerations(current_joint_values.size(), DEFAULT_JOINT_ACCEL);

						p.accelerations = accelerations;  
						
						trajectory_msgs::JointTrajectory t;
						t.joint_names = current_group_joints;
						t.points.push_back(p);
											
						control_msgs::FollowJointTrajectoryGoal goal_;
						goal_.trajectory = t; 

						goal_active = true;
							
						// If there's any other active trajectory, overwrites it
						this->ac_follow_joint_traj->sendGoal(goal_);*/
					}		
					
				
				}
						
			}else{
				ROS_ERROR("%s::readyState: IK NOT FOUND (CARTESIAN-EULER:POSITION)", component_name.c_str());
			}
		}
		
		this->cartesian_euler_msg.processed = true;
	}
	actionlib::SimpleClientGoalState st = this->ac_follow_joint_traj->getState();
	//if(goal_active and (st == actionlib::SimpleClientGoalState::SUCCEEDED or st == actionlib::SimpleClientGoalState::LOST or actionlib::SimpleClientGoalState::PREEMPTED)){
	if(st != actionlib::SimpleClientGoalState::PENDING and st != actionlib::SimpleClientGoalState::ACTIVE){	
		ROS_INFO("%s::readyState: Trajectory ended (CARTESIAN-EULER:POSITION)", component_name.c_str());
		
		goal_active = false;
	}
	
}


/*!	\fn int RtTrajPlanner::sendCartesianEulerVelocity
 *	\brief Executes the Cartesian-Euler Type control in velocity control
 * 			Uses the class attribute catersianeuler_msg 
 *  \return 0 

*/
int RtTrajPlanner::sendCartesianEulerVelocity(){
	std::vector<double> current_joint_values, robot_joint_values;
	std::vector<std::string> current_group_joints;
	static double t_planning = 0.5;
	moveit::planning_interface::MoveGroup::Plan my_plan;
	collision_detection::CollisionResult collision_result;
	double dx = 0.0, dy = 0.0, dz = 0.0, dpitch = 0.0, droll = 0.0, dyaw = 0.0;
	bool found_ik = false;
	double dist = 0.0;
	static double dt = (1.0/desired_freq);
	double max_diff = 0.0;

	
	robot_state::RobotStatePtr previous_kinematic_state = robot_state::RobotStatePtr( new robot_state::RobotState( kinematic_model ) );
	*previous_kinematic_state = *aux_kinematic_state;
	// local copy of class attribute to avoid overwritte 
	CartesianEulerMsg cartesian_euler_msg = this->cartesian_euler_msg;
	
	if((ros::Time::now() - cartesian_euler_msg.t).toSec() <= WATCHDOG_COMMAND){
		//ROS_INFO("TDIFF = %.3f",(ros::Time::now() - cartesian_euler_msg.t).toSec());
		dx = dt*cartesian_euler_msg.msg.x;
		dy = dt*cartesian_euler_msg.msg.y;
		dz = dt*cartesian_euler_msg.msg.z;
		dpitch = dt*cartesian_euler_msg.msg.pitch;
		droll = dt*cartesian_euler_msg.msg.roll;
		dyaw = dt*cartesian_euler_msg.msg.yaw;
		
		if( dx != 0.0 or dy != 0.0 or dz != 0.0 or dpitch != 0.0 or droll != 0.0 or dyaw != 0.0){
			this->m_transform << 1.0, 0.0, 0.0, dx,
				0.0, 1.0, 0.0,  dy,
				0.0, 0.0, 1.0,  dz,
				0.0, 0.0, 0.0, 1.0;

			Eigen::Affine3d rotMatPitch( Eigen::AngleAxisd( dpitch, ( Eigen::Vector3d() << 0.0, 1.0, 0.0 ).finished() ) );
			this->m_transform = this->m_transform * rotMatPitch.matrix();
			Eigen::Affine3d rotMatRoll( Eigen::AngleAxisd( droll, ( Eigen::Vector3d() << 1.0, 0.0, 0.0 ).finished() ) );
			this->m_transform = this->m_transform * rotMatRoll.matrix();
			Eigen::Affine3d rotMatYaw( Eigen::AngleAxisd( dyaw, ( Eigen::Vector3d() << 0.0, 0.0, 1.0 ).finished() ) );
			this->m_transform = this->m_transform * rotMatYaw.matrix();
			
			// Gets current joint values from move_group interface
			//aux_kinematic_state->getJointStateGroup(s_robot_group)->getVariableValues(robot_joint_values);
			//aux_kinematic_state->getJointStateGroup(current_move_group->name_group)->getVariableValues(current_joint_values);
			const std::vector<int> current_group_joint_indexes = groups2joint_model_group[current_move_group->name_group]->getVariableIndexList();
			const double * current_joint_positions = aux_kinematic_state->getVariablePositions();
			int joints_size_array_ = sizeof(current_joint_positions)/sizeof(current_joint_positions[0]);
			
			for(int i = 0; i < current_group_joint_indexes.size(); i++){
				current_joint_values.push_back(current_joint_positions[current_group_joint_indexes[i]]);
			}
			
			//aux_kinematic_state->getJointStateGroup(current_move_group->name_group)->getVariableValues(current_joint_values);
			
			current_group_joints = current_move_group->joint_names;
			
			
			
			// Updates Kinematic state of the whole robot
			//if(not aux_kinematic_state->getJointStateGroup(s_robot_group)->setVariableValues(robot_joint_values))
			//	ROS_ERROR("%s::sendCartesianEulerVelocity: ERROR setting variable values for current kinematic state(CARTESIAN-EULER:VELOCITY)", component_name.c_str());
			
			Eigen::Affine3d end_effector_state = aux_kinematic_state->getFrameTransform(current_move_group->mg->getEndEffectorLink().c_str());	
			//Eigen::Affine3d end_effector_state = aux_kinematic_state->getFrameTransform("hand_right_grasp_link");	//hand_right_grasp_link
			//Eigen::Affine3d end_effector_state = aux_kinematic_state->getLinkState("hand_right_grasp_link")->getGlobalLinkTransform();
			//joint_state_group_->getRobotState()->getLinkState(this->arm_tcp_link_)->getGlobalLinkTransform();
			// Applying transformation
			Eigen::Affine3d next_end_effector_state = end_effector_state;
			next_end_effector_state = next_end_effector_state * this->m_transform;
			
			// Calcs Inverse Kinematics
			// found_ik = aux_kinematic_state->getJointStateGroup(current_move_group->name_group)->setFromIK(next_end_effector_state, 5.0, 0.1);
			found_ik = aux_kinematic_state->setFromIK(groups2joint_model_group[current_move_group->name_group], next_end_effector_state, 5.0, 0.1);
			
			if(found_ik){
				if(checkCollision(&collision_result, aux_kinematic_state)){
					ROS_ERROR("%s::sendCartesianEulerVelocity: Robot in collision (CARTESIAN-EULER:VELOCITY)", component_name.c_str());
					*aux_kinematic_state = *previous_kinematic_state;
				}else{
				// NO-COLLISION
					if(!aux_kinematic_state->satisfiesBounds()){
						ROS_ERROR("%s::sendCartesianEulerVelocity: Robot does not satisfies bounds (CARTESIAN-EULER:VELOCITY)", component_name.c_str());
						*aux_kinematic_state = *previous_kinematic_state;
						//aux_kinematic_state->enforceBounds();
					}
					// SATIFIES BOUNDS
					std::vector<double> transformed_joint_values;
					double jump_constant = 0.2;
					bool bJump = false;
					
					// Check that next joint values doesn't produce a jump
					//aux_kinematic_state->getJointStateGroup(current_move_group->name_group)->getVariableValues(transformed_joint_values);
					
					const double * transfomed_joint_positions = aux_kinematic_state->getVariablePositions();
					
					for(int i = 0; i < current_group_joint_indexes.size(); i++){
						transformed_joint_values.push_back(transfomed_joint_positions[current_group_joint_indexes[i]]);
					}
					
					vector<double> velocities(transformed_joint_values.size(), DEFAULT_JOINT_VEL);
					
					double diff_joint = 0.0;
					for(std::size_t i = 0; i < transformed_joint_values.size(); ++i)
					{	
						diff_joint = transformed_joint_values[i] - current_joint_values[i];
						velocities[i] = diff_joint / dt;
						dist = fabs ( diff_joint );
						//ROS_INFO("Diff on joint %d =  %5.2f", (int)i+1, dist);
						if (dist > jump_constant) {   // TODO - this limit as parameter, it could be a function of speed
							ROS_INFO("%s::sendCartesianEulerVelocity: JUMP in joint (%s) -> %5.2lf", component_name.c_str(), current_group_joints[i].c_str(), dist);
							bJump = true;
							*aux_kinematic_state = *previous_kinematic_state;
						}
					} 
					//bJump = false;
					if(!bJump){
						
						
						
						// Prepare msg
						trajectory_msgs::JointTrajectoryPoint p;
					  
						p.positions = current_joint_values;	
						
						
						p.velocities = velocities;
						
						// Acceleration constant
						vector<double> accelerations(current_joint_values.size(), 1.0);

						p.accelerations = accelerations;  
						
						//ROS_INFO("VELOCITIES BEFORE");
						//for(int i = 0; i < p.velocities.size(); i++){	
						//	ROS_INFO("%s->[%d] = %f", current_group_joints[i].c_str(), i, p.velocities[i]);
						//}
						controlMaxVelocities(p.velocities, MAX_JOINT_VEL_CARTESIAN);
						//ROS_INFO("VELOCITIES AFTER");
						//for(int i = 0; i < p.velocities.size(); i++){	
						//	ROS_INFO("%s->[%d] = %f", current_group_joints[i].c_str(), i, p.velocities[i]);
						//}	
						
						trajectory_msgs::JointTrajectory t;
						t.joint_names = current_group_joints;
						t.points.push_back(p);
											
						control_msgs::FollowJointTrajectoryGoal goal_;
						goal_.trajectory = t; 

						goal_active = true;
							
						// If there's any other active trajectory, overwrites it
						this->ac_follow_joint_traj->sendGoal(goal_);
						
					}		
					
				
				}
						
			}else{
				ROS_ERROR("%s::sendCartesianEulerVelocity: IK NOT FOUND (CARTESIAN-EULER:VELOCITY)", component_name.c_str());
				*aux_kinematic_state = *previous_kinematic_state;
			}
		}
		
		
		
		this->cartesian_euler_msg.processed = true;
		
	}else{
		// TIMEOUT WITHOUT RECEIVING COMMANDS
		ROS_INFO("%s::readyState: Trajectory ended (CARTESIAN:VELOCITY)", component_name.c_str());
		//switchToState(robotnik_msgs::State::STANDBY_STATE);
		goal_active = false;
	}
}


/*!	\fn void RtTrajPlanner::controlMaxVelocities(vector<double> *velocities, double max_vel)
 *	\brief Controls and modifies the velocities to not be greater than a maximum 
 * 			In case that there were values above the max, it will recalculate the proportion of the rest ones
*/
void RtTrajPlanner::controlMaxVelocities(vector<double> &velocities, double max_vel){
	// TEST OF MAX VELOCITY 
	bool b_max_checked_vel = false;
	bool b_find_max = false;
	double reduction_ratio = 0.0;
	
	while(not b_max_checked_vel){
		b_find_max = false;
		for(int i = 0; i < velocities.size(); i++){
			if(fabs(velocities[i]) > max_vel){
				reduction_ratio = max_vel / velocities[i];
				b_find_max = true;
				break;
			}
		}
		
		if(b_find_max){
			//ROS_INFO("RtTrajPlanner::controlMaxVelocities: Applying reduction speed");
			// Apply reduction speed
			for(int i = 0; i < velocities.size(); i++){
				velocities[i] *= reduction_ratio;
				//ROS_INFO("Applying reduction speed %s-> (p = %lf, v = %lf, a= %lf) ", current_group_joints[i].c_str(), p.positions[i], p.velocities[i], p.accelerations[i] );
			}
		}
		
		if(!b_find_max)
			b_max_checked_vel = true;
		
	}
}
		

/*!	\fn int RtTrajPlanner::sendJointByJointVelocity
 *	\brief Executes the Joint by joiny Type control in velocity control
 * 			Uses the class attribute jointbyjoint_msg 
 *  \return 0 

*/
int RtTrajPlanner::sendJointByJointVelocity(){
	std::vector<double> current_joint_values, robot_joint_values;
	std::vector<std::string> current_group_joints;
	static double t_planning = 0.5;
	moveit::planning_interface::MoveGroup::Plan my_plan;
	collision_detection::CollisionResult collision_result;
	double dx = 0.0, dy = 0.0, dz = 0.0, dpitch = 0.0, droll = 0.0, dyaw = 0.0;
	bool found_ik = false;
	double dist = 0.0;
	static double dt = (1.0/desired_freq);
	double max_diff = 0.0;
	
	if((ros::Time::now() - joint_by_joint_msg.t).toSec() <= WATCHDOG_COMMAND){
			
		// Gets current joint values from move_group interface
		//aux_kinematic_state->getJointStateGroup(s_robot_group)->getVariableValues(robot_joint_values);
		
		/*const std::vector<int> current_robot_joint_indexes = groups2joint_model_group[s_robot_group]->getVariableIndexList();
		const double * current_robot_joint_positions = aux_kinematic_state->getVariablePositions();
		
		//return 0;
		for(int i = 0; i < current_robot_joint_indexes.size(); i++){
			robot_joint_values.push_back(current_robot_joint_positions[current_robot_joint_indexes[i]]);
		}*/
		
		// Gets current joint values from move_group interface
		robot_joint_values = move_group_robot->getCurrentJointValues();
		current_joint_values = current_move_group->mg->getCurrentJointValues();
		current_group_joints = current_move_group->joint_names;
		//ROS_INFO("sendJointByJointVelocity: %d indexes for group %s ", current_joint_values.size(), current_move_group->name_group.c_str());
		
		//robot_joint_values = move_group_robot->getCurrentJointValues();
		
		//aux_kinematic_state->getJointStateGroup(current_move_group->name_group)->getVariableValues(current_joint_values);
		//current_joint_values = current_move_group->mg->getCurrentJointValues();
		/*const std::vector<int> current_joint_indexes = groups2joint_model_group[current_move_group->name_group]->getVariableIndexList();
		const double * current_joint_positions = aux_kinematic_state->getVariablePositions();
		
		for(int i = 0; i < current_joint_indexes.size(); i++){
			current_joint_values.push_back(current_joint_positions[current_joint_indexes[i]]);
		}*/
		
		
		std::vector<double> velocities(current_joint_values.size());
		
		// Estimate the next joint value based on velocity
		for(int32_t i = 0; i < current_joint_values.size(); i++){
			
			for(int32_t j = 0; j < joint_by_joint_msg.msg.joints.size(); j++){
				if(current_group_joints[i].compare(joint_by_joint_msg.msg.joints[j]) == 0){
					current_joint_values[i] += dt*joint_by_joint_msg.msg.values[j];
					velocities[i] = joint_by_joint_msg.msg.values[j];
					
					//ROS_INFO("%s::sendJointByJointVelocity: Setting joint %s to %.3lf", component_name.c_str(), current_group_joints[i].c_str(), current_joint_values[i]);
					break;
				}
			}					
		}
		
		robot_state::RobotStatePtr previous_kinematic_state = robot_state::RobotStatePtr( new robot_state::RobotState( kinematic_model ) );
		*previous_kinematic_state = *aux_kinematic_state;
		// Updates kinematic state after setting the values with the previous modifications
		//if(not aux_kinematic_state->getJointStateGroup(current_move_group->name_group)->setVariableValues(current_joint_values))
		//	ROS_ERROR("%s::sendJointByJointVelocity: ERROR setting variable values for new kinematic state (JOINTBYJOINT:VELOCITY)", component_name.c_str());
		//aux_kinematic_state->setVariablePositions(current_move_group->joint_names, current_joint_values);
		aux_kinematic_state->setJointGroupPositions(groups2joint_model_group[current_move_group->name_group], current_joint_values);
		
		if(checkCollision(&collision_result, aux_kinematic_state)){
			ROS_ERROR("%s::sendJointByJointVelocity: Robot in collision (JOINTBYJOINT:VELOCITY)", component_name.c_str());
			*aux_kinematic_state = *previous_kinematic_state;
			
		}else{
		// NO-COLLISION
			if(!aux_kinematic_state->satisfiesBounds()){
				ROS_ERROR("%s::sendJointByJointVelocity: Group %s, Robot does not satisfies bounds (JOINTBYJOINT:VELOCITY)", component_name.c_str(), current_move_group->name_group.c_str());
				//*aux_kinematic_state = *previous_kinematic_state;
				aux_kinematic_state->enforceBounds();
				//aux_kinematic_state->getJointStateGroup(current_move_group->name_group)->getVariableValues(current_joint_values);
				// Resets joint values to previous ones
				current_joint_values.clear();
				const std::vector<int> current_joint_indexes = groups2joint_model_group[current_move_group->name_group]->getVariableIndexList();
				const double * current_joint_positions = aux_kinematic_state->getVariablePositions();
				for(int i = 0; i < current_joint_indexes.size(); i++){
					current_joint_values.push_back(current_joint_positions[current_joint_indexes[i]]);
				}
				//aux_kinematic_state->setVariablePositions(current_move_group->joint_names, current_joint_values);
			}
			// SATIFIES BOUNDS
			// Prepare msg
			trajectory_msgs::JointTrajectoryPoint p;
		  
			p.positions = current_joint_values;

			p.velocities = velocities;
			
			controlMaxVelocities(p.velocities, MAX_JOINT_VEL_JOINTBYJOINT);
			
			// Acceleration constant
			std::vector<double> accelerations(current_joint_values.size(), DEFAULT_JOINT_ACCEL);

			p.accelerations = accelerations;  
			
			trajectory_msgs::JointTrajectory t;
			t.joint_names = current_group_joints;
			t.points.push_back(p);
								
			control_msgs::FollowJointTrajectoryGoal goal_;
			goal_.trajectory = t; 

			goal_active = true;
				
			// If there's any other active trajectory, overwrites it
			this->ac_follow_joint_traj->sendGoal(goal_);
			
			
		
		}
		
		this->joint_by_joint_msg.processed = true;
	}else{
		// TIMEOUT WITHOUT RECEIVING COMMANDS
		ROS_INFO("%s::sendJointByJointVelocity: Trajectory ended (JOINTBYJOINT:VELOCITY)", component_name.c_str());
		//switchToState(robotnik_msgs::State::STANDBY_STATE);
		goal_active = false;
	}
	
}


/*!	\fn int RtTrajPlanner::sendJointByJointPosition
 *	\brief Executes the Joint by joiny Type control in Position control
 * 			Uses the class attribute jointbyjoint_msg 
 *  \return 0 

*/
int RtTrajPlanner::sendJointByJointPosition(){
	std::vector<double> current_joint_values, robot_joint_values;
	std::vector<std::string> current_group_joints;
	static double t_planning = 0.5;
	moveit::planning_interface::MoveGroup::Plan my_plan;
	collision_detection::CollisionResult collision_result;
	double dx = 0.0, dy = 0.0, dz = 0.0, dpitch = 0.0, droll = 0.0, dyaw = 0.0;
	bool found_ik = false;
	double dist = 0.0;
	static double dt = (1.0/desired_freq);
	double max_diff = 0.0;
	
	// local copy of class attribute to avoid overwritte 
	JointByJointMsg joint_by_joint_msg = this->joint_by_joint_msg;
	
	if(!joint_by_joint_msg.processed){
	
		// Gets current joint values from move_group interface
		robot_joint_values = move_group_robot->getCurrentJointValues();
		current_joint_values = current_move_group->mg->getCurrentJointValues();
		//current_group_joints = current_move_group->joint_names;
		current_group_joints = current_move_group->joint_names;
		std::vector<double> velocities(current_joint_values.size());
		
		// Updates Kinematic state of the whole robot
		//if(not kinematic_state->getJointStateGroup(s_robot_group)->setVariableValues(robot_joint_values))
		//	ROS_ERROR("%s::sendJointByJointPosition: ERROR setting variable values for current kinematic state(JOINTBYJOINT:POSITION)", component_name.c_str());
		kinematic_state->setJointGroupPositions(joint_robot_model_group, robot_joint_values);
		
		// Copy desired joint values into a vector with the correct order
		for(int32_t i = 0; i < current_joint_values.size(); i++){
			
			for(int32_t j = 0; j < joint_by_joint_msg.msg.joints.size(); j++){
				if(current_group_joints[i].compare(joint_by_joint_msg.msg.joints[j]) == 0){
					velocities[i] = (joint_by_joint_msg.msg.values[j] - current_joint_values[i])/ DEFAULT_T_PLAN;
					current_joint_values[i] = joint_by_joint_msg.msg.values[j];
					ROS_INFO("%s::readyState: Setting joint %s to %.3lf", component_name.c_str(), current_group_joints[i].c_str(), current_joint_values[i]);
					break;
				}
			}					
		}
		
		// Updates kinematic state after setting the values with the previous modifications
		//if(not kinematic_state->getJointStateGroup(current_move_group->name_group)->setVariableValues(current_joint_values))
		//	ROS_ERROR("%s::sendJointByJointPosition: ERROR setting variable values for new kinematic state (JOINTBYJOINT:POSITION)", component_name.c_str());
		kinematic_state->setJointGroupPositions(groups2joint_model_group[current_move_group->name_group], current_joint_values);
		
		/*for(int j = 0; j<current_group_joints.size();j++){
			ROS_INFO("joint %s", current_group_joints[j].c_str());
		}*/
	
		if(checkCollision(&collision_result, kinematic_state)){
			ROS_ERROR("%s::sendJointByJointPosition: Robot in collision (JOINTBYJOINT:POSITION)", component_name.c_str());
		}else{
		// NO-COLLISION
			if(!kinematic_state->satisfiesBounds()){
				ROS_ERROR("%s::sendJointByJointPosition: Robot does not satisfies bounds (JOINTBYJOINT:POSITION)", component_name.c_str());
				kinematic_state->enforceBounds();
				//kinematic_state->getJointStateGroup(current_move_group->name_group)->getVariableValues(current_joint_values);
				current_joint_values.clear();
				const std::vector<int> current_joint_indexes = groups2joint_model_group[current_move_group->name_group]->getVariableIndexList();
				const double * current_joint_positions = kinematic_state->getVariablePositions();
				for(int i = 0; i < current_joint_indexes.size(); i++){
					current_joint_values.push_back(current_joint_positions[current_joint_indexes[i]]);
				}
			}
			// SATIFIES BOUNDS
			// Prepare msg
			trajectory_msgs::JointTrajectoryPoint p;
		  
			p.positions = current_joint_values;

			p.velocities = velocities;
			
			controlMaxVelocities(p.velocities, MAX_JOINT_VEL_JOINTBYJOINT);
			
			// Acceleration constant
			std::vector<double> accelerations(current_joint_values.size(), DEFAULT_JOINT_ACCEL);

			p.accelerations = accelerations;  
			
			trajectory_msgs::JointTrajectory t;
			t.joint_names = current_group_joints;
			t.points.push_back(p);
								
			control_msgs::FollowJointTrajectoryGoal goal_;
			goal_.trajectory = t; 

			goal_active = true;
				
			// If there's any other active trajectory, overwrites it
			this->ac_follow_joint_traj->sendGoal(goal_);
			ROS_INFO("sendJointByJointPosition: Sending goal");
			
		}
		
		this->joint_by_joint_msg.processed = true;
	}
	
	actionlib::SimpleClientGoalState st = this->ac_follow_joint_traj->getState();
	//if(st == actionlib::SimpleClientGoalState::SUCCEEDED or st == actionlib::SimpleClientGoalState::LOST or actionlib::SimpleClientGoalState::PREEMPTED){
	if(st != actionlib::SimpleClientGoalState::PENDING and st != actionlib::SimpleClientGoalState::ACTIVE){
		ROS_INFO("%s::sendJointByJointPosition: Trajectory ended (JOINTBYJOINT:POSITION)", component_name.c_str());
		//switchToState(robotnik_msgs::State::STANDBY_STATE);
		goal_active = false;
	}
	
	return 0;
}


/*!	\fn int RtTrajPlanner::sendTrajectory()
 *	\brief Executes the Trajectory Type control
 * 			Uses the class attribute trajectory_msg 
 *  \return 0 if the traj is finished
 *  \return -1 if there's no goal
 *  \return -2 if the trajectory cannot be executed
 *  \return -3 if the trajectory has been cancelled
 *  \return -4 if the group is not defined
*/
int RtTrajPlanner::sendTrajectory(){
	std::vector<double> current_joint_values, robot_joint_values;
	std::vector<std::string> current_group_joints;
	static double t_planning = 0.5;
	moveit::planning_interface::MoveGroup::Plan my_plan;
	collision_detection::CollisionResult collision_result;
	double dx = 0.0, dy = 0.0, dz = 0.0, dpitch = 0.0, droll = 0.0, dyaw = 0.0;
	bool found_ik = false;
	double dist = 0.0;
	static double dt = (1.0/desired_freq);
	double max_diff = 0.0;
	
	bool error = false;
		
	// RUNNING TRAJ
	if(goal_active){
		
		// Process Points
		// First point
		if(current_trajectory_msg.current_point == 0){
			
			robotnik_trajectory_planner::PointTraj point = current_trajectory_msg.msg.points[current_trajectory_msg.current_point];
			
			if(selectJointStateGroup(point.group)!= 0){
				ROS_ERROR("%s::sendTrajectory: Selected group %s is not defined", component_name.c_str(), point.group.c_str());
				return -4;
			}
			
			// Gets current joint values from move_group interface
			current_joint_values = current_move_group->mg->getCurrentJointValues();
			robot_joint_values = move_group_robot->getCurrentJointValues();
			current_group_joints = current_move_group->mg->getJoints();
			
			// Updates Kinematic state of the whole robot
			//if(not kinematic_state->getJointStateGroup(s_robot_group)->setVariableValues(robot_joint_values))
			//	ROS_ERROR("%s::sendTrajectory: ERROR setting variable values for current kinematic state(TRAJECTORY)", component_name.c_str());
			// kinematic_state->setVariablePositions(robot_joint_values);
			kinematic_state->setJointGroupPositions(joint_robot_model_group, robot_joint_values);
			
			max_diff = 0;
			double diff = 0.0;
			// Copy desired joint values into a vector with the correct order
			for(int32_t i = 0; i < current_joint_values.size(); i++){
				
				for(int32_t j = 0; j < point.joint_names.size(); j++){
					if(current_group_joints[i].compare(point.joint_names[j]) == 0){
						diff = fabs(current_joint_values[i] - point.joint_values[j]);
						if(diff > max_diff)
							max_diff = diff;
						current_joint_values[i] = point.joint_values[j];
						//ROS_INFO("%s::readyState: Setting joint %s to %.3lf", component_name.c_str(), current_group_joints[i].c_str(), current_joint_values[i]);
						break;
					}
				}					
			}
			
			
			
			
			// Updates Kinematic state of the whole robot
			//if(not kinematic_state->getJointStateGroup(current_move_group->name_group)->setVariableValues(current_joint_values))
			//	ROS_ERROR("%s::sendTrajectory: ERROR setting variable values for current kinematic state(TRAJECTORY)", component_name.c_str());
			//kinematic_state->setVariablePositions(current_group_joints, current_joint_values);
			kinematic_state->setJointGroupPositions(groups2joint_model_group[current_move_group->name_group], current_joint_values);
			
			if(checkCollision(&collision_result, kinematic_state)){
				ROS_ERROR("%s::sendTrajectory: Robot in collision (TRAJECTORY)", component_name.c_str());
				error = true;
			}else{
			// NO-COLLISION
				if(!kinematic_state->satisfiesBounds()){
					kinematic_state->enforceBounds();
					ROS_INFO("%s::sendTrajectory: Robot does not satisfies bounds (TRAJECTORY). Bounds enforced", component_name.c_str());
					//error = true;
				}
				// SATIFIES BOUNDS
					
				// Sets the desired target values
				current_move_group->mg->setJointValueTarget(current_joint_values);
				current_move_group->mg->setPlanningTime(DEFAULT_T_PLAN);	// TODO: Specify velocity
				// Gets the plan
				bool success = current_move_group->mg->plan(my_plan);
				if(!success){
					ROS_ERROR("%s::sendTrajectory: Error getting the plan (TRAJECTORY)", component_name.c_str());
					error = true;
				}else{
					// Sending via action client
					control_msgs::FollowJointTrajectoryGoal goal_;
					
					// Control max velocity restrictions for every point
					for(int i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); i++){
						controlMaxVelocities(my_plan.trajectory_.joint_trajectory.points[i].velocities, MAX_JOINT_VEL_TRAJECTORY);
					}
					
					goal_.trajectory = my_plan.trajectory_.joint_trajectory;
					
					goal_active = true;
					//ROS_INFO("%s::readyState: Sending trajectory! (JOINTBYJOINT:POSITION)", component_name.c_str());
					// If there's any other active trajectory, overwrites it
					this->ac_follow_joint_traj->sendGoal(goal_);
					current_trajectory_msg.current_point++;
				}
	
			}
			if(error){
				goal_active = false;
				ROS_INFO("%s::sendTrajectory: Trajectory ended due to an error(TRAJECTORY)", component_name.c_str());
				return -2;
			}		
			
		}else{
		// Rest of points	
			// END CONDITION		
			actionlib::SimpleClientGoalState st = this->ac_follow_joint_traj->getState();
			if(st == actionlib::SimpleClientGoalState::SUCCEEDED or st == actionlib::SimpleClientGoalState::LOST ){
				// Are there more points?
				goal_active = false;
				ROS_INFO("%s::sendTrajectory: Trajectory ended (TRAJECTORY)", component_name.c_str());
				return 0;
			}
			else if(st != actionlib::SimpleClientGoalState::PENDING and st != actionlib::SimpleClientGoalState::ACTIVE){
				// Are there more points?
				goal_active = false;
				ROS_INFO("%s::sendTrajectory: Trajectory cancelled (TRAJECTORY)", component_name.c_str());
				return -3;
			}
		}
		
	
	}else{
	// NEW TRAJ
		if(!trajectory_msg.processed){
			current_trajectory_msg = trajectory_msg;
			trajectory_msg.processed = true;	// Set as processed
			
			goal_active = true;
		}
	}
	
	return -1;
}
 
 
		

/*!	\fn void RtTrajPlanner::readyState2()
 *	\brief Actions performed on ready state
*/
void RtTrajPlanner::readyState2(){
	
	std::vector<double> current_joint_values, arm_joint_values;
	std::vector<std::string> group_variable_joints;
	std::vector<double> joint_values;
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool found_ik = false;
	static double dx = -0.01, dy= -0.01, dz = 0.01, dpitch = 0.01, droll = 0.01;
	static double t_planning = 0.5;
	double dist = 0.0;
	
	this->m_transform << 1.0, 0.0, 0.0, dx,
		0.0, 1.0, 0.0,  dy,
		0.0, 0.0, 1.0,  dz,
		0.0, 0.0, 0.0, 1.0;

	Eigen::Affine3d rotMatPitch( Eigen::AngleAxisd( dpitch, ( Eigen::Vector3d() << 0.0, 1.0, 0.0 ).finished() ) );
	this->m_transform = this->m_transform * rotMatPitch.matrix();
	Eigen::Affine3d rotMatRoll( Eigen::AngleAxisd( droll, ( Eigen::Vector3d() << 1.0, 0.0, 0.0 ).finished() ) );
	this->m_transform = this->m_transform * rotMatRoll.matrix();
	
	//ROS_INFO("Planning time = %.3f", current_move_group->getPlanningTime());
	
	// Gets current joint values from move_group iface
	current_joint_values = move_group_robot->getCurrentJointValues();
	arm_joint_values = current_move_group->mg->getCurrentJointValues();
	group_variable_joints = move_group_robot->getJoints();
	
	//if(not kinematic_state->getJointStateGroup(s_robot_group)->setVariableValues(current_joint_values))
	//	ROS_ERROR("ERROR setting variable values for jointstategroup");
	kinematic_state->setVariablePositions(current_joint_values);
	
	/*
	 * PRINT THE JOINTS BEFORE MOVING
	 * for(std::size_t i = 0; i < group_variable_joints.size(); ++i)
	{
		ROS_INFO("Before moving: Joint %s: %f", group_variable_joints[i].c_str(), current_joint_values[i]);
	}*/
	
	// Gets the transformation of end-effector
	//const Eigen::Affine3d &end_effector_state = kinematic_state->getFrameTransform("wsg50_base_link");
	const Eigen::Affine3d &end_effector_state = kinematic_state->getFrameTransform(current_move_group->mg->getEndEffectorLink().c_str());
	//Print end-effector pose. Remember that this is in the model frame //
	//ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
	//ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
	
	// Applying transformation
	Eigen::Affine3d next_end_effector_state = end_effector_state;
	next_end_effector_state = next_end_effector_state * this->m_transform;
	
	// Calcs Inverse Kinematics
	//found_ik = kinematic_state->getJointStateGroup(current_move_group->name_group)->setFromIK(next_end_effector_state, 5.0, 0.1);
	found_ik = kinematic_state->setFromIK(groups2joint_model_group[current_move_group->name_group], next_end_effector_state, 5.0, 0.1);
	
	if(found_ik){
		//ROS_INFO("found_ik!");
		// Now, let's modify one of the joints, plan to the new joint
		// space goal and visualize the plan.
		//current_joint_values[0] = 2.0;  
		//current_joint_values[1] = 1.0;  
		//
		planning_scene->setCurrentState(*kinematic_state);
		//robot_state::RobotState& current_state = planning_scene->getCurrentStateNonConst();
		//current_state = *kinematic_state;
		collision_detection::CollisionRequest collision_request;
		collision_detection::CollisionResult collision_result;
		collision_request.contacts = true;
		collision_request.max_contacts = 1000;
		collision_request.group_name = current_move_group->name_group;  
		collision_request.distance = true;
		//collision_result.clear();
		//planning_scene->checkCollision(collision_request, collision_result);
		planning_scene->checkSelfCollision(collision_request, collision_result);
		//ROS_INFO("Collision result %5.10f", collision_result.distance);
		
		if(collision_result.collision){ 
			ROS_ERROR("Robot in SELF-COLLISION: distance = %.4f",collision_result.distance);
			dx = -dx;
			dy = -dy;
			dz = -dz;
			collision_detection::CollisionResult::ContactMap::const_iterator it;
			for(it = collision_result.contacts.begin();
				it != collision_result.contacts.end();
				++it)
			{
			  ROS_INFO("Contact between: %s and %s",
					   it->first.first.c_str(),
					   it->first.second.c_str());
			}
		}else{
			// Check if the bounds are satisfied
			/* Check whether any joint is outside its joint limits */
			if(!kinematic_state->satisfiesBounds()){
				dx = -dx;
				dy = -dy;
				dz = -dz;
				dpitch = -dpitch;
				droll = -droll;
				ROS_ERROR("Kinematic state doesn't satisfy bounds!");
				/* Enforce the joint limits for this state and check again*/
				kinematic_state->enforceBounds();
				ROS_INFO_STREAM("Current state after enforcing is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
			}
			
			// Check that next joint values doesn't produce a jump
			//kinematic_state->getJointStateGroup(current_move_group->name_group)->getVariableValues(joint_values);
			joint_values.clear();
			const std::vector<int> current_joint_indexes = groups2joint_model_group[current_move_group->name_group]->getVariableIndexList();
			const double * current_joint_positions = kinematic_state->getVariablePositions();
			for(int i = 0; i < current_joint_indexes.size(); i++){
				joint_values.push_back(current_joint_positions[current_joint_indexes[i]]);
			}
			
			bool bJump = false;
			for(std::size_t i = 0; i < joint_values.size(); ++i)
			{
				dist = fabs ( joint_values[i] - arm_joint_values[i] );
				//ROS_INFO("Diff on joint %d =  %5.2f", (int)i+1, dist);
				if (dist > 0.2) {   // TODO - this limit as parameter, it could be a function of speed
					ROS_INFO("JUMP in joint (%d) -> %5.2f", (int)i+1, dist);
					bJump = true;
				}
			} 
			
			if(!bJump){
					//kinematic_state->getJointStateGroup("right_arm")->getVariableValues(joint_values);
				//current_move_group->setJointValueTarget(joint_values);
				current_move_group->mg->clearPoseTarget();
				current_move_group->mg->setPoseTarget(next_end_effector_state);
				
				//current_move_group->setRandomTarget();
				current_move_group->mg->setPlanningTime(t_planning);
				bool success = current_move_group->mg->plan(my_plan);
				
				//ROS_INFO("Getting plan (joint space goal) %s",success?"OK":"FAILED");
				
				// bool success = current_move_group->asyncMove();
				// bool success = current_move_group->move();
				if(success){
					// Moving using move_group
					//current_move_group->stop();
					//success = current_move_group->asyncExecute(my_plan);
					
					// Sending via action client
					//control_msgs::FollowJointTrajectoryActionGoal goal_;
					control_msgs::FollowJointTrajectoryGoal goal_;
					goal_.trajectory = my_plan.trajectory_.joint_trajectory;
					
					actionlib::SimpleClientGoalState goal_state = this->ac_follow_joint_traj->getState();
					
					
					//ROS_INFO("RtTrajPlanner::readyState: current goal state = %s, msg = %s", goal_state.toString().c_str(), goal_state.getText().c_str());
					//this->ac_follow_joint_traj->sendGoal(&this->goal.trajectory);
					this->ac_follow_joint_traj->sendGoal(goal_);
					
					
					
					// Sleep to give Rviz time to visualize the plan. //
					sleep(t_planning);
					
					
				}else{
					ROS_ERROR("Error getting plan");
					dx = -dx;
					dy = -dy;
					dz = -dz;
				}
			}else{
				dx = -dx;
				dy = -dy;
				dz = -dz;
			}
		}
	
		//ROS_INFO("Is state collidin? %s", planning_scene->isStateColliding(*kinematic_state)? "true": "false");
			
	}else{
		ROS_INFO("Did not find IK solution");
		dx = -dx;
		dy = -dy;
		dz = -dz;
	}
	
	
		
}

/*!	\fn void RtTrajPlanner::EmergencyState()
 *	\brief Actions performed on emergency state
*/
void RtTrajPlanner::emergencyState(){

}

/*!	\fn void RtTrajPlanner::FailureState()
 *	\brief Actions performed on failure state
*/
void RtTrajPlanner::failureState(){

}

/*!	\fn void RtTrajPlanner::AllState()
 *	\brief Actions performed on all states
*/
void RtTrajPlanner::allState(){
	rosPublish();
}

/*!	\fn double RtTrajPlanner::getUpdateRate()
 * 	\brief Gets current update rate of the thread
 * 	\return real frequency of the thread
*/
double RtTrajPlanner::getUpdateRate(){
	return desired_freq;
}

/*!	\fn int RtTrajPlanner::getState()
 * 	\brief returns the state of the component
*/
int RtTrajPlanner::getState(){
	return state;
}

/*!	\fn char *RtTrajPlanner::getStateString()
 *	\brief Gets the state of the component as string
*/
char *RtTrajPlanner::getStateString(){
	return getStateString(state);
}

/*!	\fn char *RtTrajPlanner::getStateString(int state)
 *	\brief Gets the state as a string
*/
char *RtTrajPlanner::getStateString(int state){
	switch(state){
		case robotnik_msgs::State::INIT_STATE:
			return (char *)"INIT";
		break;
		case robotnik_msgs::State::STANDBY_STATE:
			return (char *)"STANDBY";
		break;
		case robotnik_msgs::State::READY_STATE:
			return (char *)"READY";
		break;
		case robotnik_msgs::State::EMERGENCY_STATE:
			return (char *)"EMERGENCY";
		break;
		case robotnik_msgs::State::FAILURE_STATE:
			return (char *)"FAILURE";
		break;
		case robotnik_msgs::State::SHUTDOWN_STATE:
			return (char *)"SHUTDOWN";
		break;
		default:
			return (char *)"UNKNOWN";
		break;
	}
}


/*!	\fn char *RtTrajPlanner::getSubstateString(int substate)
 *	\brief Gets the substate as a string
*/
char *RtTrajPlanner::getSubstateString(int substate){
	
	switch(substate){
		case INIT_SETUP:
			return (char *)"INIT_SETUP";
		break;
		case INIT_INIT_ARMS:
			return (char *)"INIT_ARMS";
		break;
		case INIT_INIT_WAIST:
			return (char *)"INIT_WAIST";
		break;
		case INIT_MOVE_ARMS:
			return (char *)"INIT_MOVE_ARMS";
		break;
		case INIT_INIT_LEFT_GRIPPER:
			return (char *)"INIT_LEFT_GRIPPER";
		break;
		case INIT_INIT_RIGHT_GRIPPER:
			return (char *)"INIT_RIGHT_GRIPPER";
		break;
		case INIT_INIT_HEAD:
			return (char *)"INIT_HEAD";
		break;
		default:
			return (char *)"UNKNOWN";
		break;
	}
}


/*!	\fn string RtTrajPlanner::controlModeToString(int mode)
 *	\brief Gets current control mode as string
*/
string RtTrajPlanner::controlModeToString(int mode){
	switch(mode){
		case POSITION:
			return "POSITION";
		break;
		case VELOCITY:
			return "VELOCITY";
		break;
		default:
			return "UNKNOWN";
		break;
	}
}


/*!	\fn string RtTrajPlanner::controlTypeToString(int type)
 *	\brief Gets current control type as string
*/
string RtTrajPlanner::controlTypeToString(int type){
	switch(type){
		case CARTESIAN_EULER:
			return "CARTESIAN_EULER";
		break;
		case JOINTBYJOINT:
			return "JOINT_BY_JOINT";
		break;
		case TRAJECTORY:
			return "TRAJECTORY";
		break;
		default:
			return "UNKNOWN";
		break;
	}
}


/*!	\fn void RtTrajPlanner::switchToState(int new_state)
 * 	function that switches the state of the component into the desired state
 * 	\param new_state as an integer, the new state of the component
*/
void RtTrajPlanner::switchToState(int new_state){
	
	if(new_state == state)
		return;

	// saves the previous state
	previous_state = state;
	ROS_INFO("%s::SwitchToState: %s -> %s", component_name.c_str(), getStateString(state), getStateString(new_state));	
	state = new_state;
 
}


/*!	\fn void RtTrajPlanner::switchToSubstate(int new_substate)
 * 	
 * 	\param new_substate as an integer, the new substate of the component
*/
void RtTrajPlanner::switchToSubstate(int new_substate){
	
	switch(state){
		case robotnik_msgs::State::INIT_STATE:
			if(new_substate == substate_init)
				return;
			//ROS_INFO("%s::switchToSubstate: %s -> %s", component_name.c_str(), getSubstateString(substate_init), getSubstateString(new_substate));	
			substate_init = new_substate;
			st.substate = getSubstateString(substate_init);
		break;
		default:
			st.substate = string("");
		break;
	}
}

/*!	\fn void RtTrajPlanner::rosSetup()
 * 	\brief Setups all ROS' stuff
*/
int RtTrajPlanner::rosSetup(){
	
	// Checks if has been initialized
	if(ros_initialized){
		ROS_INFO("%s::rosSetup: Already initialized",component_name.c_str());
		
		return INITIALIZED;
	}
	// Read params
	pnh.param<std::string>("follow_joint_traj_name", follow_joint_traj_name, "/rt_traj_exe/follow_joint_trajectory/");
	pnh.param<std::string>("robot_group", s_robot_group, "torso");
	pnh.param<std::string>("control_state_topic", control_state_topic_name, "/rt_traj_exe/state");
	pnh.param<std::string>("control_state_actions_service", control_state_actions_service_name, "/rt_traj_exe/actions");
	pnh.param<std::string>("load_trajectory_service", load_trajectory_service_name, "/rt_traj_manager/load_state");
	//pnh.param<std::string>("init_position_id", init_position_id, "init_position");
	
	XmlRpc::XmlRpcValue list;
	pnh.getParam("move_groups", list);
	
	// Checks that the read param type is correct
	if(list.getType() != XmlRpc::XmlRpcValue::TypeArray){
		ROS_ERROR("%s::rosSetup: Wrong read type (%d) for move_groups param", component_name.c_str(), list.getType());
		return ERROR;
	}
	// Saves the array into class vector
	for(int32_t i = 0; i < list.size(); ++i) {
		v_move_groups_name.push_back(static_cast<std::string>(list[i]));
		ROS_INFO("%s::rosSetup: Move group %d -> %s",component_name.c_str(), i, v_move_groups_name[i].c_str());
		// Creating one MoveGroup interface for each group name
		MoveGroupStruct mgs;
		mgs.name_group = v_move_groups_name[i];
		
		mgs.mg = new moveit::planning_interface::MoveGroup(v_move_groups_name[i]);
		mgs.selected_tcp = mgs.mg->getEndEffectorLink();
		// Saves the list of joint of this groups to avoid calling the function iteratively
		mgs.joint_names = mgs.mg->getJoints(); 
		for(int j = 0; j<mgs.joint_names.size();j++){
			ROS_INFO("joint %s", mgs.joint_names[j].c_str());
		}
		v_move_groups.push_back(mgs); 
	}
	// Creating one MoveGroup interface the whole model
	move_group_robot = new moveit::planning_interface::MoveGroup(s_robot_group);
	vector<string> robot_joints = move_group_robot->getJoints();
	
	ROS_INFO("%s::rosSetup: Follow joint trajectory action on %s",component_name.c_str(), follow_joint_traj_name.c_str());
	ROS_INFO("%s::rosSetup: Robot group %s",component_name.c_str(), s_robot_group.c_str());
	for(int j = 0; j<robot_joints.size();j++){
		ROS_INFO("joint %s", robot_joints[j].c_str());
	}
	
	// Sets the first group name by default
	selectJointStateGroup(v_move_groups_name[0]);
	
	//
	// Publishers
	//
	display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	state_publisher = pnh.advertise<robotnik_trajectory_planner::State>("state", 1, true);
	
	//
	// Subscribers
	//
	jointbyjoint_subscriber = pnh.subscribe("commands/joint_by_joint", 1, &RtTrajPlanner::jointByJointCallback, this);
	cartesianeuler_subscriber = pnh.subscribe("commands/cartesian_euler", 1, &RtTrajPlanner::cartesianEulerCallback, this);
	trajectory_subscriber = pnh.subscribe("commands/trajectory", 1, &RtTrajPlanner::trajectoryCallback, this);
	torso_control_state_subscriber = nh.subscribe(control_state_topic_name, 1, &RtTrajPlanner::controlStateCallback, this);
	
	//
	// Services
	//
	select_group_service = pnh.advertiseService("select_group",  &RtTrajPlanner::selectGroupSrv, this);
	set_control_mode_service = pnh.advertiseService("set_control_mode",  &RtTrajPlanner::setControlModeSrv, this);
	get_joints_group_service = pnh.advertiseService("get_joints_group",  &RtTrajPlanner::getJointsGroupSrv, this);
	get_groups_service = pnh.advertiseService("get_groups",  &RtTrajPlanner::getGroupsSrv, this);
	init_service = pnh.advertiseService("init",  &RtTrajPlanner::initSrv, this);
	// Service clients
	action_control_client = pnh.serviceClient<robotnik_trajectory_control::TrajExecActions>(control_state_actions_service_name);
	load_trajectory_client = pnh.serviceClient<robotnik_trajectory_manager::LoadState>(load_trajectory_service_name);
	
	
	// Load the robot model 
	robot_model_loader_ptr = new robot_model_loader::RobotModelLoader("robot_description");
	
	ac_follow_joint_traj = new ActionClient(follow_joint_traj_name, true);
	
	// Get a shared pointer to the model
	kinematic_model = robot_model_loader_ptr->getModel();
	// WORKING WITH THE KINEMATIC STATE 
	// Create a kinematic state - this represents the configuration for the robot represented by kinematic_model
	kinematic_state = robot_state::RobotStatePtr( new robot_state::RobotState( kinematic_model ) );
	// Set all joints in this state to their default values 
	kinematic_state->setToDefaultValues();
	// Auxiliar state used for some task
	aux_kinematic_state = robot_state::RobotStatePtr( new robot_state::RobotState( kinematic_model ) );
	// Set all joints in this state to their default values 
	aux_kinematic_state->setToDefaultValues();
	
	// Creates and map a JointModelGroup per group
	for(int32_t i = 0; i < v_move_groups_name.size(); ++i) {
		//robot_model::JointModelGroup * joint_model_group;
		//joint_model_group = 
		groups2joint_model_group[v_move_groups_name[i]] = kinematic_model->getJointModelGroup(v_move_groups_name[i]);
	}
	// Model for the whole robot
	joint_robot_model_group = kinematic_model->getJointModelGroup(s_robot_group);
	
	
	
	// Creating planning scene
	planning_scene = new planning_scene::PlanningScene(kinematic_model);
	
	 // Initial command
	m_transform = Eigen::Matrix4d::Identity();
	
	// ^^^^^^^^^^^^^^^^^^^^^^^^^
	// Getting Basic Information
	//
	// Get and print the name of the coordinate frame in which the transforms for this model are computed
	ROS_INFO("%s::rosSetup: Model frame: %s", component_name.c_str(), kinematic_model->getModelFrame().c_str());
	
	ros_initialized = true;

	return OK;
	
	/* Example 
	private_node_handle_.param<std::string>("port", port_, DEFAULT_DSPIC_PORT);
	private_node_handle_.param<std::string>("odom_frame_id", odom_frame_id_, "/odom_diff");
	private_node_handle_.param<std::string>("base_frame_id", base_frame_id_, "/base_link");
	private_node_handle_.param("publish_tf", publish_tf_, false);
	private_node_handle_.param("desired_freq", desired_freq_, desired_freq_);
	
	status_pub_ = private_node_handle_.advertise<agvs_controller::DspicStatus>("status", 1);
	odom_pub_ = private_node_handle_.advertise<nav_msgs::Odometry>(odom_frame_id_, 1);
	calibrate_srv_ = private_node_handle_.advertiseService("calibrate",  &dspic_controller_node::CalibrateSrv, this);
	set_odom_service_ = private_node_handle_.advertiseService("set_odometry", &dspic_controller_node::SetOdometry, this);*/
}

/*!	\fn int RtTrajPlanner::rosShutdown()
 * 	\brief Closes all ros stuff
*/
int RtTrajPlanner::rosShutdown(){
	if(running){
		ROS_INFO("%s::rosShutdown: Impossible while thread running, first must be stopped",component_name.c_str());
		return THREAD_RUNNING;
	}
	if(!ros_initialized){
		ROS_INFO("%s::rosShutdown: Impossible because of it's not initialized", component_name.c_str());
		return NOT_INITIALIZED;
	}
	
	ros_initialized = false;

	return OK;
}

/*!	\fn void RtTrajPlanner::rosPublish()
 * 	\brief Reads data a publish several info into different topics
*/
void RtTrajPlanner::rosPublish(){
	
	st.state.state = this->state;
	st.state.desired_freq = this->desired_freq;
	st.state.real_freq = this->real_freq;
	st.state.state_description = getStateString(this->state);
	st.control_mode = controlModeToString(this->control_mode);
	st.control_type = controlTypeToString(this->control_type);
	st.tcp = this->current_move_group->selected_tcp;
	
	
	if(this->ac_follow_joint_traj->isServerConnected()){
		if(goal_active){
			actionlib::SimpleClientGoalState goal_state = this->ac_follow_joint_traj->getState();
			st.goal_state = goal_state.toString();
		}else{
			st.goal_state = "IDLE";
		}
	
	}else
		st.goal_state = "NOT_CONNECTED";

	// Publish component state
	state_publisher.publish(st);
}


/*!	\fn int RtTrajPlanner::selectJointStateGroup(string group)
 * 	\brief Selects the current JointStateGroup
 *  \param group as string, name of the group
 *  \return 0 if OK, -1 if ERROR
 * 
*/
int RtTrajPlanner::selectJointStateGroup(string group){
	for (int32_t i = 0; i < v_move_groups.size(); ++i){
		if(v_move_groups[i].name_group.compare(group) == 0){
			// this->current_move_group = v_move_groups[i].mg; //OLD
			this->current_move_group = &v_move_groups[i];
			st.current_group = group;
			//v_move_groups[i].selected_tcp = current_move_group->mg->getEndEffectorLink();
			ROS_INFO("%s::selectJointStateGroup: Setting current group to %s, Reference frame: %s, End effector link: %s", component_name.c_str(),
			 group.c_str(), current_move_group->mg->getPlanningFrame().c_str(), current_move_group->mg->getEndEffectorLink().c_str());
			return 0;
		}	
	}
	
	ROS_ERROR("%s::selectJointStateGroup: group to %s not found", component_name.c_str(), group.c_str());
	return -1;
}


/*!	\fn bool RtTrajPlanner::selectGroupSrv(robotnik_trajectory_planner::SelectGroup::Request &req, robotnik_trajectory_planner::SelectGroup::Response &res )
 * 	\brief ROS Service Handler to set current JointStateGroup
 *  \return true if OK, false if ERROR
 * 
*/
bool RtTrajPlanner::selectGroupSrv(robotnik_trajectory_planner::SelectGroup::Request &req, robotnik_trajectory_planner::SelectGroup::Response &res ){
	if(selectJointStateGroup(req.group_name) == 0){
		res.ret = true;
	}else{
		res.ret = false;
	}
	return true;
}


/*!	\fn bool RtTrajPlanner::setControlModeSrv(robotnik_trajectory_planner::SetControlMode::Request &req, robotnik_trajectory_planner::SetControlMode::Response &res )
 * 	\brief ROS Service Handler to set the control mode
 *  \return true if OK, false if ERROR
 * 
*/
bool RtTrajPlanner::setControlModeSrv(robotnik_trajectory_planner::SetControlMode::Request &req, robotnik_trajectory_planner::SetControlMode::Response &res ){
	if(state == robotnik_msgs::State::READY_STATE){
		res.ret = false;
		ROS_ERROR("%s::setControlModeSrv: Control mode cannot be set while in operation", component_name.c_str());
	}else{
		if(!req.mode.compare("POSITION")){
			control_mode = POSITION;
			res.ret = true;
		}else if(!req.mode.compare("VELOCITY")){
			control_mode = VELOCITY;
			res.ret = true;
		}else
			res.ret = false;
	}
	
	return true;
}

/*!	\fn bool RtTrajPlanner::getJointsGroupSrv(robotnik_trajectory_planner::GetJointsGroup::Request &req, robotnik_trajectory_planner::GetJointsGroup::Response &res )
 * 	\brief ROS Service Handler to set the control mode
 *  \return true if OK, false if ERROR
 * 
*/
bool RtTrajPlanner::getJointsGroupSrv(robotnik_trajectory_planner::GetJointsGroup::Request &req, robotnik_trajectory_planner::GetJointsGroup::Response &res ){
	
	
	for(int i = 0; i < v_move_groups.size(); i++){
		if(!v_move_groups[i].name_group.compare(req.group_name)){
			res.ret = v_move_groups[i].joint_names;
			return true;
		}
	}
	
	ROS_ERROR("%s::getJointsGroupSrv: the group %s is no available", component_name.c_str(), req.group_name.c_str());
	
	return false;
}

/*!	\fn bool RtTrajPlanner::getGroupsSrv(robotnik_trajectory_planner::GetGroups::Request &req, robotnik_trajectory_planner::GetGroups::Response &res )
 *  * 	\brief ROS Service Handler to get all the available groups
 *  \return true if OK, false if ERROR
 * 
*/
bool RtTrajPlanner::getGroupsSrv(robotnik_trajectory_planner::GetGroups::Request &req, robotnik_trajectory_planner::GetGroups::Response &res ){
	res.ret = v_move_groups_name;
	return true;
}


/*!	\fn bool RtTrajPlanner::initSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res )
 *  * 	\brief ROS Service Handler to start the initialization
 *  
 * 
*/
bool RtTrajPlanner::initSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res ){
	initialization_request = true;
	
	return true;
}


/*!	\fn  void RtTrajPlanner::jointByJointCallback(const robotnik_trajectory_planner::JointByJointConstPtr& j)
 * 	\brief callback of jointByJoint msgs 
 */
void RtTrajPlanner::jointByJointCallback(const robotnik_trajectory_planner::JointByJointConstPtr& j)
{
	/* MSG
		# Joint names to move
		float32[]  joints
		# Joint values 
		float32[]  values
	 */
	// Constraints:
	// Joint names has to be part of current group
	// There has to be the same number of names and values
	
	int j_size = j->joints.size(), v_size = j->values.size();
	
	if(j_size > 0 and v_size == j_size){
		//ROS_INFO("Received point: len joints = %d, len values = %d, t_plan = %f", j_size, v_size, j->time_plan);
		joint_by_joint_msg.msg = *j;
		if(joint_by_joint_msg.msg.time_plan <= 0)
			joint_by_joint_msg.msg.time_plan = DEFAULT_T_PLAN;
			
		joint_by_joint_msg.t = ros::Time::now();
		joint_by_joint_msg.processed = false;
	}else
		ROS_ERROR("%s::jointByJointCallback: Error format in message", component_name.c_str());
	//ROS_INFO("Received point");
} 


/*!	\fn  void RtTrajPlanner::cartesianEulerCallback(const robotnik_trajectory_planner::CartesianEulerConstPtr& j)
 * 	\brief ROS callback handler when receiving CartesianEuler msgs
 */
void RtTrajPlanner::cartesianEulerCallback(const robotnik_trajectory_planner::CartesianEulerConstPtr& c){
	cartesian_euler_msg.msg = *c;
	// Filters velocities
	if(cartesian_euler_msg.msg.x > MAX_LINEAR_VEL)
		cartesian_euler_msg.msg.x = MAX_LINEAR_VEL;
	else if(cartesian_euler_msg.msg.x < -MAX_LINEAR_VEL)
		cartesian_euler_msg.msg.x = -MAX_LINEAR_VEL;
	
	if(cartesian_euler_msg.msg.y > MAX_LINEAR_VEL)
		cartesian_euler_msg.msg.y = MAX_LINEAR_VEL;
	else if(cartesian_euler_msg.msg.y < -MAX_LINEAR_VEL)
		cartesian_euler_msg.msg.y = -MAX_LINEAR_VEL;
	
	if(cartesian_euler_msg.msg.z > MAX_LINEAR_VEL)
		cartesian_euler_msg.msg.z = MAX_LINEAR_VEL;
	else if(cartesian_euler_msg.msg.z < -MAX_LINEAR_VEL)
		cartesian_euler_msg.msg.z = -MAX_LINEAR_VEL;
	
	cartesian_euler_msg.t = ros::Time::now();
	cartesian_euler_msg.processed = false;
	
}


/*!	\fn  void RtTrajPlanner::trajectoryCallback(const robotnik_trajectory_planner::TrajectoryConstPtr& t)
 * 	\brief ROS callback handler when receiving Trajectory msgs
 */
void RtTrajPlanner::trajectoryCallback(const robotnik_trajectory_planner::TrajectoryConstPtr& t){
	
	// Constraints:
	// Joint names has to be part of current group
	// There has to be the same number of names and values
	int p_size = t->points.size();
	int j_size = 0, n_size = 0;
	
	if(p_size <= 0){
		ROS_ERROR("%s::trajectoryCallback: Received trajectory with no points",component_name.c_str());
		return;
	}
	
	for(int i = 0; i < p_size; i++){
		if(t->points[i].joint_names.size() != t->points[i].joint_values.size()){
			ROS_ERROR("%s::trajectoryCallback: Point %s has an incorrect size in the params",component_name.c_str(), t->points[i].id.c_str());
			return;
		}
	}
	
	trajectory_msg.msg = *t;
	
	
	trajectory_msg.t = ros::Time::now();
	trajectory_msg.processed = false;
	trajectory_msg.current_point = 0;
}


/*!	\fn  void RtTrajPlanner::trajectoryCallback(const robotnik_trajectory_planner::TrajectoryConstPtr& t)
 * 	\brief ROS callback handler when receiving Trajectory msgs
 */
void RtTrajPlanner::controlStateCallback(const robotnik_trajectory_control::StateConstPtr& t){
	/*
		robotnik_msgs/State state
		# Current substate
		string substate
		# Subcomponents state
		SubcomponentState[] components
	 * */
	t_received_control_state = ros::Time::now();
	this->robotnik_control_state = *t;
	//ROS_INFO("RtTrajPlanner::controlStateCallback");
	
	
}


/*!	\fn  bool RtTrajPlanner::checkCollision(collision_detection::CollisionResult *res = NULL, robot_state::RobotStatePtr k_state)
 * 	\brief Checks for collision
 */
bool RtTrajPlanner::checkCollision(collision_detection::CollisionResult *res, robot_state::RobotStatePtr k_state){
	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
	
	planning_scene->setCurrentState(*k_state);
	collision_request.contacts = true;
	collision_request.max_contacts = 1000;
	collision_request.group_name = current_move_group->name_group;  
	collision_request.distance = true;
	
	planning_scene->checkCollision(collision_request, collision_result);
	
	if(collision_result.collision){
		collision_detection::CollisionResult::ContactMap::const_iterator it;
		for(it = collision_result.contacts.begin();
			it != collision_result.contacts.end();
			++it)
		{
		  ROS_INFO("Contact between: %s and %s",
				   it->first.first.c_str(),
				   it->first.second.c_str());
		}
	}
	*res = collision_result;
	
	return collision_result.collision;
}
	
 
/*!	\fn  int RtTrajPlanner::sendActionToControl(int action)
 * 	\brief Sends an action to the robotnik_trajectory_control component
 */
int RtTrajPlanner::sendActionToControl(int action){
	robotnik_trajectory_control::TrajExecActions msg;
	
	msg.request.action = action;
	if (action_control_client.call(msg)){
		
		return 0;
	}else{
		ROS_ERROR("%s::sendActionToControl: error calling the service %s",component_name.c_str(), control_state_actions_service_name.c_str());
		return -1;
	}
}
		
/*!	\fn  int RtTrajPlanner::loadPositionFromManager(string id)
 * 	\brief Sends an id to the trajectory manager to load a set of joints and values
 */
int RtTrajPlanner::loadPositionFromManager(string id){
	robotnik_trajectory_manager::LoadState msg;
	
	msg.request.id = id;
	if(ros::service::waitForService(load_trajectory_service_name, ros::Duration(1))){
		if (load_trajectory_client.call(msg)){
			//ROS_INFO("%s::loadPositionFromManager: service %s with param %s",component_name.c_str(), load_trajectory_service_name.c_str(), id.c_str());
			
			return 0;
		}else{
			ROS_ERROR("%s::loadPositionFromManager: error calling the service %s with param %s",component_name.c_str(), load_trajectory_service_name.c_str(), id.c_str());
			return -1;
		}
	}else{
		ROS_ERROR("%s::loadPositionFromManager: service %s not available",component_name.c_str(), load_trajectory_service_name.c_str());
		return -1;
	}
}
		
		
//////////		
// MAIN //
//////////
int main(int argc, char** argv)
{
    ros::init(argc, argv, "rt_traj_planner");
	
	ros::NodeHandle n;		
  	RtTrajPlanner planner(20.0, n);
	
	planner.start();

	return (0);
}

