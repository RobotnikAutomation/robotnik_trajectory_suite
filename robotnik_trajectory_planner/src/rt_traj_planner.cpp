/*! \class RtTrajPlanner
 *  \file RtTrajPlanner.cpp
 *	\author Robotnik Automation S.L.L
 *	\version 1.0.0
 *	\date 2015
 *  \brief Class to plan and control trajectories the RB1 robot
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
		switchToState(robotnik_msgs::State::READY_STATE);
	}
	
	// Cartesian Euler?
	else if(!cartesian_euler_msg.processed  and (t_now - cartesian_euler_msg.t).toSec() <= WATCHDOG_COMMAND ){
		ROS_INFO("%s::standbyState: Received new CartesianEuler message", component_name.c_str());
		control_type = CARTESIAN_EULER;
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
		sendCartesianEulerPosition();
		
		
		if(not goal_active)
			switchToState(robotnik_msgs::State::STANDBY_STATE);
	}
	
	//
	//	JOINTBYJOINT
	//
	else if(control_type == JOINTBYJOINT){
		//
		// CONTROL POSITION
		sendJointByJointPosition();	
		
		if(not goal_active)
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
	int current_collision_level = 0;
	static double dx_inc = 0.01, dy_inc = 0.01, dz_inc = 0.01;
	int trials = 0, max_trials = 2;
	//ROS_INFO("TDIFF = %.3f",(ros::Time::now() - cartesian_euler_msg.t).toSec());
		
	// local copy of class attribute to avoid overwritte 
	CartesianEulerMsg cartesian_euler_msg = this->cartesian_euler_msg;
	
	if(!cartesian_euler_msg.processed){
		//ROS_INFO("sendCartesianEulerPosition: 1");
		dx = cartesian_euler_msg.msg.x;
		dy = cartesian_euler_msg.msg.y;
		dz = cartesian_euler_msg.msg.z;
		dpitch = cartesian_euler_msg.msg.pitch;
		droll = cartesian_euler_msg.msg.roll;
		dyaw = cartesian_euler_msg.msg.yaw;
		//ROS_INFO("sendCartesianEulerPosition: x = %lf, y = %lf, z= %lf, pitch = %lf, roll = %lf, yaw = %lf", dx, dy, dz, dpitch, droll, dyaw);
		
		if( dx != 0.0 or dy != 0.0 or dz != 0.0 or dpitch != 0.0 or droll != 0.0 or dyaw != 0.0){
			//ROS_INFO("sendCartesianEulerPosition: 2");
			
			while(!found_ik && trials < max_trials){
				
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
				kinematic_state->setJointGroupPositions(joint_robot_model_group, robot_joint_values);
				
				const Eigen::Affine3d &end_effector_state = kinematic_state->getFrameTransform(current_move_group->mg->getEndEffectorLink().c_str());	
				
				// Applying transformation
				Eigen::Affine3d next_end_effector_state = end_effector_state;
				next_end_effector_state = next_end_effector_state * this->m_transform;
				
				//ROS_INFO("current joint values of group %s", current_move_group->name_group.c_str());
				/*for(std::size_t i = 0; i < current_joint_values.size(); ++i){
					ROS_INFO("%lf", current_joint_values[i] );
				}*/
				// Calcs Inverse Kinematics
				found_ik = kinematic_state->setFromIK(groups2joint_model_group[current_move_group->name_group], next_end_effector_state, 5, 0.5);
				
				//ROS_INFO("rand = %d", rand());
				
				trials++;
			}
			if(found_ik){
				//ROS_INFO("sendCartesianEulerPosition: 3 (IK)");
				if(checkCollision(&collision_result, kinematic_state, &current_collision_level)){
					ROS_ERROR("%s::readyState: Robot in collision (CARTESIAN-EULER:POSITION)", component_name.c_str());
				}else{
					ROS_INFO("sendCartesianEulerPosition: 4 (NO COLLISION)");
					// NO-COLLISION
					if(!kinematic_state->satisfiesBounds()){
						ROS_ERROR("%s::readyState: Robot does not satisfies bounds (JOINTBYJOINT:POSITION)", component_name.c_str());
						kinematic_state->enforceBounds();
					}
					// SATIFIES BOUNDS
					std::vector<double> transformed_joint_values;

					// Check that next joint values doesn't produce a jump
					//kinematic_state->getJointStateGroup(current_move_group->name_group)->getVariableValues(transformed_joint_values);
					//kinematic_state->setVariablePositions(transformed_joint_values);
					const std::vector<int> current_group_joint_indexes = groups2joint_model_group[current_move_group->name_group]->getVariableIndexList();
					
					
					const double * current_joint_positions = kinematic_state->getVariablePositions();
					std::vector< std::string > variable_names = kinematic_state->getVariableNames();
					int variables = (int)kinematic_state->getVariableCount();
					std::map< std::string, double > target;
					
					// Get the valid joints
					for(int j = 0; j < (int)current_group_joints.size(); j++){
						for(int i = 0; i < variables;i++){
							if(current_group_joints[j] == variable_names[i]){
								target[variable_names[i]] = current_joint_positions[i];
								ROS_INFO("%s = %lf", variable_names[i].c_str() , target[variable_names[i]]);
								break;
							}
						}
					}				
					//ROS_INFO("%d joints, names = %d", (int)current_group_joint_indexes.size(), (int)current_group_joints.size());
					
					for(int i = 0; i < current_group_joint_indexes.size(); i++){
						transformed_joint_values.push_back(current_joint_positions[current_group_joint_indexes[i]]);
					}
					
					//ROS_INFO("Planning frame = %s", current_move_group->mg->getPlanningFrame().c_str());
					current_move_group->mg->setPoseReferenceFrame("/tip_link");
					//ROS_INFO("Reference frame = %s", current_move_group->mg->getPoseReferenceFrame().c_str());
					current_move_group->mg->setStartStateToCurrentState();
					//current_move_group->mg->clearPoseTarget();
					// TARGETS
					//current_move_group->mg->setPoseTarget(next_end_effector_state);
					//current_move_group->mg->setRandomTarget();
					//current_move_group->mg->setPositionTarget(0, 0, 0.2);
					
					current_move_group->mg->setJointValueTarget(target);
					
					current_move_group->mg->setPlanningTime(DEFAULT_T_PLAN);	// TODO: Specify velocity
					current_move_group->mg->setPlannerId(planner_id_);	
					// Gets the plan
					bool success = current_move_group->mg->plan(my_plan);
					
					if(!success){
						ROS_ERROR("%s::readyState: Error getting the plan (CARTESIAN:POSITION)", component_name.c_str());
					}else{
						// Sending via action client
						control_msgs::FollowJointTrajectoryGoal goal_;
						
						goal_.trajectory = my_plan.trajectory_.joint_trajectory;
						
						goal_active = true;
						ROS_INFO("%s::readyState: Sending trajectory! (CARTESIAN-EULER:POSITION)", component_name.c_str());
						// If there's any other active trajectory, overwrites it
						this->ac_follow_joint_traj->sendGoal(goal_);
					}
				}
						
			}else{
				ROS_INFO("%s::readyState: IK NOT FOUND (CARTESIAN-EULER:POSITION)", component_name.c_str());
				ROS_ERROR("%s::readyState: IK NOT FOUND (CARTESIAN-EULER:POSITION)", component_name.c_str());
				// TODO: Show a kind of error msg in the state
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
	int current_collision_level = 0;
	
	// local copy of class attribute to avoid overwritte 
	JointByJointMsg joint_by_joint_msg = this->joint_by_joint_msg;
	
	if(!joint_by_joint_msg.processed){
	
		// Gets current joint values from move_group interface
		robot_joint_values = move_group_robot->getCurrentJointValues();
		current_joint_values = current_move_group->mg->getCurrentJointValues();
		current_group_joints = current_move_group->joint_names;
		
		std::vector<double> velocities(current_joint_values.size());
		
		// Updates Kinematic state of the whole robot
		kinematic_state->setJointGroupPositions(joint_robot_model_group, robot_joint_values);
		
		
		const double * current_joint_positions = kinematic_state->getVariablePositions();
		std::vector< std::string > variable_names = kinematic_state->getVariableNames();
		int variables = (int)kinematic_state->getVariableCount();
		std::map< std::string, double > target;
		
		// Get the valid joints
		for(int j = 0; j < (int)current_group_joints.size(); j++){
			for(int i = 0; i < variables;i++){
				if(current_group_joints[j] == variable_names[i]){
					target[variable_names[i]] = current_joint_positions[i];
					//ROS_INFO("%s = %lf", variable_names[i].c_str() , target[variable_names[i]]);
					break;
				}
			}
		}				
		
		for(int32_t j = 0; j < joint_by_joint_msg.msg.joints.size(); j++){
			target[joint_by_joint_msg.msg.joints[j]] = joint_by_joint_msg.msg.values[j];
			//ROS_INFO("%s::readyState: Setting joint %s to %.3lf", component_name.c_str(), joint_by_joint_msg.msg.joints[j].c_str(), target[joint_by_joint_msg.msg.joints[j]]);
				
		}
			
		current_move_group->mg->setStartStateToCurrentState();	
		current_move_group->mg->setJointValueTarget(target);
		robot_state::RobotState k_state = current_move_group->mg->getJointValueTarget();
		
		if(checkCollision(&collision_result, k_state, &current_collision_level)){
			ROS_ERROR("%s::sendJointByJointPosition: Robot in collision (JOINTBYJOINT:POSITION)", component_name.c_str());
		}else{
			// NO-COLLISION
			current_move_group->mg->setPlanningTime(DEFAULT_T_PLAN);	// TODO: Specify velocity
			current_move_group->mg->setPlannerId(planner_id_);	
			// Gets the plan
			bool success = current_move_group->mg->plan(my_plan);
			
			if(!success){
				ROS_ERROR("%s::readyState: Error getting the plan (JOINTBYJOINT:POSITION)", component_name.c_str());
			}else{
				// Sending via action client
				control_msgs::FollowJointTrajectoryGoal goal_;
				
				goal_.trajectory = my_plan.trajectory_.joint_trajectory;
				
				goal_active = true;
				ROS_INFO("%s::readyState: Sending trajectory! (JOINTBYJOINT-EULER:POSITION)", component_name.c_str());
				// If there's any other active trajectory, overwrites it
				this->ac_follow_joint_traj->sendGoal(goal_);
			}
			
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
	pnh.param<std::string>("planner_id", planner_id_, DEFAULT_PLANNER_ID);
	pnh.param<double>("min_joint_position_inc", min_joint_position_inc_, MIN_JOINT_POSITION_INC);
	pnh.param<double>("collision_distance_1", collision_distance_1_, DEFAULT_COLLISION_DISTANCE_1);
	pnh.param<double>("collision_distance_2", collision_distance_2_, DEFAULT_COLLISION_DISTANCE_2);
	pnh.param<double>("collision_distance_3", collision_distance_3_, DEFAULT_COLLISION_DISTANCE_3);
	// Positive values required
	collision_distance_1_ = fabs(collision_distance_1_);
	collision_distance_2_ = fabs(collision_distance_2_);
	collision_distance_3_ = fabs(collision_distance_3_);
	
	//pnh.param<std::string>("init_position_id", init_position_id, "init_position");
	
	// Creating one MoveGroup interface the whole model
	move_group_robot = new moveit::planning_interface::MoveGroup(s_robot_group);
	vector<string> robot_joints = move_group_robot->getJoints();
	
	ROS_INFO("%s::rosSetup: Follow joint trajectory action on %s",component_name.c_str(), follow_joint_traj_name.c_str());
	ROS_INFO("%s::rosSetup: Robot group %s",component_name.c_str(), s_robot_group.c_str());
	
	// Initializing JointState map to save the joint state values
	for(int j = 0; j<robot_joints.size();j++){
		ROS_INFO("\tjoint %s", robot_joints[j].c_str());
		robot_joint_state[robot_joints[j]] = 0.0;
	}
	
	
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
		//mgs.mg->setStartStateToCurrentState();
		mgs.selected_tcp = mgs.mg->getEndEffectorLink();
		// Saves the list of joint of this groups to avoid calling the function iteratively
		mgs.joint_names = mgs.mg->getActiveJoints(); 
		for(int j = 0; j<mgs.joint_names.size();j++){
			ROS_INFO("\tjoint %s", mgs.joint_names[j].c_str());
			// Linking joint value with the map 
			mgs.joint_values.push_back(&robot_joint_state[mgs.joint_names[j]]);
		}
		v_move_groups.push_back(mgs); 
	}
	
	// Sets the first group name by default
	selectJointStateGroup(v_move_groups_name[0]);
	
	//
	// Publishers
	//
	display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	state_publisher = pnh.advertise<robotnik_trajectory_planner::State>("state", 1, true);
	joint_command_publisher = nh.advertise<sensor_msgs::JointState>("/joint_commands", 1, true);
	
	//
	// Subscribers
	//
	jointbyjoint_subscriber = pnh.subscribe("commands/joint_by_joint", 1, &RtTrajPlanner::jointByJointCallback, this);
	cartesianeuler_subscriber = pnh.subscribe("commands/cartesian_euler", 1, &RtTrajPlanner::cartesianEulerCallback, this);
	trajectory_subscriber = pnh.subscribe("commands/trajectory", 1, &RtTrajPlanner::trajectoryCallback, this);
	torso_control_state_subscriber = nh.subscribe(control_state_topic_name, 1, &RtTrajPlanner::controlStateCallback, this);
	joint_state_subscriber = nh.subscribe("/joint_states", 1, &RtTrajPlanner::jointStateCallback, this);
	
	//
	// Services
	//
	select_group_service = pnh.advertiseService("select_group",  &RtTrajPlanner::selectGroupSrv, this);
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
			if((control_type == CARTESIAN_EULER or control_type == JOINTBYJOINT) and control_mode == VELOCITY)
				st.goal_state = "IDLE";
			else{
				actionlib::SimpleClientGoalState goal_state = this->ac_follow_joint_traj->getState();
				st.goal_state = goal_state.toString();
			}
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
	
	// It only accepts commands if there isn't any goal being executed
	if( not goal_active){	
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
	}
	//ROS_INFO("Received point");
} 


/*!	\fn  void RtTrajPlanner::cartesianEulerCallback(const robotnik_trajectory_planner::CartesianEulerConstPtr& j)
 * 	\brief ROS callback handler when receiving CartesianEuler msgs
 */
void RtTrajPlanner::cartesianEulerCallback(const robotnik_trajectory_planner::CartesianEulerConstPtr& c){
	// It only accepts commands if there isn't any goal being executed
	if( not goal_active){
		cartesian_euler_msg.msg = *c;
		cartesian_euler_msg.t = ros::Time::now();
		cartesian_euler_msg.processed = false;
	}
}


/*!	\fn  void RtTrajPlanner::trajectoryCallback(const robotnik_trajectory_planner::TrajectoryConstPtr& t)
 * 	\brief ROS callback handler when receiving Trajectory msgs
 */
void RtTrajPlanner::trajectoryCallback(const robotnik_trajectory_planner::TrajectoryConstPtr& t){
	
	// Constraints:
	// Joint names has to be part of current group
	// There has to be the same number of names and values
	
	// It only accepts commands if there isn't any goal being executed
	if( not goal_active){
		
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


/*!	\fn  bool RtTrajPlanner::checkCollision(collision_detection::CollisionResult *res = NULL, robot_state::RobotStatePtr k_state, int *collision_level)
 * 	\brief Checks for collision
 * 	\param collision_level as int*, defines different levels of proximity
 */
bool RtTrajPlanner::checkCollision(collision_detection::CollisionResult *res, robot_state::RobotStatePtr k_state, int *collision_level){
	bool ret = false;
	
	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
	
	planning_scene->setCurrentState(*k_state);
	collision_request.contacts = true;
	collision_request.max_contacts = 1000;
	collision_request.group_name = current_move_group->name_group;  
	collision_request.distance = true;
	
	*collision_level = -1;
	
	planning_scene->checkCollision(collision_request, collision_result);
	
	if(collision_result.collision){
		collision_detection::CollisionResult::ContactMap::const_iterator it;
		for(it = collision_result.contacts.begin();
			it != collision_result.contacts.end();
			++it)
		{
		  ROS_ERROR("RtTrajPlanner::checkCollision: Contact between: %s and %s",
				   it->first.first.c_str(),
				   it->first.second.c_str());
		}
		
		ret = true;
		
		*collision_level = 0;
		
	}else if(collision_result.distance <= collision_distance_1_){
		  ROS_DEBUG("RtTrajPlanner::checkCollision: level 1 Contact at %.3lf m", collision_result.distance);
		  *collision_level = 1;

	}else if(collision_result.distance > collision_distance_1_ && collision_result.distance <= collision_distance_2_){
		  ROS_DEBUG("RtTrajPlanner::checkCollision: level 2 Contact at %.3lf m", collision_result.distance);
		  *collision_level = 2;
		
	}else if(collision_result.distance > collision_distance_2_ && collision_result.distance <= collision_distance_3_){
		  ROS_DEBUG("RtTrajPlanner::checkCollision: level 3 Contact at %.3lf m", collision_result.distance);
		  *collision_level = 3;
		  
	}
	
	*res = collision_result;
	
	return ret;
}

/*!	\fn  bool RtTrajPlanner::checkCollision(collision_detection::CollisionResult *res = NULL, robot_state::RobotState &k_state, int *collision_level)
 * 	\brief Checks for collision
 * 	\param collision_level as int*, defines different levels of proximity
 */
bool RtTrajPlanner::checkCollision(collision_detection::CollisionResult *res, robot_state::RobotState &k_state, int *collision_level){
	bool ret = false;
	
	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
	
	planning_scene->setCurrentState(k_state);
	collision_request.contacts = true;
	collision_request.max_contacts = 1000;
	collision_request.group_name = current_move_group->name_group;  
	collision_request.distance = true;
	
	*collision_level = -1;
	
	planning_scene->checkCollision(collision_request, collision_result);
	
	if(collision_result.collision){
		collision_detection::CollisionResult::ContactMap::const_iterator it;
		for(it = collision_result.contacts.begin();
			it != collision_result.contacts.end();
			++it)
		{
		  ROS_ERROR("RtTrajPlanner::checkCollision: Contact between: %s and %s",
				   it->first.first.c_str(),
				   it->first.second.c_str());
		}
		
		ret = true;
		
		*collision_level = 0;
		
	}else if(collision_result.distance <= collision_distance_1_){
		  ROS_DEBUG("RtTrajPlanner::checkCollision: level 1 Contact at %.3lf m", collision_result.distance);
		  *collision_level = 1;

	}else if(collision_result.distance > collision_distance_1_ && collision_result.distance <= collision_distance_2_){
		  ROS_DEBUG("RtTrajPlanner::checkCollision: level 2 Contact at %.3lf m", collision_result.distance);
		  *collision_level = 2;
		
	}else if(collision_result.distance > collision_distance_2_ && collision_result.distance <= collision_distance_3_){
		  ROS_DEBUG("RtTrajPlanner::checkCollision: level 3 Contact at %.3lf m", collision_result.distance);
		  *collision_level = 3;
		  
	}
	
	*res = collision_result;
	
	return ret;
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
		

/*!	\fn RtTrajPlanner::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
 * 	\brief Topic callback to move all the joints through joint state interface
*/
void RtTrajPlanner::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{	

	bool match_key = false;
	
	for(int i = 0; i<msg->name.size(); i++){
		try{
			robot_joint_state.at(msg->name[i]) = msg->position[i];
			match_key = true;
		}catch (const std::out_of_range& oor) {
			//ROS_ERROR( "RtTrajPlanner::jointStateCallback: joint %s, Out of Range error: %s",msg->name[i].c_str(), oor.what());
		}
	}
	
	//if(match_key)
	//	timer_joints = ros::Time::now();

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

