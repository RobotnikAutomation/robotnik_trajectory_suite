/*! \class RtTrajPlanner
 *  \file RtTrajPlanner.h
 *	\author Robotnik Automation S.L.L
 *	\version 0.1.0
 *	\date 2015
 *  \brief Class to plan and control the trajectories for the RB1 robot
 * 
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */
 
#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <vector>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/PoseStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
//#include <control_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <sensor_msgs/JointState.h>

#include <XmlRpcValue.h>


#include <robotnik_trajectory_planner/State.h>
#include <robotnik_trajectory_planner/SelectGroup.h>
#include <robotnik_trajectory_planner/JointByJoint.h>
#include <robotnik_trajectory_planner/SetControlMode.h>
#include <robotnik_trajectory_planner/CartesianEuler.h>
#include <robotnik_trajectory_planner/GetJointsGroup.h>
#include <robotnik_trajectory_planner/Trajectory.h>
#include <robotnik_trajectory_planner/PointTraj.h>
#include <robotnik_trajectory_planner/GetGroups.h>
#include <robotnik_trajectory_control/State.h>
#include <robotnik_trajectory_control/TrajExecActions.h>

#include <robotnik_msgs/State.h>
#include <robotnik_trajectory_manager/LoadState.h>
#include <std_srvs/Empty.h>

using namespace std;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;

#ifndef __RTTRAJPLANNER_H
	#define __RTTRAJPLANNER_H

//! Size of string for logging
#define DEFAULT_THREAD_DESIRED_HZ		40.0
#define DEFAULT_T_PLAN					10.0
#define DEFAULT_N_ATTEMPTS				3
#define WATCHDOG_COMMAND				0.1	// Timeout for receiving commands (in velocity mode)
#define DEFAULT_JOINT_ACCEL				0.1 // Acceleration applied to joints
#define DEFAULT_JOINT_VEL				0.1 // Velocity applied to joints
#define MAX_JOINT_VEL					0.1 // Max joint velocity allowed
#define MAX_JOINT_VEL_CARTESIAN			0.1 // Max joint velocity allowed
#define MAX_JOINT_VEL_TRAJECTORY		0.3 // Max joint velocity allowed
#define MAX_JOINT_VEL_JOINTBYJOINT		0.2 // Max joint velocity allowed
#define MAX_LINEAR_VEL					0.06 // m/s
#define MAX_JOINT_VEL_2					1.0 // rad/s
#define DESIRED_MAX_JOINT_VEL			0.1	// MAx joint velocity allowed
#define MIN_JOINT_POSITION_INC			0.01 // Min resolution accepted by the arm
#define DEFAULT_COLLISION_DISTANCE_1	0.002	// distance in m
#define DEFAULT_COLLISION_DISTANCE_2	0.02	// distance in m
#define DEFAULT_COLLISION_DISTANCE_3	0.03	// distance in m
#define DEFAULT_PLANNER_ID				"RRTkConfigDefault"
#define TIME_IN_FAILURE					2	// time (seconds) in Failure before switching to a new state

//! Defines return values for methods and functions
enum ReturnValue{
	OK = 0,
	INITIALIZED,
	THREAD_RUNNING,
	ERROR = -1,
	NOT_INITIALIZED = -2,
	THREAD_NOT_RUNNING = -3,
	COM_ERROR = -4,
	NOT_ERROR = -5
};

//! Supported control modes 
enum ControlMode{
	POSITION = 111,
	VELOCITY = 112
};

//! Supported control types
enum ControlType{
	CARTESIAN_EULER,
	JOINTBYJOINT,
	TRAJECTORY
};

//! Substates
enum Substate{
	INIT_SETUP,
	INIT_INIT_ARMS,
	INIT_INIT_WAIST,
	INIT_MOVE_ARMS,
	INIT_INIT_LEFT_GRIPPER,
	INIT_INIT_RIGHT_GRIPPER,
	INIT_INIT_HEAD
};


//! Struct to save a Move group iface together with its name
typedef struct MoveGroupStruct{
	moveit::planning_interface::MoveGroup *mg;
	string name_group;
	//! saves the list of joints of the group
	vector<string> joint_names;
	//! Points the joint values
	vector<double *> joint_values;
	string selected_tcp;
}MoveGroupStruct;

//! Struct to save JointByJointMsgs
typedef struct JointByJointMsg{
	//! Message received
	robotnik_trajectory_planner::JointByJoint msg;
	//! Time of receivement
	ros::Time t;
	//! Flag to know if it has been processed
	bool processed;
}JointByJointMsg;


//! Struct to save CartesianEuler Msgs
typedef struct CartesianEulerMsg{
	//! Message received
	robotnik_trajectory_planner::CartesianEuler msg;
	//! Time of receivement
	ros::Time t;
	//! Flag to know if it has been processed
	bool processed;
}CartesianEulerMsg;


//! Struct to save Trajectory Msgs
typedef struct TrajectoryMsg{
	//! Message received
	robotnik_trajectory_planner::Trajectory msg;
	//! Time of receivement
	ros::Time t;
	//! Flag to know if it has been processed
	bool processed;
	//! Index of the point being executed
	int current_point;
}TrajectoryMsg;


//! Class Rcomponent
class RtTrajPlanner{
	protected:
		//! Controls if has been initialized succesfully
		bool initialized, ros_initialized;
		//! Controls the execution of the RtTrajPlanner's thread
		bool running;
		
		//! Contains data for the secondary threads
		//vector<thread_data> vThreadData;
		//! State of the RtTrajPlanner
		int state;
		//! Substate in INIT_STATE
		int substate_init;
		//! Substate used in ReadySubstate
		//int ready_substate, previous_state;
		//! VELOCITY, POSITION
		int control_mode;
		//! Type of control (CARTESIAN-EULER, JOINTBYJOINT, TRAJECTORY)
		int control_type;
		
		//! State before
		int previous_state;
		//!	Saves the name of the component
		string component_name;
		//! ROS node handle
		ros::NodeHandle nh;
		//! Private ROS node handle
		ros::NodeHandle pnh;
		//! Desired loop frequency
		double desired_freq;
		double real_freq;
		//!	Flag to know if a goal has been sent to the action server
		bool goal_active;
		//! Flag to save the request of initialization
		bool initialization_request;
		//! Min joint increment(resolution) accepted by the arm controller
		double min_joint_position_inc_;
		
		//! Sets the collision distance
		double collision_distance_1_;
		double collision_distance_2_;
		double collision_distance_3_;
		
		// PUBLISHERS
		//! Creates a publisher for visualizing plans in Rviz.
		ros::Publisher display_publisher;
		//! Publishes component state
		ros::Publisher state_publisher;
		robotnik_trajectory_planner::State st;
		//! Publishes to dpro_controller
		ros::Publisher joint_command_publisher;
		
		//! Saves the time whenever receives control state msg
		ros::Time t_received_control_state;
		//! Saves the time when the component moves to failure state
		ros::Time t_failure_state;
		// SUBSCRIBERS
		ros::Subscriber jointbyjoint_subscriber;
		ros::Subscriber cartesianeuler_subscriber;
		ros::Subscriber trajectory_subscriber;
		ros::Subscriber torso_control_state_subscriber;
		ros::Subscriber joint_state_subscriber;
		//! Saves the state of the robotnik control
		robotnik_trajectory_control::State robotnik_control_state;
		
		// SERVICES
		//! Service to select group
		ros::ServiceServer select_group_service;
		//! Service to set the control mode
		ros::ServiceServer set_control_mode_service;
		//! Service to get the name of the joints
		ros::ServiceServer get_joints_group_service;
		//! Service to get all the available groups
		ros::ServiceServer get_groups_service;
		//! Service to initialize all the components
		ros::ServiceServer init_service;
		
		// SERVICE CLIENTS
		//! Service to call actions from robotnik_trajectory_control
		ros::ServiceClient action_control_client;
		//! Nave of service set in action_control_client
		string control_state_actions_service_name; 
		//! Service to load a trajectory
		ros::ServiceClient load_trajectory_client;
		//! Nave of the service to load a trajectory
		string load_trajectory_service_name; 
		//! Id of the position that it'll load in INIT_STATE
		string init_position_id; 
		
		//! planner used by MoveGroup
		string planner_id_;
		// The :move_group_interface:`MoveGroup` class can be easily 
		// setup using just the name
		// of the group you would like to control and plan for.
		//! Points to the current JointStateGroup
		MoveGroupStruct *current_move_group;
		//! Vector with all the available move groups
		vector<MoveGroupStruct> v_move_groups; 
		
		//! Saves the current state of the joints
		std::map<std::string, double> robot_joint_state;
		
		moveit::planning_interface::MoveGroup *move_group_robot; 
		// We will use the :planning_scene_interface:`PlanningSceneInterface`
		// class to deal directly with the world.
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 
		planning_scene_monitor::PlanningSceneMonitor *planning_scene_monitor; 
		planning_scene::PlanningScene *planning_scene;
		
		//! Load the robot model 
		robot_model_loader::RobotModelLoader* robot_model_loader_ptr;
		//! Pointer to the kinematic model
		robot_model::RobotModelPtr kinematic_model;
		//! Pointer to the kinematic state
		robot_state::RobotStatePtr kinematic_state;
		robot_state::RobotStatePtr aux_kinematic_state;
		
		//! Maps joint_name <-> joint_values
		std::map<std::string, robot_model::JointModelGroup *> groups2joint_model_group;
		robot_model::JointModelGroup * joint_robot_model_group;
		
		
		// Homogeneous transform containing the transformation that will be applied to current Eigen::Affine3d
		Eigen::Matrix4d m_transform;
		// Client to send Follow Joint Traj actions to the robot
		ActionClient *ac_follow_joint_traj;
		control_msgs::FollowJointTrajectoryGoal goal;
		
		////////////////////////////
		// Read From PARAM server //
		////////////////////////////
		//! vector with all the available groups to control
		vector<std::string> v_move_groups_name;
		//! string with the name of the group of the whole robot
		string s_robot_group;
		//! Saves the name of the service
		string follow_joint_traj_name;
		//! Saves the name of this topic
		string control_state_topic_name;
		
		/////////////////
		// MOTION MSGS //
		/////////////////
		//! Saves the received joint by joint msg
		JointByJointMsg joint_by_joint_msg;
		//! Saves the received cartesian-euler msg
		CartesianEulerMsg cartesian_euler_msg;
		//! Saves the received trajectory msg
		TrajectoryMsg trajectory_msg, current_trajectory_msg;
		
	public:
		//! Public constructor
		RtTrajPlanner(double hz, ros::NodeHandle h);
		//! Public destructor
		~RtTrajPlanner();
		
		//! Starts the control loop of the component and its subcomponents
		//! @return OK
		//! @return ERROR starting the thread
		//! @return RUNNING if it's already running
		//! @return NOT_INITIALIZED if it's not initialized
		int start();
		//! Stops the main control loop of the component and its subcomponents
		//! @return OK
		//! @return ERROR if any error has been produced
		//! @return NOT_RUNNING if the main thread isn't running
		int stop();
		//! Returns the general state of the RtTrajPlanner
		int getState();
		//! Returns the general state of the RtTrajPlanner as string
		char *getStateString();
		//! Returns the general state as string
		char *getStateString(int state);
		//! Returns the general state as string
		char *getSubstateString(int substate);
		//! Method to get current update rate of the thread
		//! @return pthread_hz
		double getUpdateRate();
		//! Gets current control mode as string
		string controlModeToString(int mode);
		//! Gets current control type as string
		string controlTypeToString(int type);
		
				
	protected:
		//! Configures and initializes the component
		//! @return OK
		//! @return INITIALIZED if the component is already intialized
		//! @return ERROR
		int setup();
		//! Closes and frees the reserved resources
		//! @return OK
		//! @return ERROR if fails when closes the devices
		//! @return RUNNING if the component is running
		//! @return NOT_INITIALIZED if the component is not initialized
		int shutdown();
		//! All core component functionality is contained in this thread.
		//!	All of the RtTrajPlanner component state machine code can be found here.
		void controlLoop();
		//! Actions performed on initial state
		void initState();
		//! Actions performed on standby state
		void standbyState();
		//! Actions performed on ready state
		void readyState();
		//! Actions performed on ready state
		void readyState2();
		//! Actions performed on the emergency state
		void emergencyState();
		//! Actions performed on Failure state
		void failureState();
		//! Actions performed on Shutdown state
		void shutdownState();
		//! Actions performed in all states
		void allState();
		//! Switches between states
		void switchToState(int new_state);
		//! Switches between substates
		void switchToSubstate(int new_substate);
		//! Setups all the ROS' stuff
		int rosSetup();
		//! Shutdowns all the ROS' stuff
		int rosShutdown();
		//! Reads data a publish several info into different topics
		void rosPublish();
		
		//! Selects the current JointStateGroup
		int selectJointStateGroup(string group);
		//! ROS Service Handler to set current JointStateGroup
		bool selectGroupSrv(robotnik_trajectory_planner::SelectGroup::Request &req, robotnik_trajectory_planner::SelectGroup::Response &res );
		//! ROS callback handler when receiving JointByJoint msgs
		void jointByJointCallback(const robotnik_trajectory_planner::JointByJointConstPtr& j);
		//! ROS callback handler when receiving CartesianEuler msgs
		void cartesianEulerCallback(const robotnik_trajectory_planner::CartesianEulerConstPtr& c);
		//! ROS callback handler when receiven Trajectory msgs
		void trajectoryCallback(const robotnik_trajectory_planner::TrajectoryConstPtr& t);
		//! Checks for collision
		bool checkCollision(collision_detection::CollisionResult *res, robot_state::RobotStatePtr k_state, int *collision_level);
		bool checkCollision(collision_detection::CollisionResult *res, robot_state::RobotState &k_state, int *collision_level);
		//! ROS service to get the joints of a group
		bool getJointsGroupSrv(robotnik_trajectory_planner::GetJointsGroup::Request &req, robotnik_trajectory_planner::GetJointsGroup::Response &res );
		//! ROS service to get all the available groups
		bool getGroupsSrv(robotnik_trajectory_planner::GetGroups::Request &req, robotnik_trajectory_planner::GetGroups::Response &res );
		//! ROS service to intialize the components 
		bool initSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res );
		
		void controlStateCallback(const robotnik_trajectory_control::StateConstPtr& t);
		//! Sends an action to the robotnik_trajectory_control component
		int sendActionToControl(int action);
		//! Executes the jointbyjoint position control
		int sendJointByJointPosition();
		//! Executes the jointbyjoint position control
		int sendCartesianEulerPosition();
		//! Sends an id to the trajectory manager to load a set of joints and values
		int loadPositionFromManager(string id);
		//! Controls and modifies the velocities to not be greater than a maximum 
		void controlMaxVelocities(vector<double> &velocities, double max_vel);
		//! Callback to process the joint_state msgs
		void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
};

#endif
