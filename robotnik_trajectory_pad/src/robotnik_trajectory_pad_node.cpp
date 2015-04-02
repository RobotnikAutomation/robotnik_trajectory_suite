/*
 * robotnik_trajectory_pad
 * Copyright (c) 2013, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik Automation, SLL
 * \brief Allows to use a pad with the robot controller, sending the messages received from the joystick device
 */


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
#include <vector>
//#include <torso_pad/enable_disable.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"

#include <robotnik_trajectory_planner/SelectGroup.h>
#include <robotnik_trajectory_planner/JointByJoint.h>
#include <robotnik_trajectory_planner/SetControlMode.h>
#include <robotnik_trajectory_planner/CartesianEuler.h>
#include <robotnik_trajectory_planner/GetJointsGroup.h>
#include <robotnik_trajectory_pad/State.h>
#include <robotnik_msgs/State.h>
//#include <bhand/Actions.h>
//#include <bhand/Service.h>
//#include <wsg_50_common/Move.h>


#include <XmlRpcValue.h>

#define MAX_NUM_OF_BUTTONS			16
#define MAX_NUM_OF_AXES				8
#define MAX_NUM_OF_BUTTONS_PS3		19
#define MAX_NUM_OF_AXES_PS3			20

#define DEFAULT_NUM_OF_BUTTONS		16
#define DEFAULT_NUM_OF_AXES			8

#define DEFAULT_AXIS_LINEAR_X		1
#define DEFAULT_AXIS_ANGULAR		0	
#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		1.0

#define MAX_GRIPPER_VEL				100

#define DEFAULT_JOY			"/joy"

#define DEFAULT_HZ			50.0

#define CARTESIAN_CONTROL			1
#define JOINTBYJOINT_CONTROL		2

#define MAX_AXIS_VELOCITY			0.1 // m/s
#define MAX_JOINT_VELOCITY			0.1 // rad/s

using namespace std;

//! Struct to save the name of a group and its joints and values
typedef struct MoveGroupsJoints{
	//! Name of the group
	string name;
	//! Selected joint to modify
	int selected_joint;
	//! Joint names 
	vector<string> v_joint_names;
	//! Values of joints (NOT used)
	vector<double> v_joint_values;
}MoveGroupsJoints;


//! Class to save the state of the buttons
class Button{
	int iPressed;
	bool bReleased;
	
	public:
	
	Button(){
		iPressed = 0;
		bReleased = false;
	}
	//! Set the button as 'pressed'/'released'
	void Press(int value){		
		if(iPressed and !value){
			bReleased = true;
			
		}else if(bReleased and value)
			bReleased = false;
			
		iPressed = value;
			
	}
	
	int IsPressed(){
		return iPressed;
	}
	
	bool IsReleased(){
		bool b = bReleased;
		bReleased = false;
		return b;
	}
};

////////////////////////////////////////////////////////////////////////
//                               		                                //
////////////////////////////////////////////////////////////////////////
class RobotnikTrajectoryPad
{
	public:
	
	RobotnikTrajectoryPad();
	
	void ControlLoop();
	
	private:
	
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	
	char * StateToString(int state);
	int SwitchToState(int new_state);
	
	void PublishState();
	//! Enables/Disables the joystick
	//bool EnableDisable(robotnik_msgs::enable_disable::Request &req, robotnik_msgs::enable_disable::Response &res );
	void Update();
	
	//! Changes the current control group
	void ChangeGroup(int next);
	//! Gets all the joints of every group
	void GetJointsGroup();
	//! Sets the internal control type (CARTESIAN, JOINTBYJOINT)
	void SetControlType(int type);
	//! Sets the planner control mode (POSITION, VELOCITY)
	void SetControlMode(string mode);
	//! Increase/Decrease the joint number
	void SetJoint(int next);
	
private:	
	
	ros::NodeHandle pnh_; // Private node handle
	ros::NodeHandle nh_; 

	//! Axis numbers 
	int axis_x, axis_y, axis_z, axis_pitch, axis_roll, axis_yaw, axis_move_joint;
	
	double l_scale_, a_scale_;
	double current_speed_lvl;
	//! Set the max speed sent to the robot
	double max_axis_velocity, max_joint_velocity;
	
	//! Desired component's freq
	double desired_freq_;
	//! Robot control type (CARTESIAN, JOINT BY JOINT)
	int control_type;
	
	// TOPICS
	// PUBLISHER
	//! It will publish to cartesian command
	ros::Publisher cartesian_pub_;
	//! It will publish to joint by joint command
	ros::Publisher jointbyjoint_pub_;
	//! Topic to publish the state
	ros::Publisher state_pub_;
	//! Topic to move the gripper
	ros::Publisher gripper_pub_;
	//! Name of the topic to publish
	string cmd_topic_cartesian;
	string cmd_topic_jointbyjoint;
	
	// SUBSCRIBER
	//! they will be suscribed to the joysticks
	ros::Subscriber joy_sub_;
	//! Subscribes to the planner node state
	ros::Subscriber planner_state_sub_;
	//! topic name for the state
	string topic_planner_state_;
	
	//! // Name of the joystick's topic
	string  joy_topic_;	
	//! Name of services
	string select_group_service_name_;
	string set_mode_service_name_;
	string get_joints_group_service_name_;
	string bhand_actions_service_name_;
	string wsg50_grasp_service_name_;
	string wsg50_release_service_name_;
	string terabot_gripper_topic_name_;
	//! Name of the param to get the available groups to control
	string param_groups_name_;
	
	//! Vector with all the available move groups
	vector<MoveGroupsJoints> v_groups; 
	//! Position of the vector for the selected group
	int selected_group;
		
	// SERVICES
	//! Enables/disables the pad
	//ros::ServiceServer enable_disable_srv_;
	//! Service to communicate with the planner
	ros::ServiceClient select_group_service_;  
	ros::ServiceClient set_control_mode_service_;  // Sets Control position or velocity
	ros::ServiceClient get_joints_group_service_;  
	//! Service to communicate with the BHAND
	ros::ServiceClient bhand_actions_service_;  
	//! Services to communicate with the WSG50
	ros::ServiceClient wsg50_grasp_service_;  
	ros::ServiceClient wsg50_release_service_;  
	
	// JOYSTICK
	//! Current number of buttons of the joystick
	int num_of_buttons_;
	int num_of_axes_;
	
	//! Vector to save the axis values
	std::vector<float> fAxes;
	//! Vector to save and control the axis values
	std::vector<Button> vButtons;
	//! Button's number associated to every functionality
	int button_dead_man_, button_euler_mode_, button_change_group_right_, button_change_group_left_, button_set_control_mode_,
	 button_set_control_type_, button_increase_joint_, button_decrease_joint_;
	int button_control_bhand_, button_control_bhand_grasp_, button_control_bhand_release_, button_control_bhand_mode1_, button_control_bhand_mode2_;
	int button_control_wsg50_, button_control_wsg50_grasp_, button_control_wsg50_release_;
	int button_control_terabot_grasp_, button_control_terabot_release_;
	
	//! Number of the button for increase or decrease the speed max of the joystick	
	int button_speed_up_, button_speed_down_;
		
	// DIAGNOSTICS
	//! Diagnostic to control the frequency of the published command velocity topic
	diagnostic_updater::HeaderlessTopicDiagnostic *pub_command_freq; 
	//! Diagnostic to control the reception frequency of the subscribed joy topic 
	diagnostic_updater::HeaderlessTopicDiagnostic *sus_joy_freq; 
	//! General status diagnostic updater
	diagnostic_updater::Updater updater_pad;	
	//! Diagnostics min freq
	double min_freq_command, min_freq_joy; 
	//! Diagnostics max freq
	double max_freq_command, max_freq_joy; 	
	//! Flag to enable/disable the communication with the publishers topics
	bool bEnable;

protected:
	//! Sends an action to the BHand
	void sendBHandAction(int action);
	//! Sends a grasp action to the WSG50
	void sendWSG50GraspAction();
	//! Sends a release action to the WSG50
	void sendWSG50ReleaseAction();
};


RobotnikTrajectoryPad::RobotnikTrajectoryPad():
  pnh_("~")
{	
	control_type = CARTESIAN_CONTROL;
	current_speed_lvl = 0.1;
	// JOYSTICK CONFIG
	pnh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);
	pnh_.param("num_of_axes", num_of_axes_, DEFAULT_NUM_OF_AXES);
	pnh_.param("desired_freq", desired_freq_, DEFAULT_HZ);
	
	if(num_of_axes_ > MAX_NUM_OF_AXES){
		num_of_axes_ = MAX_NUM_OF_AXES;
		ROS_INFO("RobotnikTrajectoryPad::RobotnikTrajectoryPad: Limiting the max number of axes to %d", MAX_NUM_OF_AXES);
	}
	if(num_of_buttons_ > MAX_NUM_OF_BUTTONS){
		num_of_buttons_ = MAX_NUM_OF_BUTTONS;
		ROS_INFO("RobotnikTrajectoryPad::RobotnikTrajectoryPad: Limiting the max number of buttons to %d", MAX_NUM_OF_BUTTONS);
	}
	
	pnh_.param("topic_joy", joy_topic_, std::string(DEFAULT_JOY));	
	
	// MOTION CONF
	pnh_.param("cmd_topic_cartesian", cmd_topic_cartesian, std::string("/rt_traj_planner/commands/cartesian_euler"));
	pnh_.param("cmd_topic_jointbyjoint", cmd_topic_jointbyjoint, std::string("/rt_traj_planner/commands/jointbyjoint"));
	
	pnh_.param("button_dead_man", button_dead_man_, button_dead_man_);
	pnh_.param("button_speed_up", button_speed_up_, button_speed_up_);
	pnh_.param("button_speed_down", button_speed_down_, button_speed_down_); 
	pnh_.param("button_euler_mode", button_euler_mode_, button_euler_mode_); 
	pnh_.param("button_change_group_right", button_change_group_right_, button_change_group_right_); 
	pnh_.param("button_change_group_left", button_change_group_left_, button_change_group_left_); 
	pnh_.param("button_set_control_mode", button_set_control_mode_, button_set_control_mode_); 
	pnh_.param("button_set_control_type", button_set_control_type_, button_set_control_type_); 
	pnh_.param("button_increase_joint", button_increase_joint_, button_increase_joint_); 
	pnh_.param("button_decrease_joint", button_decrease_joint_, button_decrease_joint_); 
	// BHAND buttons
	pnh_.param("button_control_bhand", button_control_bhand_, button_control_bhand_); 
	pnh_.param("button_control_bhand_grasp", button_control_bhand_grasp_, button_control_bhand_grasp_); 
	pnh_.param("button_control_bhand_release", button_control_bhand_release_, button_control_bhand_release_); 
	pnh_.param("button_control_bhand_mode1", button_control_bhand_mode1_, button_control_bhand_mode1_); 
	pnh_.param("button_control_bhand_mode2", button_control_bhand_mode2_, button_control_bhand_mode2_); 
	// WSG50 Buttons
	pnh_.param("button_control_wsg50", button_control_wsg50_, button_control_wsg50_); 
	pnh_.param("button_control_wsg50_grasp", button_control_wsg50_grasp_, button_control_wsg50_grasp_); 
	pnh_.param("button_control_wsg50_release", button_control_wsg50_release_, button_control_wsg50_release_); 
	pnh_.param("button_control_terabot_grasp", button_control_terabot_grasp_, button_control_terabot_grasp_); 
	pnh_.param("button_control_terabot_release", button_control_terabot_release_, button_control_terabot_release_); 
	
	pnh_.param("axis_x", axis_x, axis_x); 
	pnh_.param("axis_y", axis_y, axis_y); 
	pnh_.param("axis_z", axis_z, axis_z); 
	pnh_.param("axis_pitch", axis_pitch, axis_pitch); 
	pnh_.param("axis_roll", axis_roll, axis_roll); 
	pnh_.param("axis_yaw", axis_yaw, axis_yaw); 
	pnh_.param("axis_move_joint", axis_move_joint, axis_move_joint); 
	
	pnh_.param("max_axis_velocity", max_axis_velocity, MAX_AXIS_VELOCITY); 
	pnh_.param("max_joint_velocity", max_joint_velocity, MAX_JOINT_VELOCITY); 
	
	//ROS_INFO("axis_linear_speed_ = %d, axis_angular = %d", axis_linear_speed_, axis_angular_position_);
	//ROS_INFO("max_linear_speed = %lf, max_angular_speed = %lf", max_linear_speed_, max_angular_position_);
	

	pnh_.param("srv_select_group", select_group_service_name_, string("/rt_traj_planner/select_group"));
	pnh_.param("srv_set_mode", set_mode_service_name_, string("/rt_traj_planner/set_control_mode"));
	pnh_.param("srv_get_joints_group", get_joints_group_service_name_, string("/rt_traj_planner/get_joints_group"));
	pnh_.param("param_groups", param_groups_name_, string("/rt_traj_planner/move_groups")); 
	pnh_.param("bhand_actions_service_name", bhand_actions_service_name_, string("/bhand_node/actions"));
	pnh_.param("wsg50_grasp_service_name", wsg50_grasp_service_name_, string("/wsg_50/grasp"));
	pnh_.param("wsg50_release_service_name", wsg50_release_service_name_, string("/wsg_50/release"));
	pnh_.param("terabot_gripper_topic_name", terabot_gripper_topic_name_, string("/terabot/gripper"));
	//pnh_.param("topic_planner_state", topic_planner_state_, std::string("/rt_traj_planner/state"));
	
	
	XmlRpc::XmlRpcValue list;
	nh_.getParam(param_groups_name_, list);
	
	// Checks that the read param type is correct
	if(list.getType() != XmlRpc::XmlRpcValue::TypeArray){
		ROS_ERROR("Wrong read type (%d) for move_groups param", list.getType());
	}else{
		// Saves the array into class vector
		for(int32_t i = 0; i < list.size(); ++i) 
		{
			MoveGroupsJoints mgj;
			mgj.name = static_cast<std::string>(list[i]);
			v_groups.push_back(mgj);
			ROS_INFO("RobotnikTrajectoryPad: Move group %d -> %s", i, v_groups[i].name.c_str());
		}
	}
	
	
	//ROS_INFO("RobotnikTrajectoryPad num_of_buttons_ = %d, axes = %d, topic controller: %s, hz = %.2lf", num_of_buttons_, num_of_axes_, cmd_topic_vel.c_str(), desired_freq_);	
	
	for(int i = 0; i < MAX_NUM_OF_BUTTONS_PS3; i++){
		Button b;
		vButtons.push_back(b);
	}
	
	for(int i = 0; i < MAX_NUM_OF_AXES_PS3; i++){
		fAxes.push_back(0.0);
	}
	
	//
  	// Publish through the node handle Twist type messages to the guardian_controller/command topic
  	this->cartesian_pub_ = pnh_.advertise<robotnik_trajectory_planner::CartesianEuler>(this->cmd_topic_cartesian, 1);
  	this->jointbyjoint_pub_ = pnh_.advertise<robotnik_trajectory_planner::JointByJoint>(this->cmd_topic_jointbyjoint, 1);
  	this->gripper_pub_ = pnh_.advertise<std_msgs::Int32>(this->terabot_gripper_topic_name_, 1);
	
	//
	// Publishes the state
	state_pub_ = pnh_.advertise<robotnik_trajectory_pad::State>("state", 1);
	
 	// Listen through the node handle sensor_msgs::Joy messages from joystick 
	// (these are the references that we will sent to rescuer_controller/command)
	joy_sub_ = pnh_.subscribe<sensor_msgs::Joy>(joy_topic_, 1, &RobotnikTrajectoryPad::joyCallback, this);
	
 	// SERVICES
 	ros::service::waitForService(select_group_service_name_);
 	ros::service::waitForService(set_mode_service_name_);
 	ros::service::waitForService(get_joints_group_service_name_);
	select_group_service_ = pnh_.serviceClient<robotnik_trajectory_planner::SelectGroup>(select_group_service_name_);
	set_control_mode_service_ = pnh_.serviceClient<robotnik_trajectory_planner::SetControlMode>(set_mode_service_name_);
	get_joints_group_service_ = pnh_.serviceClient<robotnik_trajectory_planner::GetJointsGroup>(get_joints_group_service_name_);
	
	//bhand_actions_service_ = pnh_.serviceClient<bhand::Actions>(bhand_actions_service_name_);
	//wsg50_grasp_service_ = pnh_.serviceClient<wsg_50_common::Move>(wsg50_grasp_service_name_);
	//wsg50_release_service_ = pnh_.serviceClient<wsg_50_common::Move>(wsg50_release_service_name_);
	
	
	// Diagnostics
	updater_pad.setHardwareID("TORSO-PAD");
	// Topics freq control 
	min_freq_command = min_freq_joy = 5.0;
	max_freq_command = max_freq_joy = 50.0;
	sus_joy_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/joy", updater_pad,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq_joy, &max_freq_joy, 0.1, 10));

	/*pub_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(cmd_topic_vel.c_str(), updater_pad,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq_command, &max_freq_command, 0.1, 10));*/

	// Advertises new service to enable/disable the pad
	//enable_disable_srv_ = pnh_.advertiseService("/robotnik_trajectory_pad/enable_disable",  &RobotnikTrajectoryPad::EnableDisable, this);
	//
	bEnable = true;	// Communication flag enabled by default
	
	if(v_groups.size() > 0){
		selected_group = 0;
		// Gets all the joints of each group
		GetJointsGroup();
	}else
		selected_group = -1; // No groups
		
	
}


/*
 *	\brief Updates the diagnostic component. Diagnostics
 * 		   Publishes the state
 *
 */
void RobotnikTrajectoryPad::Update(){
	PublishState();
}

//! 
void RobotnikTrajectoryPad::PublishState(){
	robotnik_trajectory_pad::State pad_state;
	
	
	pad_state.state.state = robotnik_msgs::State::READY_STATE;
	pad_state.state.state_description = string("READY");
	pad_state.state.desired_freq = desired_freq_;
	
	if(control_type == JOINTBYJOINT_CONTROL)
		pad_state.control_type = string("JOINT_BY_JOINT");
	else
		pad_state.control_type = string("CARTESIAN_EULER");
	
	pad_state.current_group = v_groups[selected_group].name;
	pad_state.selected_joint = v_groups[selected_group].v_joint_names[v_groups[selected_group].selected_joint];
	
	pad_state.velocity_level = current_speed_lvl;
	pad_state.deadman_active = (bool) vButtons[button_dead_man_].IsPressed();
	
	state_pub_.publish(pad_state);
	
}

/*
 *	\brief Enables/Disables the pad
 *
 */
/*bool RobotnikTrajectoryPad::EnableDisable(robotnik_msgs::enable_disable::Request &req, robotnik_msgs::enable_disable::Response &res )
{
	bEnable = req.value;

	ROS_INFO("RobotnikTrajectoryPad::EnableDisable: Setting to %d", req.value);
	res.ret = true;
	return true;
}*/


/*! \fn void RobotnikTrajectoryPad::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
 * Callback when receiving a joystick command
*/
void RobotnikTrajectoryPad::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	
	// Save values into an internal structre
	for(int i = 0; i < joy->axes.size(); i++){
		this->fAxes[i] = joy->axes[i];
	}
	for(int i = 0; i < joy->buttons.size(); i++){
		this->vButtons[i].Press(joy->buttons[i]);
	}
	
	//ROS_INFO("RobotnikTrajectoryPad::joyCallback: num_of_axes = %d, buttons = %d", (int)(joy->axes.size()), (int)(joy->buttons.size()));
}


/*! \fn void RobotnikTrajectoryPad::ControlLoop()
 *  Controls the actions and states
*/
void RobotnikTrajectoryPad::ControlLoop(){
	
	double desired_linear_speed = 0.0, desired_angular_position = 0.0;	
	robotnik_trajectory_planner::CartesianEuler cartesian_msg;
	robotnik_trajectory_planner::JointByJoint jointbyjoint_msg;
	
	ros::Rate r(desired_freq_);   

    while(ros::ok()) {
		
		Update();
			
		if(bEnable){
			// CHANGE CONTROL GROUP
			if(vButtons[button_change_group_right_].IsReleased()){
				ChangeGroup(1);
			}else if(vButtons[button_change_group_left_].IsReleased()){
				ChangeGroup(-1);
			}
			// SET JOINT
			if(vButtons[button_increase_joint_].IsReleased()){
				SetJoint(1);
			}else if(vButtons[button_decrease_joint_].IsReleased()){
				SetJoint(-1);
			}
			
			// SET CONTROL MODE (ALWAYS VELOCITY)
			if(vButtons[button_set_control_mode_].IsReleased()){
				SetControlMode(string("VELOCITY"));
			}
			
			// Changing internal control type
			if(vButtons[button_set_control_type_].IsReleased()){
				if(control_type == CARTESIAN_CONTROL)
					SetControlType(JOINTBYJOINT_CONTROL);
				else
					SetControlType(CARTESIAN_CONTROL);
			}
							
			if(vButtons[button_dead_man_].IsPressed()){
				
				// Depending the type we'll use some buttons or others
				// 
				// CARTESIAN-EULER CONTROL
				if(control_type == CARTESIAN_CONTROL){
					cartesian_msg.x = cartesian_msg.y = cartesian_msg.z = cartesian_msg.pitch = cartesian_msg.roll = 0.0;
							
					// EULER MODE
					if(vButtons[button_euler_mode_].IsPressed()){
						cartesian_msg.pitch = max_joint_velocity * current_speed_lvl * fAxes[axis_pitch];
						cartesian_msg.roll = max_joint_velocity * current_speed_lvl * fAxes[axis_roll];
						cartesian_msg.yaw = max_joint_velocity * current_speed_lvl * fAxes[axis_yaw];
					}else{
					// CARTESIAN MODE
						cartesian_msg.x = max_axis_velocity * current_speed_lvl * fAxes[axis_x];
						cartesian_msg.y = max_axis_velocity * current_speed_lvl * fAxes[axis_y];
						cartesian_msg.z = max_axis_velocity * current_speed_lvl * fAxes[axis_z];
					}
					
					cartesian_pub_.publish(cartesian_msg);
					
				}else{
				//
				// JOINT BY JOINT CONTROL
					
					if(v_groups.size() > 0){
						jointbyjoint_msg.joints.clear();
						jointbyjoint_msg.values.clear();
						// Sends only the current joint
						jointbyjoint_msg.joints.push_back(v_groups[selected_group].v_joint_names[v_groups[selected_group].selected_joint]);
						jointbyjoint_msg.values.push_back(max_joint_velocity * current_speed_lvl * fAxes[axis_move_joint]) ;
						jointbyjoint_pub_.publish(jointbyjoint_msg);
					}
					
				}		
				
				
				if(vButtons[button_control_terabot_grasp_].IsPressed()){
					std_msgs::Int32 griper_msg;
					griper_msg.data = int(current_speed_lvl * MAX_GRIPPER_VEL);
					gripper_pub_.publish(griper_msg);
				}else if(vButtons[button_control_terabot_release_].IsPressed()){
					std_msgs::Int32 griper_msg;
					griper_msg.data = -int(current_speed_lvl * MAX_GRIPPER_VEL);
					gripper_pub_.publish(griper_msg);
				}
				
				// BHAND
				// Deadman for bhand
				
				if(vButtons[button_control_bhand_].IsPressed()){
				/*	if(vButtons[button_control_bhand_grasp_].IsReleased()){
						sendBHandAction(bhand::Service::CLOSE_GRASP);
					}
					else if(vButtons[button_control_bhand_release_].IsReleased()){
						sendBHandAction(bhand::Service::OPEN_GRASP);
					}
					else if(vButtons[button_control_bhand_mode1_].IsReleased()){
						sendBHandAction(bhand::Service::SET_GRASP_1);
					}
					else if(vButtons[button_control_bhand_mode2_].IsReleased()){
						sendBHandAction(bhand::Service::SET_GRASP_2);
					}*/
				// Deadman for WSG50	
				}else if(vButtons[button_control_wsg50_].IsPressed()){
					/*if(vButtons[button_control_wsg50_grasp_].IsReleased()){
						sendWSG50GraspAction();
					}
					else if(vButtons[button_control_wsg50_release_].IsReleased()){
						sendWSG50ReleaseAction();
					}
				*/
					
				}else{
						// INCREASE/DECREASE MAX SPEED
						if(vButtons[button_speed_up_].IsReleased()){
							current_speed_lvl += 0.1;
							if(current_speed_lvl > 1.0)
								current_speed_lvl = 1.0;
						}
						if(vButtons[button_speed_down_].IsReleased()){
							current_speed_lvl -= 0.1;
							if(current_speed_lvl < 0.0)
								current_speed_lvl = 0.0;
						}
				}
				
			}else if(vButtons[button_dead_man_].IsReleased()){
				
			}
		}
		
		ros::spinOnce();
		r.sleep();
	}
    	
}


/*! \fn void RobotnikTrajectoryPad::ChangeGroup(int next)
 *  Changes the current control group
*/
void RobotnikTrajectoryPad::ChangeGroup(int next){
	if(selected_group >= 0){
		int size = v_groups.size();
		robotnik_trajectory_planner::SelectGroup s;
		
		
		
		selected_group = selected_group + next;
		
		if(selected_group < 0)
			selected_group = size - 1;
		else
			selected_group %= size;
		
		s.request.group_name = v_groups[selected_group].name;
		
		//ROS_INFO("RobotnikTrajectoryPad::ChangeGroup: calling service to change group to %s", v_groups[selected_group].name.c_str());
		if(!select_group_service_.call(s)){
			ROS_ERROR("RobotnikTrajectoryPad::ChangeGroup: ERROR calling service to change group to %s", v_groups[selected_group].name.c_str());
		}
		
	}
}


/*! \fn void RobotnikTrajectoryPad::GetJointsGroup()
 *  Gets all the joints of a every group
*/
void RobotnikTrajectoryPad::GetJointsGroup(){
	
	int size = v_groups.size();
	robotnik_trajectory_planner::GetJointsGroup s;
	
	
	for(int i = 0; i < size; i++){
		s.request.group_name = v_groups[i].name;
		if(!get_joints_group_service_.call(s)){
			ROS_ERROR("RobotnikTrajectoryPad::GetJointsGroup: ERROR calling service to get joints of group to %s", v_groups[i].name.c_str());
		}else{
			v_groups[i].selected_joint = 0;
			v_groups[i].v_joint_names = s.response.ret;
		}
	}

}


/*! \fn void RobotnikTrajectoryPad::SetControlType(int type)
 *  Sets the internal control type (CARTESIAN, JOINTBYJOINT)
*/
void RobotnikTrajectoryPad::SetControlType(int type){
	control_type = type;
	//ROS_INFO("RobotnikTrajectoryPad::SetControlType: Setting type to %d", type);
}


/*! \fn void RobotnikTrajectoryPad::SetControlMode(int type)
 *  Sets the planner control mode (POSITION, VELOCITY)
*/
void RobotnikTrajectoryPad::SetControlMode(string mode){
	robotnik_trajectory_planner::SetControlMode s;
	
	s.request.mode = mode;
	
	if(!set_control_mode_service_.call(s)){
		ROS_ERROR("RobotnikTrajectoryPad::SetControlMode: ERROR calling service to set %s control mode", mode.c_str());
	}
}


/*! \fn void RobotnikTrajectoryPad::SetJoint(int next)
 *  Increase/Decrease the joint number
*/
void RobotnikTrajectoryPad::SetJoint(int next){
	if(v_groups.size() > 0){
		v_groups[selected_group].selected_joint = (v_groups[selected_group].selected_joint + next);
		
		if(v_groups[selected_group].selected_joint < 0){
			v_groups[selected_group].selected_joint = v_groups[selected_group].v_joint_names.size() - 1;
		}else{
			v_groups[selected_group].selected_joint %= v_groups[selected_group].v_joint_names.size();
		}
			
		//ROS_INFO("RobotnikTrajectoryPad::SetJoint: Groupt %s, joint %s", v_groups[selected_group].name.c_str(), v_groups[selected_group].v_joint_names[v_groups[selected_group].selected_joint].c_str());
	}
}



/*! \fn void RobotnikTrajectoryPad::sendBHandAction(int action)
 *  Sends an action to the BHand
*/
void RobotnikTrajectoryPad::sendBHandAction(int action){
	/*bhand::Actions ac;
	
	ac.request.action = action;
	
	if(!bhand_actions_service_.call(ac)){
		ROS_ERROR("RobotnikTrajectoryPad::sendBHandAction: ERROR calling service %d to the bhand", action);
	}*/
}


/*! \fn void RobotnikTrajectoryPad::sendWSG50GraspAction()
 *  Sends a grasp action to the WSG50
*/
void RobotnikTrajectoryPad::sendWSG50GraspAction(){
	/*wsg_50_common::Move ac;
	
	ac.request.width = 0.0;
	ac.request.speed = 5.0;
	
	if(!wsg50_grasp_service_.call(ac)){
		ROS_ERROR("RobotnikTrajectoryPad::sendWSG50GraspAction: ERROR calling service to the wsg50");
	}*/
}


/*! \fn void RobotnikTrajectoryPad::sendWSG50ReleaseAction()
 *  Sends a release action to the WSG50
*/
void RobotnikTrajectoryPad::sendWSG50ReleaseAction(){
	/*wsg_50_common::Move ac;
	
	ac.request.width = 110;
	ac.request.speed = 5.0;
	
	if(!wsg50_grasp_service_.call(ac)){
		ROS_ERROR("RobotnikTrajectoryPad::sendWSG50ReleaseAction: ERROR calling service to the wsg50");
	}*/
}


	
///////////////////////// MAIN /////////////////////////////////
int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotnik_trajectory_pad");
	RobotnikTrajectoryPad pad;
	
	pad.ControlLoop();
	
}

