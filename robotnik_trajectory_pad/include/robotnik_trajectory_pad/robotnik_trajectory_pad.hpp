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
 * \author Ángel Soriano
 * \brief Allows to use a pad with the robot controller, sending the messages received from the joystick device
 */

 #ifndef ROBOTNIK_TRAJECTORY_PAD_HPP
#define ROBOTNIK_TRAJECTORY_PAD_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <robotnik_trajectory_pad/CartesianEuler.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <std_msgs/Float64.h>

class RobotnikTrajectoryPad
{
public:
    // Constructor y destructor
    RobotnikTrajectoryPad(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~RobotnikTrajectoryPad() {};

    // Métodos públicos
    void Update();

    // Servicios
    bool srvSetAngleMode(std_srvs::SetBool::Request &request, 
                         std_srvs::SetBool::Response &response);
    bool srvSetDeadmanMode(std_srvs::SetBool::Request &request, 
                           std_srvs::SetBool::Response &response);

private:
    // Callbacks
    void padCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void muxCallback(const std_msgs::String::ConstPtr& msg);

    // Métodos internos de ayuda
    void processSpeedButtons(const sensor_msgs::Joy::ConstPtr& joy);
    void publishCartesianMsg(const sensor_msgs::Joy::ConstPtr& joy);

    // Atributos ROS
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher pad_pub_;
    ros::Publisher pad_vel_pub_;
    ros::Subscriber pad_sub_;
    ros::Subscriber mux_sub_;

    // Services
    ros::ServiceServer srv_set_angle_mode_;
    ros::ServiceServer srv_set_deadman_mode_;

    // Parámetros
    int num_of_buttons_;
    int linear_x_, linear_y_, linear_z_, angular_;
    double l_scale_, a_scale_, l_scale_z_;
    double current_step_;

    // Botones y flags
    int dead_man_button_;
    int speed_up_button_, speed_down_button_;
    int button_euler_mode_, button_angle_deadman_;
    int button_joint_deadman_;
    bool angle_A_mode_; 
    bool deadMan_mode_;
    bool bEnable;
    bool last_command_;
    bool itowa_pad_;

    // Otras variables
    bool bRegisteredButtonEvent[16];  // Ejemplo, ajusta a tus necesidades
    bool bRegisteredDirectionalArrows[4];

    double max_axis_step_;
    double max_joint_step_;

    // Diagnósticos
    diagnostic_updater::Updater updater_pad_;
    diagnostic_updater::HeaderlessTopicDiagnostic* pub_command_freq_;
    diagnostic_updater::HeaderlessTopicDiagnostic* sus_joy_freq_;
    double min_freq_command_, max_freq_command_;
    double min_freq_joy_, max_freq_joy_;

    // Otros nombres de topics
    std::string cartesian_topic_name_;
    std::string joint_topic_name_;
    std::string pad_type_;

};

#endif // ROBOTNIK_TRAJECTORY_PAD_HPP
