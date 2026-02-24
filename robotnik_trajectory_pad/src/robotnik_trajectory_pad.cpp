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

#include <robotnik_trajectory_pad/robotnik_trajectory_pad.hpp>

#include <std_msgs/Int32.h> // si hace falta
#include <std_msgs/Bool.h>
#include <unistd.h>

RobotnikTrajectoryPad::RobotnikTrajectoryPad(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , angle_A_mode_(true)
    , deadMan_mode_(true)
    , bEnable(false)
    , last_command_(true)
{
    // Lectura de parámetros
    pnh_.param("pad_type", pad_type_, std::string("ps3"));
    pnh_.param("num_of_buttons", num_of_buttons_, 16);

    pnh_.param("axis_linear_x", linear_x_, 1);
    pnh_.param("axis_linear_y", linear_y_, 1);
    pnh_.param("axis_linear_z", linear_z_, 1);
    pnh_.param("axis_angular", angular_, 1);

    pnh_.param("scale_angular", a_scale_, 0.05);
    pnh_.param("scale_linear", l_scale_, -1.0);
    pnh_.param("scale_linear_z", l_scale_z_, 1.0);

    pnh_.param("cartesian_topic_name", cartesian_topic_name_, std::string("cartesian_move"));
    pnh_.param("joint_topic_name", joint_topic_name_, std::string("joint_move"));

    pnh_.param("button_dead_man", dead_man_button_, 0);
    pnh_.param("button_speed_up", speed_up_button_, 4);
    pnh_.param("button_speed_down", speed_down_button_, 5);
    pnh_.param("button_euler_mode", button_euler_mode_, 0);
    pnh_.param("button_angle_deadman", button_angle_deadman_, 0);
    pnh_.param("button_joint_mode", button_joint_deadman_, 0);

    pnh_.param("max_axis_step", max_axis_step_, 0.1);
    pnh_.param("max_joint_step", max_joint_step_, 0.1);

    current_step_ = 0.10; // Por ejemplo
    
    itowa_pad_ = false;

    // Inicialización arrays de botones
    for(int i = 0; i < 16; ++i) {
        bRegisteredButtonEvent[i] = false;
    }
    for(int i = 0; i < 4; ++i) {
        bRegisteredDirectionalArrows[i] = false;
    }

    // Publisher
    pad_pub_ = nh_.advertise<robotnik_trajectory_pad::CartesianEuler>(cartesian_topic_name_, 1);
    pad_vel_pub_ = nh.advertise<std_msgs::Float64>("pad_vel", 10);

    // Subscriber
    pad_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RobotnikTrajectoryPad::padCallback, this);
    mux_sub_ = nh_.subscribe<std_msgs::String>("/mux_joy/selected", 1, &RobotnikTrajectoryPad::muxCallback, this);

    // Services
    srv_set_angle_mode_ = nh_.advertiseService("/kuka_pad/set_angle_mode", 
                                               &RobotnikTrajectoryPad::srvSetAngleMode, this);
    srv_set_deadman_mode_ = nh_.advertiseService("/kuka_pad/set_deadman_mode", 
                                                 &RobotnikTrajectoryPad::srvSetDeadmanMode, this);

    // Diagnostics
    updater_pad_.setHardwareID("None");

    min_freq_command_ = min_freq_joy_ = 5.0;
    max_freq_command_ = max_freq_joy_ = 50.0;

    sus_joy_freq_ = new diagnostic_updater::HeaderlessTopicDiagnostic("/joy", updater_pad_,
                           diagnostic_updater::FrequencyStatusParam(&min_freq_joy_, &max_freq_joy_, 0.1, 10));
    pub_command_freq_ = new diagnostic_updater::HeaderlessTopicDiagnostic(cartesian_topic_name_.c_str(), updater_pad_,
                           diagnostic_updater::FrequencyStatusParam(&min_freq_command_, &max_freq_command_, 0.1, 10));

    std_msgs::Float64 vel_msg;
    vel_msg.data = current_step_ * 500;
    pad_vel_pub_.publish(vel_msg);
}

void RobotnikTrajectoryPad::Update()
{
    updater_pad_.update();
}

void RobotnikTrajectoryPad::padCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // Marca la recepción de un mensaje joy para diagnóstico
    sus_joy_freq_->tick();

    // Determina si se activa el control (dead man)
    if(deadMan_mode_) {
        bEnable = (joy->buttons[dead_man_button_] == 1);
    } else if(!deadMan_mode_ && joy->buttons[button_euler_mode_] == 0) {
        bEnable = true;
    }

    // Procesa los botones de subida/bajada de velocidad
    if(bEnable) {
        processSpeedButtons(joy);
    }

    // Publica el mensaje cartesian euler correspondiente
    publishCartesianMsg(joy);

    // Si se habilita, se hace tick de frecuencia de publicación
    if(bEnable) {
        pub_command_freq_->tick();
        last_command_ = true;
    } else if(!bEnable && last_command_) {
        // Publica un mensaje de "parada" si veníamos publicando
        robotnik_trajectory_pad::CartesianEuler cartesian_msg;
        cartesian_msg.x = 0.0;
        cartesian_msg.y = 0.0;
        cartesian_msg.z = 0.0;
        cartesian_msg.pitch = 0.0;
        cartesian_msg.roll = 0.0;
        cartesian_msg.yaw = 0.0;
        pad_pub_.publish(cartesian_msg);

        pub_command_freq_->tick();
        last_command_ = false;
    }
}

void RobotnikTrajectoryPad::muxCallback(const std_msgs::String::ConstPtr& msg)
{
    // Determina si el itowa está activo
    if(msg->data == "/kuka_pad/itowa_joy") {
        itowa_pad_ = true;
    } else {
        itowa_pad_ = false;
    }
}

void RobotnikTrajectoryPad::processSpeedButtons(const sensor_msgs::Joy::ConstPtr& joy)
{
    std_msgs::Float64 msg;
    
    // SPEED DOWN
    if (joy->buttons[speed_down_button_] == 1) {
        if(!bRegisteredButtonEvent[speed_down_button_]) {
            if(current_step_ >= 0.10) {
                current_step_ -= 0.05;
                bRegisteredButtonEvent[speed_down_button_] = true;
                ROS_INFO("Decreasing step: %.1f%%", current_step_ * 500);
            }
        }
    } else {
        bRegisteredButtonEvent[speed_down_button_] = false;
    }

    // SPEED UP
    if (joy->buttons[speed_up_button_] == 1) {
        if(!bRegisteredButtonEvent[speed_up_button_]) {
            if(current_step_ < 0.2 ) {
                current_step_ += 0.05;
                bRegisteredButtonEvent[speed_up_button_] = true;
                ROS_INFO("Increasing step: %.1f%%", current_step_ * 500);
            }
        }
    } else {
        bRegisteredButtonEvent[speed_up_button_] = false;
    }
    std_msgs::Float64 vel_msg;
    vel_msg.data = current_step_ * 500;
    pad_vel_pub_.publish(vel_msg);

}

void RobotnikTrajectoryPad::publishCartesianMsg(const sensor_msgs::Joy::ConstPtr& joy)
{
    robotnik_trajectory_pad::CartesianEuler cartesian_msg;

    // Inicializamos a 0
    cartesian_msg.x = 0.0;
    cartesian_msg.y = 0.0;
    cartesian_msg.z = 0.0;
    cartesian_msg.pitch = 0.0;
    cartesian_msg.roll = 0.0;
    cartesian_msg.yaw = 0.0;

    // Solo si bEnable es true o tenemos algún otro modo activo
    if(bEnable) {
        if(joy->buttons[button_angle_deadman_] == 1 && angle_A_mode_) {
            // Modo Euler “angle A”
            cartesian_msg.yaw = 0.06 * a_scale_ * joy->axes[angular_];
        } else {
            // Modo cartesiano
            if(itowa_pad_ && joy->buttons[button_euler_mode_] == 1){

            }else{
                cartesian_msg.x = current_step_ * l_scale_ * joy->axes[linear_x_];
                cartesian_msg.y = current_step_ * l_scale_ * joy->axes[linear_y_];
                cartesian_msg.z = current_step_ * l_scale_z_ * joy->axes[linear_z_];
            }
        }
    }

    pad_pub_.publish(cartesian_msg);
}

bool RobotnikTrajectoryPad::srvSetAngleMode(std_srvs::SetBool::Request &request,
                                            std_srvs::SetBool::Response &response)
{
    angle_A_mode_ = request.data;
    response.success = true;
    std::string status = (angle_A_mode_ ? "Enabled" : "Disabled");
    ROS_INFO("Angle mode: %s", status.c_str());
    return true;
}

bool RobotnikTrajectoryPad::srvSetDeadmanMode(std_srvs::SetBool::Request &request,
                                              std_srvs::SetBool::Response &response)
{
    deadMan_mode_ = request.data;
    response.success = true;
    std::string status = (deadMan_mode_ ? "Enabled" : "Disabled");
    ROS_INFO("DeadMan mode: %s", status.c_str());
    return true;
}
