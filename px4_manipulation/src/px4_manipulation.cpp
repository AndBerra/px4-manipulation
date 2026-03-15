/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_avoidance.cpp
 *
 * px4 manipulation
 *
 */

#include "px4_manipulation/px4_manipulation.h"


Px4Manipulation::Px4Manipulation() : Node("minimal_publisher") {
    
	  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    kp_ = this->declare_parameter<double>("kp", kp_);
    kd_ = this->declare_parameter<double>("kd", kd_);

    // Publishers
    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_profile);
    vehicle_attitude_pub_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", qos_profile);
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_profile);    

    // Subscribers
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos_profile, std::bind(&Px4Manipulation::vehicleStatusCallback, this, std::placeholders::_1));
    vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos_profile, std::bind(&Px4Manipulation::vehicleAttitudeCallback, this, std::placeholders::_1));
    vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos_profile, std::bind(&Px4Manipulation::vehicleLocalPositionCallback, this, std::placeholders::_1));

    // Services      
    pose_service_ = this->create_service<manipulation_msgs::srv::SetPose>("/set_pose", std::bind(&Px4Manipulation::targetPoseCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Setup loop timers
    statusloop_timer_ = this->create_wall_timer(20ms, std::bind(&Px4Manipulation::statusloopCallback, this));
}



void Px4Manipulation::statusloopCallback() {

    // RCLCPP_INFO(this->get_logger(), "statusloopCallback()");

    // Simple PID position controller
    Eigen::Vector3d error_position = vehicle_position_ - reference_position_;

    Eigen::Vector3d hover_thrust_inertial(0.0, 0.0, 0.2);
    Eigen::Vector3d acceleration_feedback = -kp_ * error_position -kd_ * vehicle_velocity_;

    /// Compute attitude reference

    /// Compute thrust values
    Eigen::Vector3d thrust_inertial = acceleration_feedback + hover_thrust_inertial;
    thrust_inertial.array().max(1.0);    // Saturate thrust values
    thrust_inertial.array().min(-1.0);   
    auto R = vehicle_attitude_.normalized().toRotationMatrix();
    Eigen::Vector3d thrust_body = R.transpose() * thrust_inertial;
    
    // Publish offboard control mode
    px4_msgs::msg::OffboardControlMode offboard_ctrl_mode_msg;
    offboard_ctrl_mode_msg.position=false;
    offboard_ctrl_mode_msg.velocity=false;
    offboard_ctrl_mode_msg.attitude=true;
    offboard_ctrl_mode_msg.acceleration=false;
    offboard_mode_pub_->publish(offboard_ctrl_mode_msg);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", offboard_ctrl_mode_msg.data.c_str());

    // After ~1 second of streaming the heartbeat, switch to offboard and arm
    if (loop_counter_ < ARMING_TRIGGER_COUNT) {
        loop_counter_++;
    } else if (loop_counter_ == ARMING_TRIGGER_COUNT) {
        RCLCPP_INFO(this->get_logger(), "Switching to offboard mode and arming...");
        setOffboardMode();
        loop_counter_++;
    }

    // Publish attitude setpoints
    if (vehicle_nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      // RCLCPP_INFO(this->get_logger(), "publising attitude setpoint");

      px4_msgs::msg::VehicleAttitudeSetpoint attitude_setpoint_msg;
      attitude_setpoint_msg.q_d[0] = reference_attitude_.w();
      attitude_setpoint_msg.q_d[1] = reference_attitude_.x();
      attitude_setpoint_msg.q_d[2] = -reference_attitude_.y();
      attitude_setpoint_msg.q_d[3] = -reference_attitude_.z();
      attitude_setpoint_msg.thrust_body[0] = thrust_body(0);
      attitude_setpoint_msg.thrust_body[1] = -thrust_body(1);
      attitude_setpoint_msg.thrust_body[2] = -thrust_body(2);
      vehicle_attitude_pub_->publish(attitude_setpoint_msg);
    }
}

void Px4Manipulation::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus &msg) {
    if (msg.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
        vehicle_nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        reference_position_ = vehicle_position_;
        RCLCPP_INFO(this->get_logger(), "Offboard active! Holding ENU position: %.2f %.2f %.2f",
            vehicle_position_.x(), vehicle_position_.y(), vehicle_position_.z());
    }
      
    vehicle_nav_state_ = msg.nav_state;
    // RCLCPP_INFO(this->get_logger(), "Publishing: %f", double(vehicle_nav_state_));
}

void Px4Manipulation::publishVehicleCommand(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(msg);
}

void Px4Manipulation::setOffboardMode() {
    // param1=1 (custom mode enabled), param2=6 (PX4 offboard mode)
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
}

void Px4Manipulation::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude &msg) {
    ///TODO: Get vehicle attitude
    vehicle_attitude_.w() = msg.q[0];
    vehicle_attitude_.x() = msg.q[1];
    vehicle_attitude_.y() = -msg.q[2];
    vehicle_attitude_.z() = -msg.q[3];

}

void Px4Manipulation::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition &msg) {
    ///TODO: Get vehicle attitude
    vehicle_position_(0) = msg.x;
    vehicle_position_(1) = -msg.y;
    vehicle_position_(2) = -msg.z;
    vehicle_velocity_(0) = msg.vx;
    vehicle_velocity_(1) = -msg.vy;
    vehicle_velocity_(2) = -msg.vz;
}

void Px4Manipulation::targetPoseCallback(const std::shared_ptr<manipulation_msgs::srv::SetPose::Request> request,
          std::shared_ptr<manipulation_msgs::srv::SetPose::Response> response) {

  // RCLCPP_INFO(this->get_logger(), "targetPoseCallback()");

  reference_position_.x() = request->pose.position.y;
  reference_position_.y() = -request->pose.position.x;
  reference_position_.z() = request->pose.position.z;

  reference_attitude_.w() = request->pose.orientation.w;
  reference_attitude_.x() = request->pose.orientation.x;
  reference_attitude_.y() = request->pose.orientation.y;
  reference_attitude_.z() = request->pose.orientation.z;

  response->result = true;
}
