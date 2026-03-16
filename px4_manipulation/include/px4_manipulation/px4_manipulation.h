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

#include <chrono>
#include <functional>
#include <memory>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/position_setpoint_triplet.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "manipulation_msgs/srv/set_pose.hpp"
#include "manipulation_msgs/srv/set_waypoints.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Px4Manipulation : public rclcpp::Node
{
  public:
    Px4Manipulation();

  private:
    /**
     * @brief Status loop for running decisions
     * 
     */
    void statusloopCallback();

    
    /**
     * @brief vehicle status callback from px4
     * 
     * @param msg 
     */
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus &msg);

    /**
     * @brief Callback for vehicle attitude
     * 
     * @param msg 
     */
    void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude &msg);

    /**
     * @brief Callback for vehicle local position
     * 
     * @param msg 
     */
    void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition &msg);

    /**
     * @brief Callback for target vehicle pose service
     * 
     * @param request 
     * @param response 
     */
    void targetPoseCallback(const std::shared_ptr<manipulation_msgs::srv::SetPose::Request> request,
          std::shared_ptr<manipulation_msgs::srv::SetPose::Response> response);

    /** @brief 
     * Callback to receive full waypoint list and start sequencing 
    */
    void setWaypointsCallback(
        const std::shared_ptr<manipulation_msgs::srv::SetWaypoints::Request> request,
        std::shared_ptr<manipulation_msgs::srv::SetWaypoints::Response> response);
           
    /**
     * @brief Publish a VehicleCommand to PX4
     * @param command  MAVLink command ID
     * @param param1   First parameter (default 0.0)
     * @param param2   Second parameter (default 0.0)
     */
    void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
 
    /**
     * @brief Switch to offboard mode
     */
    void setOffboardMode();

    /** @brief 
     * Check if drone is close enough to current waypoint and advance 
     */
    void updateWaypointSequencing();
 
    /** @brief 
     * Load waypoints from JSON file and start sequencing 
     */
    void loadWaypointsFromFile(const std::string & path);

    // Timers
    rclcpp::TimerBase::SharedPtr statusloop_timer_;
    
    // Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    
    // Subscribers    
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    
    // Services
    rclcpp::Service<manipulation_msgs::srv::SetPose>::SharedPtr pose_service_;
    rclcpp::Service<manipulation_msgs::srv::SetWaypoints>::SharedPtr waypoints_service_;

    // Vehicle state
    uint8_t vehicle_nav_state_{0};
    Eigen::Quaterniond vehicle_attitude_{Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)};
    Eigen::Vector3d vehicle_position_{Eigen::Vector3d(0.0, 0.0, 0.0)};
    Eigen::Vector3d vehicle_velocity_{Eigen::Vector3d(0.0, 0.0, 0.0)};

    // Reference setpoints    
    Eigen::Vector3d reference_position_{Eigen::Vector3d(0.0, 0.0, 10.0)};
    Eigen::Quaterniond reference_attitude_{Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)};

    // Waypoint list populated by /set_waypoints
    std::vector<Eigen::Vector3d> waypoints_;
    std::vector<Eigen::Quaterniond> waypoint_attitudes_;
 
    // Index of current wp
    int current_waypoint_idx_{-1};
 
    // True when waypoint sequencing
    bool waypoint_sequencing_running_{false};
 
    // Distance threshold to consider a waypoint reached (meters)
    static constexpr double WAYPOINT_ACCEPTANCE_RADIUS{0.3};

    // Controller gains    
    double kp_{0.05};
    double kd_{0.05};

    // If true, load waypoints from file on startup and start sequencing 
    bool follow_waypoints_{false};
 
    // Path to waypoints JSON file
    std::string waypoints_path_{""};

    // Counter for delayed arm/offboard trigger
    int loop_counter_{0};
    static constexpr int ARMING_TRIGGER_COUNT{50};    
};
