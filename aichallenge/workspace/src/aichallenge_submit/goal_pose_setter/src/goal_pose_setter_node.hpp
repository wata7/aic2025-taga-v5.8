// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GOAL_POSE_SETTER_NODE_
#define GOAL_POSE_SETTER_NODE_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

class GoalPosePublisher : public rclcpp::Node
{
public:
    GoalPosePublisher();

private:
    void on_timer();

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ekf_trigger_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    geometry_msgs::msg::PoseStamped goal_pose_;
    
    bool stop_initializing_pose_ = false;
    int delay_count_ = 0;
};

#endif  // GOAL_POSE_SETTER_NODE_
