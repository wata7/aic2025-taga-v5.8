#ifndef SIMPLE_PURE_PURSUIT_HPP_
#define SIMPLE_PURE_PURSUIT_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace simple_pure_pursuit {

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

class SimplePurePursuit : public rclcpp::Node {
 public:
  explicit SimplePurePursuit();
  
  // subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  
  // publishers
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_raw_cmd_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_lookahead_point_;  
  
  // timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // updated by subscribers
  Trajectory::SharedPtr trajectory_;
  Odometry::SharedPtr odometry_;
  
  // pure pursuit parameters
  // ★★★ const を削除して動的変更可能に ★★★
  double wheel_base_;
  double lookahead_gain_;
  double lookahead_min_distance_;
  double speed_proportional_gain_;
  bool use_external_target_vel_;
  double external_target_vel_;
  double steering_tire_angle_gain_;
  
 private:
  void onTimer();
  bool subscribeMessageAvailable();
  
  // ★★★ パラメータ変更時のコールバック（追加）★★★
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);
  
  // ★★★ コールバックハンドル（追加）★★★
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace simple_pure_pursuit

#endif  // SIMPLE_PURE_PURSUIT_HPP_
