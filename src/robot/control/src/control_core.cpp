#include "control_core.hpp"
#include <cmath>
#include <cstddef>

namespace
{
  double distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
  {
    auto dx = b.x - a.x;
    auto dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double yaw(const geometry_msgs::msg::Quaternion &q)
  {
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }
}

namespace robot
{

  ControlCore::ControlCore(const rclcpp::Logger &logger)
      : lookahead_distance(1.0), goal_tolerance(0.5), linear_speed(1), logger_(logger) {}

  std::optional<geometry_msgs::msg::Twist> ControlCore::getTwist()
  {
    if (!robot_odom_ || !current_path_)
    {
      if (!robot_odom_)
        RCLCPP_WARN(logger_, "No odometry received yet.");
      if (!current_path_)
        RCLCPP_WARN(logger_, "No path received yet.");

      return std::nullopt;
    }

    auto target_pos_opt = getTarget();
    if (!target_pos_opt)
    {
      RCLCPP_INFO(logger_, "No target found, stopping robot.");
      return geometry_msgs::msg::Twist();
    }

    auto target_pos = target_pos_opt.value();
    auto pos = robot_odom_->pose.pose.position;
    auto target_yaw = std::atan2(target_pos.y - pos.y, target_pos.x - pos.x);

    auto orientation = robot_odom_->pose.pose.orientation;
    auto yaw = std::atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));

    // pure pursuit doesn't handle 180 turns very well
    // so I'm handling them another way
    auto twist = geometry_msgs::msg::Twist();
    // clamp angular velocity to prevent lidar desync
    constexpr double ANGULAR_VEL_MAX = 1.5;
    if (std::cos(target_yaw - yaw) > 0)
    {
      double angular_vel = 2.0 * linear_speed * std::sin(target_yaw - yaw) / distance(pos, target_pos);
      angular_vel = std::max(-ANGULAR_VEL_MAX, std::min(ANGULAR_VEL_MAX, angular_vel));
      twist.set__angular(geometry_msgs::msg::Vector3().set__z(angular_vel));
      RCLCPP_INFO(logger_, "Target found, moving robot. Angular velocity: %f", angular_vel);
      twist.set__linear(geometry_msgs::msg::Vector3().set__x(linear_speed));
    }
    else
    {
      double angular_vel = ANGULAR_VEL_MAX * ((target_yaw - yaw > 0) ? 1 : -1);
      angular_vel = std::max(-ANGULAR_VEL_MAX, std::min(ANGULAR_VEL_MAX, angular_vel));
      twist.set__angular(geometry_msgs::msg::Vector3().set__z(std::fmod(target_yaw - yaw + M_PI, 2 * M_PI) - M_PI));
    }
    return twist;
  }

  /// Assumes odometry and path are not null
  std::optional<geometry_msgs::msg::Point> ControlCore::getTarget()
  {
    auto pos = robot_odom_->pose.pose.position;
    if (distance(pos, current_path_->poses.back().pose.position) < goal_tolerance)
    {
      return std::nullopt;
    }
    for (std::size_t i = current_path_->poses.size(); i --> 0;)
    {
      const auto &pose_stamped = current_path_->poses[i];
      auto target_pos = pose_stamped.pose.position;
      if (distance(pos, target_pos) < lookahead_distance)
      {
        return target_pos;
      }
    }
    return std::nullopt;
  }
}