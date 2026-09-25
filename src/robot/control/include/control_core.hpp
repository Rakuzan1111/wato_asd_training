#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <optional>

class ControlNode;

namespace robot
{

  class ControlCore
  {

  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger &logger);

    std::optional<geometry_msgs::msg::Twist> getTwist();
    std::optional<geometry_msgs::msg::Point> getTarget();

    double lookahead_distance;
    double goal_tolerance;
    double linear_speed;

  private:
    friend class ::ControlNode;

    rclcpp::Logger logger_;
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;
  };

}

#endif
