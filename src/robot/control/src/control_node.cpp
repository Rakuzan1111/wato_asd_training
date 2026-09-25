#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(get_logger()))
{
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg)
      { control_.current_path_ = msg; });

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      { control_.robot_odom_ = msg; });

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]()
      { controlLoop(); });
}

void ControlNode::controlLoop()
{
  auto twist_opt = control_.getTwist();
  if (twist_opt.has_value())
  {
    cmd_vel_pub_->publish(twist_opt.value());
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
