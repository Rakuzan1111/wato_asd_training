#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "costmap_core.hpp"
#include "nav_msgs/msg/odometry.hpp"


class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    


  private:
    robot::CostmapCore costmap_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    double robot_x_ {0.0};
    double robot_y_ {0.0};
};

#endif