#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    
    void costmapCallback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odometryCallback (const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback ();
    
};

#endif 
