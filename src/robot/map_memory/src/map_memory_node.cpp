#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odometryCallback, this, std::placeholders::_1));
  global_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::timerCallback, this));
}
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // dereference the shared pointer since storingCostmap expects an actual message, not a pointer
  map_memory_.storingCostmap(*msg);
}

void MapMemoryNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  map_memory_.odometryCheck(msg->pose.pose.position.x, msg->pose.pose.position.y);
  
}

void MapMemoryNode::timerCallback() {
  /* skip this cycle if no costmap has arrived yet — otherwise mergeCostmap()
     runs on an empty current_map_ before its size matches global_map_,
     which was causing a segfault on startup */
  if (!map_memory_.hasReceivedCostmap()) {
    return;  // nothing to merge yet, skip this cycle
  }
   // get the merged map, timestamp it, and publish it on /map for the planner node.
  nav_msgs::msg::OccupancyGrid msg {map_memory_.mergeCostmap()};
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";
    global_map_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
