#include <chrono>
#include <memory>
#include <cmath>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  occupancy_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&CostmapNode::odomCallback, this, std::placeholders::_1));

}

void CostmapNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

      /* Pull the robot's heading (yaw) out of the orientation quaternion.
       This is the standard quaternion -> yaw formula; we only care about
       rotation about the vertical axis since the robot drives on flat ground. */
    const auto &q = msg->pose.pose.orientation;
    robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}


void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 1: Initialize costmap
    costmap_.initializeCostmap();

    // Step 2: Convert LaserScan to grid and mark obstacles
     for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        // skip inf/NaN readings (a beam that hit nothing) as well as out-of-spec ranges
        if (std::isfinite(range) && range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            int x_grid, y_grid;

            /* Add the robot's heading to the beam angle. The lidar reports angles
               relative to the robot's nose, but our grid is aligned to the world
               (the published origin has no rotation). Without this, every obstacle
               rotates around the robot as it turns, which smears the remembered
               map into arcs instead of building up stable walls. */
            costmap_.convertToGrid(range, angle + robot_yaw_, x_grid, y_grid);
            costmap_.markObstacle(x_grid, y_grid);
        }
    }

    // Step 3: Inflate obstacles
    costmap_.inflateObstacles();

    // Step 4: Publish costmap
    nav_msgs::msg::OccupancyGrid msg {costmap_.publishCostmap(robot_x_, robot_y_)};
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "sim_world";
    occupancy_pub_->publish(msg);
}

 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}