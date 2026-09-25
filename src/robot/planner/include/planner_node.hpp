#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node
{
public:
    PlannerNode();

private:
    // plannerCore object
    robot::PlannerCore planner_;
    
    enum class State
    {
        WAITING_FOR_GOAL,
        WAITING_FOR_ROBOT_TO_REACH_GOAL
    };
    State state_ {State::WAITING_FOR_GOAL};

    // subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // stored data
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;

    // path data
    nav_msgs::msg::Path current_path_;
    bool has_path_ {false};
    
    // timer
    rclcpp::TimerBase::SharedPtr timer_;

    // callback declarations
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    bool map_received_ {false};
    bool goal_received_ {false};
    bool odom_received_ {false};

    bool goalReached() const;
    void planPath();

    // checks if the current path is still valid (no obstacles in the way)
    bool pathStillValid() const;
    
    // converts the world position to a grid position (meters to cells)
    robot::CellIndex worldToGrid(double worldX, double worldY) const;
    geometry_msgs::msg::PoseStamped gridToWorld(const robot::CellIndex& cell) const;
};

#endif
