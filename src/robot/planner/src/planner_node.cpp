#include <cmath>
#include <functional>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode()
    : Node("planner_node"),
      planner_(this->get_logger())
{
    // subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
 
    // publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
 
    // timer (check if robot reached goal / needs replanning)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), std::bind(&PlannerNode::timerCallback, this));
}

// confirming the map is received and makes an initial replan if it changes
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{

    current_map_ = *msg;
    map_received_ = true;
    
    /*
    // replan whenever the global map changes
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
    {
        planPath();
    }
    */
}

// confirming the robot's goal is received
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    goal_ = *msg;
    goal_received_ = true;

    state_ =
        State::WAITING_FOR_ROBOT_TO_REACH_GOAL;

    RCLCPP_INFO
    (
        this->get_logger(),
        "Received goal: x=%.2f y=%.2f",
        goal_.point.x,
        goal_.point.y
    );

    // create an initial plan based on the goal just received
    planPath();
}

// confirming the robot's current position
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_pose_ = msg->pose.pose;
    odom_received_ = true;
}

// create "ticks" at which the robot plans its route
void PlannerNode::timerCallback()
{
    // prevent useless A* calculations when the robot has no intended destination
    if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
    {
        return;
    }

    if (goalReached())
    {
        RCLCPP_INFO
        (
            this->get_logger(),
            "Goal reached!"
        );

        // finished navigation
        state_ = State::WAITING_FOR_GOAL;
        goal_received_ = false;

        return;
    }

    // replan if and only if the robot is navigating but hasn't reached its goal
    planPath();
}

// checks if the robot reached the goal
bool PlannerNode::goalReached() const
{
    // make sure we received the goal and the robot's current position
    if (!goal_received_ || !odom_received_)
    {
        return false;
    }

    double dx {goal_.point.x - robot_pose_.position.x};

    double dy {goal_.point.y - robot_pose_.position.y};

    // distance away by calculating the hypothenuse 
    double distance {std::sqrt(dx * dx + dy * dy)};

    // true if reached, false if not
    return distance < 0.5;
}

// converts the world position to a grid position (meters to cells)
robot::CellIndex PlannerNode::worldToGrid(
    double worldX,
    double worldY) const
{
    // size of one cell
    double resolution {current_map_.info.resolution};

    // what is the map's grid origin
    double originX {current_map_.info.origin.position.x};
    double originY {current_map_.info.origin.position.y};

    // conversion from world size to grid size 
    int gridX {static_cast<int>(std::floor((worldX - originX) / resolution))};
    int gridY {static_cast<int>(std::floor((worldY - originY) / resolution))};

    return robot::CellIndex {gridX, gridY};
}

// converts from a grid position to a world position (cells to meters)
geometry_msgs::msg::PoseStamped
PlannerNode::gridToWorld(
    const robot::CellIndex &cell) const
{
    geometry_msgs::msg::PoseStamped pose {};

    double resolution {current_map_.info.resolution};

    double originX {current_map_.info.origin.position.x};
    double originY {current_map_.info.origin.position.y};

    // use the center of each occupancy-grid cell
    pose.pose.position.x = originX + (static_cast<double>(cell.x) + 0.5) * resolution;

    pose.pose.position.y = originY + (static_cast<double>(cell.y) + 0.5) * resolution;

    pose.pose.position.z = 0.0;

    // Identity orientation
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    return pose;
}

// using the A* to plan the optimal path
void PlannerNode::planPath()
{
    // making sure we can run this function
    if(!map_received_ || !goal_received_ || !odom_received_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Cannot plan path: missing map, goal or odometry."
        );
        return;
    }

    if (current_map_.data.empty())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Cannot plan: map is empty."
        );
        return;
    }

    // size of the map
    int width {static_cast<int>(current_map_.info.width)};
    int height {static_cast<int>(current_map_.info.height)};

    // creating start and goal coordinates (by grid cell units)
    robot::CellIndex start 
    {
        worldToGrid
        (
            robot_pose_.position.x,
            robot_pose_.position.y
        )
    };
    robot::CellIndex goal {
        worldToGrid(
            goal_.point.x,
            goal_.point.y)
    };

    // logging how the plan is going
    RCLCPP_INFO(
        this->get_logger(),
        "Planning from grid (%d,%d) to (%d,%d)",
        start.x,
        start.y,
        goal.x,
        goal.y);

    // calling the aStar function in planner_core to generate us a path using A* algorithm
    std::vector<robot::CellIndex> gridPath {
        planner_.aStar(
            current_map_.data,
            width,
            height,
            start,
            goal)
    };

    // checks if the path was generated
    if (gridPath.empty())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "A* could not find a path.");

        return;
    }

    // store the path 
    nav_msgs::msg::Path path {};

    // create a header and timestamp for the path created
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = current_map_.header.frame_id; // every point uses the same coordinate frame as the map
    path.poses.reserve(gridPath.size());

    // convert every cell into actual units
    for (const robot::CellIndex &cell : gridPath)
    {
        geometry_msgs::msg::PoseStamped pose {gridToWorld(cell)};

        pose.header = path.header;

        path.poses.push_back(pose);
    }

    // confirming the path has been created (and with how many nodes)
    RCLCPP_INFO(
        this->get_logger(),
        "Publishing path with %zu waypoints.",
        path.poses.size()
    );
    
    // send the path to ROS
    path_pub_->publish(path);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
      std::make_shared<PlannerNode>());

  rclcpp::shutdown();

  return 0;
}