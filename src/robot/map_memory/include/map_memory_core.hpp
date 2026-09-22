#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"


namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    void storingCostmap(nav_msgs::msg::OccupancyGrid msg);
    void odometryCheck(double current_x, double current_y);
    nav_msgs::msg::OccupancyGrid mergeCostmap();

  private:
    rclcpp::Logger logger_;
    double last_x_ {};
    double last_y_ {};
    double distance_threshold_ {1.5};
    nav_msgs::msg::OccupancyGrid global_map_;

    bool first_run_ = true;
    bool costmap_updated_ = false;
    nav_msgs::msg::OccupancyGrid current_map_;
    
};

}  

#endif  

