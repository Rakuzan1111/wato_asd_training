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
    bool hasReceivedCostmap(); // returns whether a real costmap has ever been received yet

  private:
    rclcpp::Logger logger_;
    double last_x_ {};
    double last_y_ {};
    double distance_threshold_ {1.5};          // distance (m) before resetting position check
    nav_msgs::msg::OccupancyGrid global_map_; 

    /* true until first merge so that mergeCostmap() copies the whole map instead of 
    merging into it the first run */
    bool first_run_ = true; 
                      
    bool costmap_updated_ = false;
    nav_msgs::msg::OccupancyGrid current_map_;

    // fixed size/resolution/origin for global_map_, decided once so every merge
    // lines up with the same real-world frame no matter where the robot is
    static constexpr int global_width_ {600};
    static constexpr int global_height_ {600};
    static constexpr double global_resolution_ {0.1};
    static constexpr double global_origin_x_ {-30.0};
    static constexpr double global_origin_y_ {-30.0};

    // converts a cell index inside current_map_ into global_map_'s matching cell index,
    // using each map's own origin/resolution so they line up in real-world space
    bool worldIndexFor(size_t current_index, int &global_index) const;
    
};

}  

#endif  

