#include "map_memory_core.hpp"
namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

/* tells the caller whether storingCostmap() has run at least once, so
   mergeCostmap() isn't called on an empty, uninitialized current_map_ */
bool MapMemoryCore::hasReceivedCostmap() {
  return costmap_updated_;
}

// save the latest costmap and flag that new data arrived
void MapMemoryCore::storingCostmap(nav_msgs::msg::OccupancyGrid msg) {
  costmap_updated_ = true;
  current_map_ = msg;
}

// calculate how far the robot has moved since last check using distance formula
void MapMemoryCore::odometryCheck(double current_x, double current_y) {
  double distance_since_last_check {std::sqrt(std::pow(current_x - last_x_, 2) + std::pow(current_y - last_y_, 2))};
  
  // if moved far enough, change saved position
  if (distance_since_last_check > distance_threshold_) {
    last_x_ = current_x; 
    last_y_ = current_y;
  }
} 

nav_msgs::msg::OccupancyGrid MapMemoryCore::mergeCostmap() {
  // set global_map_ to current map on the first run since there's nothing to merge into it yet
  if (first_run_ == true) {
    first_run_ = false;
    global_map_ = current_map_;
  }

  else {
    /* -1 means "unknown" (e.g. out of the laser's current view). Only
     overwrite global_map_ where the current scan has real data */

    for (size_t iii = 0; iii < current_map_.data.size(); ++iii) {
      if (current_map_.data[iii] != -1) {
        global_map_.data[iii] = current_map_.data[iii];
      }
    }
  }
  return global_map_;
  }
}
