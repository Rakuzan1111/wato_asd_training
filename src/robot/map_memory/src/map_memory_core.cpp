#include "map_memory_core.hpp"
namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

void MapMemoryCore::storingCostmap(nav_msgs::msg::OccupancyGrid msg) {
  costmap_updated_ = true;
  current_map_ = msg;
}

void MapMemoryCore::odometryCheck(double current_x, double current_y) {
  double distance_since_last_check {std::sqrt(std::pow(current_x - last_x_, 2) + std::pow(current_y - last_y_, 2))};
  
  
  if (distance_since_last_check > distance_threshold_) {
    last_x_ = current_x; 
    last_y_ = current_y;
  }
} 

nav_msgs::msg::OccupancyGrid MapMemoryCore::mergeCostmap() {
  if (first_run_ == true) {
    first_run_ = false;
    global_map_ = current_map_;
  }

  else {
    for (size_t iii = 0; iii < current_map_.data.size(); ++iii) {
      if (current_map_.data[iii] != -1) {
        global_map_.data[iii] = current_map_.data[iii];
      }
    }
  }
  return global_map_;
  }
}
