#include "map_memory_core.hpp"
#include <cmath>

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

// converts a cell in current_map_ into the matching cell in global_map_ by
// going through real-world coordinates, since current_map_'s origin now moves
// with the robot while global_map_'s origin stays fixed
bool MapMemoryCore::worldIndexFor(size_t current_index, int &global_index) const {
  int current_width {static_cast<int>(current_map_.info.width)};
  

  int local_x {static_cast<int>(current_index) % current_width};
  int local_y {static_cast<int>(current_index) / current_width};

  // convert the local cell to a real-world position using current_map_'s own origin
  double world_x {current_map_.info.origin.position.x + (local_x + 0.5) * current_map_.info.resolution};
  double world_y {current_map_.info.origin.position.y + (local_y + 0.5) * current_map_.info.resolution};

  // convert that world position into global_map_'s fixed frame
  int global_x {static_cast<int>(std::floor((world_x - global_origin_x_) / global_resolution_))};
  int global_y {static_cast<int>(std::floor((world_y - global_origin_y_) / global_resolution_))};

  // reject anything that falls outside the fixed global map
  if (global_x < 0 || global_x >= global_width_ || global_y < 0 || global_y >= global_height_) {
    return false;
  }

  global_index = global_y * global_width_ + global_x;
  return true;
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::mergeCostmap() {
  // set up global_map_'s fixed size/origin once, on the first run
  if (first_run_ == true) {
    first_run_ = false;

    global_map_.info.width = global_width_;
    global_map_.info.height = global_height_;
    global_map_.info.resolution = global_resolution_;
    global_map_.info.origin.position.x = global_origin_x_;
    global_map_.info.origin.position.y = global_origin_y_;
    global_map_.info.origin.orientation.w = 1.0;
    global_map_.data.assign(global_width_ * global_height_, -1);
  }

  /* -1 means "unknown" (e.g. out of the laser's current view). Only
     overwrite global_map_ where the current scan has real data, translating
     each cell through world coordinates since current_map_'s origin moves
     with the robot while global_map_'s origin stays fixed */
  for (size_t iii = 0; iii < current_map_.data.size(); ++iii) {
    if (current_map_.data[iii] != -1) {
      int global_index {};
      if (worldIndexFor(iii, global_index)) {
        global_map_.data[global_index] = current_map_.data[iii];
      }
    }
  }

  return global_map_;
}

}

