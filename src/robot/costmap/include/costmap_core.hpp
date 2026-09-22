#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal

    explicit CostmapCore(const rclcpp::Logger& logger);
    void initializeCostmap();
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    nav_msgs::msg::OccupancyGrid publishCostmap();

  private:
    rclcpp::Logger logger_;
    std::vector<std::vector<int>> grid_;
    int height_ {300};
    int width_ {300};
    double resolution_ {0.1};
    double inflation_radius_ {1.0};
    

};

}  

#endif  