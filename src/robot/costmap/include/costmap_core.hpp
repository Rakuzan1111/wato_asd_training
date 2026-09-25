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
    nav_msgs::msg::OccupancyGrid publishCostmap(double robot_x, double robot_y);

  private:
    rclcpp::Logger logger_;
    
    // all the used variables for the costmap
    std::vector<std::vector<int>> grid_;

    // the grid is 300 cells tall and wide, with each cell being 0.1m in real space
    int height_ {300};
    int width_ {300};
    double resolution_ {0.1};

    
    double inflation_radius_ {1.0};  // obstacles inflated by 1.0m
    

};

}  

#endif  
