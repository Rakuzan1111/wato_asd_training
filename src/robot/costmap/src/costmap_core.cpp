#include "costmap_core.hpp"
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}
void CostmapCore::initializeCostmap() {
    grid_.assign(height_, std::vector<int>(width_, 0));
}

void CostmapCore::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);
    x_grid = static_cast<int> (x / resolution_);
    y_grid = static_cast<int> (y / resolution_);
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
    if (x_grid < width_ && x_grid >= 0 && y_grid >= 0 && y_grid < height_) {
        grid_[y_grid][x_grid] = 100;
    }

}

void CostmapCore::inflateObstacles() {
    for (int iii = 0; iii < height_; ++iii) {

        for (int jjj = 0; jjj < width_; ++jjj) {

            if (grid_[iii][jjj] == 100 ) {
                int radius_in_cells = static_cast<int>(inflation_radius_ / resolution_);

                for (int dy = -radius_in_cells; dy <= radius_in_cells; ++dy) {

                     for (int dx = -radius_in_cells; dx <= radius_in_cells; ++dx) {

                        int neighboring_y = iii + dy;
                        int neighboring_x = jjj + dx;

                        if (neighboring_x >= 0 && neighboring_x < width_ && neighboring_y >= 0 && neighboring_y < height_) {
                           double distance {std::sqrt(std::pow(dy, 2) + std::pow(dx, 2)) * resolution_};
                           int cost {static_cast<int>(100 * (1 - distance/inflation_radius_))};

                           if (distance <= inflation_radius_ && cost > grid_[neighboring_y][neighboring_x]) {
                                grid_[neighboring_y][neighboring_x] = cost;
                           }
                        }
                    }
                }
            }

        }

    }


}

nav_msgs::msg::OccupancyGrid CostmapCore::publishCostmap() {
    nav_msgs::msg::OccupancyGrid msg; 
    msg.info.resolution = resolution_;
    msg.info.width = width_;
    msg.info.height = height_;
    for (int iii = 0; iii < height_; ++iii) {

        for (int jjj = 0; jjj < width_; ++jjj) {

            msg.data.push_back(grid_[iii][jjj]);

        }
    }
return msg;
}

}