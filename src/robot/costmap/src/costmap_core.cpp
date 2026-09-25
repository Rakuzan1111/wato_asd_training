#include "costmap_core.hpp"
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

// resets the grid to all 0 before each new scan
void CostmapCore::initializeCostmap() {
    grid_.assign(height_, std::vector<int>(width_, 0));
}

void CostmapCore::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
    // convert distance + direction into x/y position in meters.
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

     // convert meters into grid cells by dividing by cell size (resolution_).
    x_grid = static_cast<int> (x / resolution_);
    y_grid = static_cast<int> (y / resolution_);
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
    // bounds check so an out-of-range reading can't crash the program.
    if (x_grid < width_ && x_grid >= 0 && y_grid >= 0 && y_grid < height_) {
        grid_[y_grid][x_grid] = 100;
    }

}

void CostmapCore::inflateObstacles() {
    // Scan every cell in the grid.
    for (int iii = 0; iii < height_; ++iii) {

        for (int jjj = 0; jjj < width_; ++jjj) {
            
            // spread inflation outward from confirmed obstacle cells.
            if (grid_[iii][jjj] == 100 ) {
                int radius_in_cells = static_cast<int>(inflation_radius_ / resolution_);

                // scan a square region around this obstacle.
                for (int dy = -radius_in_cells; dy <= radius_in_cells; ++dy) {

                     for (int dx = -radius_in_cells; dx <= radius_in_cells; ++dx) {

                        int neighboring_y = iii + dy;
                        int neighboring_x = jjj + dx;

                        if (neighboring_x >= 0 && neighboring_x < width_ && neighboring_y >= 0 && neighboring_y < height_) {
                            // real-world distance to this neighbor, used to filter the square scan area down to a circle.
                           double distance {std::sqrt(std::pow(dy, 2) + std::pow(dx, 2)) * resolution_};

                           // calculate inflation value based on distance from obstacle
                           int cost {static_cast<int>(100 * (1 - distance/inflation_radius_))};

                           /*Only overwrite current cell value if within the radius and 
                           higher than the existing value, so a farther obstacle
                           can't override a closer one's inflation. */
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

nav_msgs::msg::OccupancyGrid CostmapCore::publishCostmap(double robot_x, double robot_y) {
    nav_msgs::msg::OccupancyGrid msg; 
    msg.info.resolution = resolution_;
    msg.info.width = width_;
    msg.info.height = height_;

     // anchor grid (0,0) to the robot's actual world position, offset so the
    // robot sits at the center of the grid instead of the corner
    msg.info.origin.position.x = robot_x - (width_ * resolution_) / 2.0;
    msg.info.origin.position.y = robot_y - (height_ * resolution_) / 2.0;
    msg.info.origin.orientation.w = 1.0;

    // flatten the 2D grid into a 1D array row by row.
    for (int iii = 0; iii < height_; ++iii) {

        for (int jjj = 0; jjj < width_; ++jjj) {

            msg.data.push_back(grid_[iii][jjj]);

        }
    }
return msg;
}

}