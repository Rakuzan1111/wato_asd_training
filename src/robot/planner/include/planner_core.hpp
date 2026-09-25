#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <array>
#include <cstdint>
#include <functional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace robot
{

// represents a cell in the occupancy grid by its x and y coordinates
struct CellIndex 
{
    int x;
    int y;

    bool operator==(const CellIndex& other) const
    {
      return x == other.x && y == other.y;
    }

    bool operator!=(const CellIndex &other) const
    {
        return (x != other.x || y != other.y);
    }

};

// hash function for CellIndex to be used in unordered_map and unordered_set
struct CellIndexHash
{
    std::size_t operator()(const CellIndex& cell) const
    {
        return std::hash<int>()(cell.x)^(std::hash<int>()(cell.y) << 1);
    }
};

// represents a node in the A* algorithm, containing the cell index and its f_score
struct AStarNode
{
    CellIndex index;
    double f_score;

    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// comparison function for the priority queue to order nodes by f_score
struct CompareF
{
    bool operator()(const AStarNode& a, const AStarNode& b) const
    {
        return a.f_score > b.f_score;
    }
};

// PlannerCore class encapsulates the A* pathfinding algorithm and related utilities
class PlannerCore {
    public:
        explicit PlannerCore(const rclcpp::Logger& logger);

        double heuristic(const CellIndex& current, const CellIndex& goal) const;

        // returns a vector of neighboring cells that are free (not occupied) in the map
        std::vector <CellIndex> getNeighbors 
        (
            const CellIndex& current, 
            const std::vector<int8_t>& mapData, 
            int width, 
            int height
        ) const;
    
        // checks if a cell is free (not occupied) and within the bounds of the map
        bool isCellFree
        (
            const CellIndex& cell,
            const std::vector<int8_t>& mapData,
            int width,
            int height
        ) const;

        // implements the A* algorithm to find the best path from start to goal
        std::vector<CellIndex> aStar
        (
            const std::vector<int8_t>& mapData,
            int width,
            int height,
            const CellIndex& start,
            const CellIndex& goal
        ) const;
    private:
        rclcpp::Logger logger_;
};

}  

#endif  
