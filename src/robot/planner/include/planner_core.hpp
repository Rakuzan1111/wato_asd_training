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

struct CellIndexHash
{
    std::size_t operator()(const CellIndex& cell) const
    {
        return std::hash<int>()(cell.x)^(std::hash<int>()(cell.y) << 1);
    }
};

struct AStarNode
{
    CellIndex index;
    double f_score;

    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF
{
    bool operator()(const AStarNode& a, const AStarNode& b) const
    {
        return a.f_score > b.f_score;
    }
};

class PlannerCore {
    public:
        explicit PlannerCore(const rclcpp::Logger& logger);

        double heuristic(const CellIndex& current, const CellIndex& goal) const;

        std::vector <CellIndex> getNeighbors 
        (
            const CellIndex& current, 
            const std::vector<int8_t>& mapData, 
            int width, 
            int height
        ) const;
    
        bool isCellFree
        (
            const CellIndex& cell,
            const std::vector<int8_t>& mapData,
            int width,
            int height
        ) const;
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
