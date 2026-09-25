#include <cmath>
#include <cstdint>
#include <algorithm>

#include "planner_core.hpp"


namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

// =============================================

// estimates the remaining cost from the current cell to the goal
double PlannerCore::heuristic
(
    const CellIndex& current, const CellIndex& goal
) const
{
    // calculates based on distance purely from x and y (no diagonals)
    double distance = std::abs(goal.x - current.x) + std::abs(goal.y - current.y);
    return distance;
}

// =============================================

// creates the coordinates of the neighboring cells
std::vector <CellIndex> PlannerCore::getNeighbors 
(
    const CellIndex& current, 
    const std::vector<int8_t>& mapData, 
    int width, 
    int height
) const
{
    std::vector <CellIndex> neighborCoords {};
    const std::array <CellIndex, 4> directions {{
        {-1, 0},
        {1, 0},
        {0, -1},
        {0, 1}
    }};
    for(const CellIndex& direction : directions)
    {
        int newX {current.x + direction.x};
        int newY {current.y + direction.y};

        // check if within bounds
        if (isCellFree({newX, newY}, mapData, width, height))
        {
            neighborCoords.push_back({newX, newY});
        }
    }
    return neighborCoords;
}

// ============================================

// checks if the cell is available or not 
// (no obstacles nor is it out of bounds)
bool PlannerCore::isCellFree
(
    const CellIndex& cell,
    const std::vector<int8_t>& mapData,
    int width,
    int height
) const
{
    // check if out of bounds
    if (cell.x < 0 || cell.x >= width || cell.y < 0 || cell.y >= height)
    {
        return false;
    }
    int mapIndex {cell.y * width + cell.x};
    int cost {static_cast<int>(mapData[mapIndex])};

    /* -1 means "unknown" - treat it as driveable so the robot is allowed to
       plan into areas it hasn't mapped yet (otherwise it can never leave the
       small patch it has already seen). */
    if (cost < 0)
    {
        return true;
    }

    /* Block anything at or above this cost, not just an exact 100. The costmap
       inflates obstacles outward with costs from 99 down to 0, so checking only
       "== 100" made every inflated cell look free and let the path cut straight
       through the safety buffer and clip corners. 50 corresponds to roughly
       half the inflation radius; raise it to hug walls more, lower it to keep
       further away. */
    constexpr int blocked_cost {50};
    if (cost >= blocked_cost)
    {
        return false;
    }
    else
    {
        return true;
    }
}

// =============================================

// implement the A* algorithm to find the best path to goal
std::vector<CellIndex> PlannerCore::aStar
(
    const std::vector<int8_t>& mapData,
    int width,
    int height,
    const CellIndex& start,
    const CellIndex& goal
) const
{
    // queue of cells waiting to be explored
    std::priority_queue
    <
        AStarNode,
        std::vector<AStarNode>,
        CompareF
    > openSet {};

    // cheapest known "cost" to each cell
    std::unordered_map
    <
        CellIndex,
        double,
        CellIndexHash
    > gScore {};

    // parent of each cell we choose to build the path
    std::unordered_map
    <
        CellIndex,
        CellIndex,
        CellIndexHash
    > cameFrom {};

    // cells already explored
    std::unordered_set
    <
        CellIndex,
        CellIndexHash
    > closedSet {};

    // create the first node (the start)
    gScore[start] = 0.0;
    double startF {heuristic(start, goal)};
    openSet.push(AStarNode(start, startF));

    while (!openSet.empty())
    {
        AStarNode currentNode {openSet.top()};
        openSet.pop();
        CellIndex current {currentNode.index};

        // check if the current node is the goal
        if (current == goal)
        {
            std::vector<CellIndex> bestPath {};
            CellIndex pathCell {current};
            bestPath.push_back(pathCell);

            // reconstruct the best path using cameFrom
            while(pathCell != start)
            {
                pathCell = cameFrom.at(pathCell);
                bestPath.push_back(pathCell);

            }

            std::reverse(bestPath.begin(), bestPath.end());
            return bestPath;
        }

        // check if the current node was already processed beforehand
        if (closedSet.find(current) != closedSet.end())
        {
            continue;
        }

        // mark the current node as processed so we do not expand it again
        closedSet.insert(current);

        // creates the next set of neighbors that might possibly be processed/used
        std::vector <CellIndex> currentNeighbors 
        {
            getNeighbors
            (
                current,
                mapData,
                width,
                height
            )
        };

        // associates the F score to every newly created neighbor
        for (const CellIndex& neighbor : currentNeighbors)
        {
            // calculate current gScore for neighbor
            double tentativeG {gScore[current] + 1.0};
            auto existing = gScore.find(neighbor);

            // note down best tentativeG for that neighbor
            if (existing == gScore.end() || tentativeG < existing->second)
            {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeG;

                // calculate current hScore for neighbor
                double h {heuristic(neighbor, goal)};

                // calculate current fScore for neighbor
                double fScore {tentativeG + h};

                // push neighbor into openSet (for future use)
                openSet.push({neighbor, fScore});
            }


        }
    }
    return {};
    }
} 