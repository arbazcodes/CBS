#include "astar.h"
#include <queue>
#include <cmath>
#include <algorithm>
#include <map>
#include <set>

// Function to calculate Manhattan distance
int ManhattanDistance(const Pair &a, const Pair &b)
{
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}

// Function to calculate the cost of rotation between two directions
int RotationCost(Direction from, Direction to)
{
    return (from == to) ? 0 : 1;
}

std::vector<std::vector<int>> AStarAlgorithm(
    const Pair &start,
    const Pair &goal,
    const std::vector<Constraint> &constraints,
    const std::vector<std::vector<int>> &grid)
{
    int rows = grid.size();
    int cols = (rows > 0) ? grid[0].size() : 0;

    std::vector<Pair> direction_vectors = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    std::vector<Direction> directions = {RIGHT, DOWN, LEFT, UP};

    // Priority queue to store open list
    std::priority_queue<
        std::tuple<int, State>,
        std::vector<std::tuple<int, State>>,
        std::greater<std::tuple<int, State>>>
        open_list;
    open_list.push({0, {start, RIGHT, 0}}); // (priority, (position, direction, time_step))

    std::map<State, int> g_costs;
    g_costs[{start, RIGHT, 0}] = 0;

    std::map<State, State> came_from;

    std::map<int, std::set<Pair>> vertex_constraint_map;
    std::map<int, std::set<std::pair<Pair, Pair>>> edge_constraint_map;

    for (const auto &constraint : constraints)
    {
        if (constraint.IsEdgeConstraint())
        {
            edge_constraint_map[constraint.time].emplace(
                std::make_pair(constraint.x, constraint.y),
                std::make_pair(constraint.x2.value(), constraint.y2.value()));
        }
        else
        {
            vertex_constraint_map[constraint.time].insert({constraint.x, constraint.y});
        }
    }

    while (!open_list.empty())
    {
        auto [_, current] = open_list.top();
        open_list.pop();

        if (current.position == goal)
        {
            std::vector<std::vector<int>> path;
            while (came_from.find(current) != came_from.end())
            {
                path.push_back({current.position.first,
                                current.position.second,
                                static_cast<int>(current.direction),
                                current.time_step});
                current = came_from[current];
            }
            path.push_back({start.first, start.second, static_cast<int>(RIGHT), 0});
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Explore neighbors
        for (size_t i = 0; i < directions.size(); ++i)
        {
            Direction new_direction = static_cast<Direction>(i);
            Pair direction_vector = direction_vectors[i];
            Pair next_cell = {
                current.position.first + direction_vector.first,
                current.position.second + direction_vector.second};
            int next_time_step = current.time_step + 1;

            // Check if the next cell is within bounds and not blocked
            if (next_cell.first < 0 || next_cell.first >= rows ||
                next_cell.second < 0 || next_cell.second >= cols ||
                grid[next_cell.first][next_cell.second] != 1)
            {
                continue;
            }

            // Check vertex constraints
            if (vertex_constraint_map.find(next_time_step) != vertex_constraint_map.end() &&
                vertex_constraint_map[next_time_step].find(next_cell) != vertex_constraint_map[next_time_step].end())
            {
                continue;
            }

            // Check edge constraints
            bool edge_conflict = false;
            if (edge_constraint_map.find(next_time_step) != edge_constraint_map.end())
            {
                for (const auto &edge_constraint : edge_constraint_map[next_time_step])
                {
                    const auto &start_pos = edge_constraint.first;
                    const auto &end_pos = edge_constraint.second;

                    if ((current.position == start_pos && next_cell == end_pos) ||
                        (current.position == end_pos && next_cell == start_pos))
                    {
                        edge_conflict = true;
                        break;
                    }
                }
            }
            if (edge_conflict)
            {
                continue;
            }

            // Determine the cost to transition to the new state
            int rotation_cost_value = RotationCost(current.direction, new_direction);
            int move_cost = 1;
            int final_g_cost = g_costs[current] + rotation_cost_value + move_cost;
            State final_state = {next_cell, new_direction, next_time_step};

            // Check if the new state is better than previously known states
            if (g_costs.find(final_state) == g_costs.end() ||
                final_g_cost < g_costs[final_state])
            {
                g_costs[final_state] = final_g_cost;
                int f_cost = final_g_cost + ManhattanDistance(final_state.position, goal);
                open_list.push({f_cost, final_state});
                came_from[final_state] = current;
            }
        }
    }

    return {};
}
