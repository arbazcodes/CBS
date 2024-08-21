#include "astar.h"
#include <queue>
#include <iostream>

// Function to calculate Manhattan distance
int ManhattanDistance(const Pair &a, const Pair &b)
{
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}

// Function to calculate the cost of rotation between two directions
int RotationCost(Direction from, Direction to)
{
    if (from == to)
        return 0;
    if ((from == UP && to == DOWN) || (from == DOWN && to == UP) ||
        (from == LEFT && to == RIGHT) || (from == RIGHT && to == LEFT))
    {
        return 1; // Cost for opposite directions
    }
    return 2; // Default rotation cost
}

// Function to get constraint time for a given position
std::optional<int> GetConstraintTime(const Pair &position, const std::unordered_map<int, std::unordered_set<Pair>> &constraints)
{
    for (const auto &entry : constraints)
    {
        if (entry.second.find(position) != entry.second.end())
            return entry.first;
    }
    return std::nullopt;
}

// Function to get valid neighbors for a given state
std::vector<State> GetNeighbors(
    const State &current,
    const Pair &goal,
    const std::vector<std::vector<int>> &grid,
    const std::unordered_map<int, std::unordered_set<Pair>> &vertex_constraints,
    const std::unordered_map<int, std::unordered_set<Pair>> &edge_constraints,
    const std::unordered_map<int, std::unordered_set<Pair>> &following_constraints,
    const std::unordered_map<int, std::unordered_set<Pair>> &stopping_constraints)
{
    std::vector<State> neighbors;
    static const std::vector<Pair> direction_vectors = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {0, 0}};
    static const std::vector<Direction> directions = {DOWN, RIGHT, UP, LEFT, STAY};

    for (size_t i = 0; i < directions.size(); ++i)
    {
        Direction new_direction = directions[i];
        Pair direction_vector = direction_vectors[i];
        Pair next_cell = {current.position.first + direction_vector.first, current.position.second + direction_vector.second};
        int next_time_step = current.time_step + 1;

        // Boundary and obstacle check
        if (next_cell.first < 0 || next_cell.first >= grid.size() || next_cell.second < 0 || next_cell.second >= grid[0].size() || grid[next_cell.first][next_cell.second] != 1)
            continue;

        // Constraint checks
        if (vertex_constraints.count(next_time_step) && vertex_constraints.at(next_time_step).count(next_cell))
            continue;
        if (edge_constraints.count(next_time_step) && edge_constraints.at(next_time_step).count(next_cell))
            continue;
        if (following_constraints.count(next_time_step) && following_constraints.at(next_time_step).count(next_cell))
            continue;
        if (stopping_constraints.count(next_time_step) && stopping_constraints.at(next_time_step).count(next_cell))
            continue;

        // Update direction if moving in opposite direction
        if ((current.direction == UP && new_direction == DOWN) || (current.direction == DOWN && new_direction == UP) ||
            (current.direction == LEFT && new_direction == RIGHT) || (current.direction == RIGHT && new_direction == LEFT))
        {
            new_direction = current.direction;
        }

        neighbors.push_back({next_cell, new_direction, next_time_step});
    }

    return neighbors;
}

// A* algorithm implementation
std::vector<std::vector<int>> AStarAlgorithm(
    const Pair &start,
    const Pair &goal,
    const std::vector<Constraint> &constraints,
    const std::vector<std::vector<int>> &grid)
{
    int rows = grid.size();
    int cols = (rows > 0) ? grid[0].size() : 0;

    std::priority_queue<
        std::tuple<int, State>,
        std::vector<std::tuple<int, State>>,
        std::greater<std::tuple<int, State>>>
        open_list;
    open_list.push({0, {start, UP, 0}});

    std::unordered_map<State, int> g_costs;
    g_costs[{start, UP, 0}] = 0;

    std::unordered_map<State, State> came_from;

    std::unordered_map<int, std::unordered_set<Pair>> vertex_constraints;
    std::unordered_map<int, std::unordered_set<Pair>> edge_constraints;
    std::unordered_map<int, std::unordered_set<Pair>> stopping_constraints;
    std::unordered_map<int, std::unordered_set<Pair>> following_constraints;

    for (const auto &constraint : constraints)
    {
        if (constraint.type == 0)
        {
            vertex_constraints[constraint.time].insert({constraint.x, constraint.y});
        }
        else if (constraint.type == 1)
        {
            edge_constraints[constraint.time].insert({constraint.x, constraint.y});
        }
        else if (constraint.type == 2)
        {
            stopping_constraints[constraint.time].insert({constraint.x, constraint.y});
        }
        else if (constraint.type == 3)
        {
            following_constraints[constraint.time].insert({constraint.x, constraint.y});
        }
    }

    while (!open_list.empty())
    {
        auto [_, current] = open_list.top();
        open_list.pop();

        if (current.position == goal)
        {
            if (GetConstraintTime(current.position, stopping_constraints).has_value() && current.time_step < GetConstraintTime(current.position, stopping_constraints).value())
            {
                std::vector<State> neighbors = GetNeighbors(current, goal, grid, vertex_constraints, edge_constraints, following_constraints, stopping_constraints);

                for (auto &neighbor : neighbors)
                {
                    if (neighbor.direction == STAY)
                        neighbor.direction = current.direction;
                    int rotation_cost_value = RotationCost(current.direction, neighbor.direction);
                    int move_cost = (current.position == neighbor.position) ? 1 : 2;
                    int final_g_cost = g_costs[current] + rotation_cost_value + move_cost;
                    State final_state = neighbor;

                    if (g_costs.find(final_state) == g_costs.end() || final_g_cost < g_costs[final_state])
                    {
                        g_costs[final_state] = final_g_cost;
                        int f_cost = final_g_cost + ManhattanDistance(final_state.position, goal);
                        open_list.push({f_cost, final_state});
                        came_from[final_state] = current;
                    }
                }
                continue; // Skip this goal state as it violates the stopping constraint
            }

            std::vector<std::vector<int>> path;
            while (came_from.find(current) != came_from.end())
            {
                path.push_back({current.position.first, current.position.second, static_cast<int>(current.direction), current.time_step});
                current = came_from[current];
            }
            path.push_back({start.first, start.second, static_cast<int>(UP), 0});
            std::reverse(path.begin(), path.end());
            return path;
        }

        std::vector<State> neighbors = GetNeighbors(current, goal, grid, vertex_constraints, edge_constraints, following_constraints, stopping_constraints);

        for (auto &neighbor : neighbors)
        {
            if (neighbor.direction == STAY)
                neighbor.direction = current.direction;
            int rotation_cost_value = RotationCost(current.direction, neighbor.direction);
            int move_cost = (current.position == neighbor.position) ? 1 : 2;
            int final_g_cost = g_costs[current] + rotation_cost_value + move_cost;
            State final_state = neighbor;

            if (g_costs.find(final_state) == g_costs.end() || final_g_cost < g_costs[final_state])
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
