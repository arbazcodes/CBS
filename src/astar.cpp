#include "astar.h"
#include <queue>
#include <cmath>
#include <algorithm>
#include <map>
#include <set>

namespace std {
    // Hash function for State
    template <>
    struct hash<State> {
        size_t operator()(const State& s) const {
            return hash<int>()(s.position.first) ^ hash<int>()(s.position.second) ^ hash<int>()(s.time_step) ^ hash<int>()(s.direction);
        }
    };
}

int manhattan_distance(const Pair& a, const Pair& b) {
    return abs(a.first - b.first) + abs(a.second - b.second);
}

int rotation_cost(Direction from, Direction to) {
    if (from == to) return 0;
    return 1;
}

vector<vector<int>> a_star_algorithm(const Pair& start, const Pair& goal, const vector<Constraint>& constraints, const vector<vector<int>>& grid) {
    int rows = grid.size();
    int cols = rows > 0 ? grid[0].size() : 0;

    vector<Pair> direction_vectors = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    vector<Direction> directions = {RIGHT, DOWN, LEFT, UP};

    priority_queue<tuple<int, State>, vector<tuple<int, State>>, greater<tuple<int, State>>> open_list;
    open_list.push({0, {start, 0, RIGHT}});  // (priority, (position, time_step, direction))

    map<State, int> g_costs;
    g_costs[{start, 0, RIGHT}] = 0;

    map<State, State> came_from;

    map<int, set<Pair>> constraint_map;
    for (const auto& c : constraints) {
        constraint_map[c.time].insert({c.x, c.y});
    }

    while (!open_list.empty()) {
        auto [_, current] = open_list.top();
        open_list.pop();

        if (current.position == goal) {
            vector<vector<int>> path;
            while (came_from.find(current) != came_from.end()) {
                path.push_back({current.position.first, current.position.second, current.time_step, current.direction});
                current = came_from[current];
            }
            path.push_back({start.first, start.second, 0, RIGHT});
            reverse(path.begin(), path.end());
            return path;
        }

        // explore neighbors
        for (size_t i = 0; i < directions.size(); ++i) {
            Direction new_direction = static_cast<Direction>(i);
            Pair direction_vector = direction_vectors[i];
            Pair next_cell = {current.position.first + direction_vector.first, current.position.second + direction_vector.second};
            int next_time_step = current.time_step + 1;

            // if within bounds and not blocked
            if (next_cell.first < 0 || next_cell.first >= rows ||
                next_cell.second < 0 || next_cell.second >= cols ||
                grid[next_cell.first][next_cell.second] != 1) {
                continue;
            }

            // check constraints
            if (constraint_map.find(next_time_step) != constraint_map.end() &&
                constraint_map[next_time_step].find(next_cell) != constraint_map[next_time_step].end()) {
                continue;
            }

            // change direction (create intermediate state)
            if (current.direction != new_direction) {
                State direction_change_state = {current.position, current.time_step + 1, new_direction};
                int rotation_cost_value = rotation_cost(current.direction, new_direction);
                int g_cost_direction_change = g_costs[current] + rotation_cost_value;

                if (g_costs.find(direction_change_state) == g_costs.end() || g_cost_direction_change < g_costs[direction_change_state]) {
                    g_costs[direction_change_state] = g_cost_direction_change;
                    int f_cost_direction_change = g_cost_direction_change + manhattan_distance(direction_change_state.position, goal);
                    open_list.push({f_cost_direction_change, direction_change_state});
                    came_from[direction_change_state] = current;
                }
            }

            // move in the new direction (after direction change)
            if (current.direction == new_direction) {
                State final_state = {next_cell, next_time_step, new_direction};
                int move_cost = 1;
                int final_g_cost = g_costs[current] + move_cost;

                if (g_costs.find(final_state) == g_costs.end() || final_g_cost < g_costs[final_state]) {
                    g_costs[final_state] = final_g_cost;
                    int f_cost = final_g_cost + manhattan_distance(final_state.position, goal);
                    open_list.push({f_cost, final_state});
                    came_from[final_state] = current;
                }
            }
        }
    }

    return {};
}

