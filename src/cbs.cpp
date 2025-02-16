#include "cbs.h"
#include <map>
#include <iostream>
#include <optional>
#include <queue>
#include <limits>
// Constructor
Cbs::Cbs(const std::vector<std::vector<int>> &grid) : grid(grid) {}

std::optional<std::vector<CostPath>> Cbs::LowLevel(
    const std::vector<Pair> &sources,
    const std::vector<Pair> &destinations,
    const std::vector<Constraint> &constraints) const
{
    std::vector<CostPath> solution;
    std::map<int, std::vector<Constraint>> constraint_by_id;

    for (const auto &constraint : constraints)
    {
        constraint_by_id[constraint.id].push_back(constraint);
    }

    for (int i = 0; i < sources.size(); ++i)
    {
        auto path = AStarAlgorithm(sources[i], destinations[i], constraint_by_id[i], grid);

        if (path.empty())
        {
            // std::cout << "No solution found for Agent " << i << " with constraint." << std::endl;
            return std::nullopt;
        }
        solution.push_back(path);
    }

    return solution;
}

// Calculate the total cost of a solution
int Cbs::FindTotalCost(const std::vector<CostPath> &solution) const
{
    int total_cost = 0;

    for (const auto &path : solution)
    {
        total_cost += path.size();
    }

    return total_cost;
}

std::vector<std::vector<int>> Cbs::FindConflictsVertex(const std::vector<CostPath> &solution) const
{
    std::vector<std::vector<int>> Conflicts;

    for (int i = 0; i < solution.size(); ++i)
    {
        const std::vector<std::vector<int>> &path_1 = solution[i];

        for (int j = i + 1; j < solution.size(); ++j)
        {
            const std::vector<std::vector<int>> &path_2 = solution[j];

            // Check for vertex conflicts
            for (int t = 0; t < path_1.size() && t < path_2.size(); ++t)
            {
                const auto &step_1 = path_1[t];
                const auto &step_2 = path_2[t];

                if (step_1.size() == 4 && step_2.size() == 4)
                {
                    int x1 = step_1[0];
                    int y1 = step_1[1];
                    int x2 = step_2[0];
                    int y2 = step_2[1];

                    if ((x1 == x2 && y1 == y2))
                    {
                        Conflicts.push_back({i, j, x1, y1, t});
                    }
                }
            }
        }
    }

    return Conflicts;
}

std::vector<std::vector<int>> Cbs::FindConflictsEdge(const std::vector<CostPath> &solution) const
{
    std::vector<std::vector<int>> conflicts;

    for (int i = 0; i < solution.size(); ++i)
    {
        const auto &path1 = solution[i];
        for (int j = i + 1; j < solution.size(); ++j)
        {
            const auto &path2 = solution[j];
            for (int t = 0; t < std::min(path1.size(), path2.size()) - 1; ++t)
            {
                const auto &pos1_t = path1[t];
                const auto &pos1_t1 = path1[t + 1];
                const auto &pos2_t = path2[t];
                const auto &pos2_t1 = path2[t + 1];

                // Check for edge conflicts where agents swap places
                if ((pos1_t[0] == pos2_t1[0] && pos1_t[1] == pos2_t1[1] &&
                     pos2_t[0] == pos1_t1[0] && pos2_t[1] == pos1_t1[1]))
                {
                    // Edge conflict detected
                    conflicts.push_back({i, j, pos1_t[0], pos1_t[1], pos2_t[0], pos2_t[1], t + 1});
                }
            }
        }
    }

    return conflicts;
}

std::vector<std::vector<int>> Cbs::FindStoppingConflicts(const std::vector<CostPath> &solution) const
{
    std::vector<std::vector<int>> stopping_conflicts;

    for (int i = 0; i < solution.size(); ++i)
    {
        const auto &path1 = solution[i];
        if (path1.empty())
            continue;

        int goal_x = path1.back()[0];
        int goal_y = path1.back()[1];

        for (int j = 0; j < solution.size(); ++j)
        {
            if (i == j)
                continue;

            const auto &path2 = solution[j];
            for (int t = path1.size(); t < path2.size(); ++t)
            {

                const auto &pos2 = path2[t];

                if (pos2.size() == 4 && pos2[0] == goal_x && pos2[1] == goal_y)
                {
                    stopping_conflicts.push_back({i, j, goal_x, goal_y, t, (int)path1.size() - 1});
                }
            }
        }
    }

    return stopping_conflicts;
}

std::vector<std::vector<int>> Cbs::FindConflictsFollow(const std::vector<std::vector<std::vector<int>>> &solution) const
{
    std::vector<std::vector<int>> Conflicts;
    int num_paths = solution.size();

    for (int i = 0; i < num_paths; ++i)
    {
        const auto &path_1 = solution[i];
        int path_1_size = path_1.size();
        if (path_1_size == 0)
            continue;

        for (int j = i + 1; j < num_paths; ++j)
        {
            const auto &path_2 = solution[j];
            int path_2_size = path_2.size();
            if (path_2_size == 0)
                continue;

            int min_size = std::min(path_1_size, path_2_size);

            for (int t = 0; t < min_size; ++t)
            {
                if (t < path_1_size && t > 0 && t - 1 < path_2_size)
                {
                    const auto &step_1 = path_1[t];
                    const auto &step_2_back = path_2[t - 1];

                    if (step_1.size() == 4 && step_2_back.size() == 4)
                    {
                        int x1 = step_1[0];
                        int y1 = step_1[1];
                        int x2_back = step_2_back[0];
                        int y2_back = step_2_back[1];

                        if (x1 == x2_back && y1 == y2_back)
                        {
                            Conflicts.push_back({-1, j, i, x1, y1, t - 1, t});
                        }
                    }
                }

                if (t + 1 < path_2_size)
                {
                    const auto &step_1 = path_1[t];
                    const auto &step_2_fwd = path_2[t + 1];

                    if (step_1.size() == 4 && step_2_fwd.size() == 4)
                    {
                        int x1 = step_1[0];
                        int y1 = step_1[1];
                        int x2_fwd = step_2_fwd[0];
                        int y2_fwd = step_2_fwd[1];

                        if (x1 == x2_fwd && y1 == y2_fwd)
                        {
                            Conflicts.push_back({-1, j, i, x1, y1, t + 1, t});
                        }
                    }
                }
            }
        }
    }

    return Conflicts;
}

std::vector<std::vector<int>> Cbs::FindConflicts(const std::vector<CostPath> &solution) const
{
    // Use a set to automatically handle duplicates
    std::vector<std::vector<int>> conflicts;

    // Check for vertex conflicts
    auto vertex_conflicts = FindConflictsVertex(solution);
    conflicts.insert(conflicts.end(), vertex_conflicts.begin(), vertex_conflicts.end());

    // Check for edge conflicts
    auto edge_conflicts = FindConflictsEdge(solution);
    conflicts.insert(conflicts.end(), edge_conflicts.begin(), edge_conflicts.end());

    // Check for stopping conflicts
    auto stopping_conflicts = FindStoppingConflicts(solution);
    conflicts.insert(conflicts.end(), stopping_conflicts.begin(), stopping_conflicts.end());

    // Check for following conflicts
    auto follow_conflicts = FindConflictsFollow(solution);
    conflicts.insert(conflicts.end(), follow_conflicts.begin(), follow_conflicts.end());

    return conflicts;
}

std::vector<Constraint> Cbs::GenerateConstraints(const std::vector<std::vector<int>> &conflicts) const
{
    // Use a set to avoid duplicate constraints
    std::vector<Constraint> constraint_set;

    for (const auto &conflict : conflicts)
    {
        if (conflict.size() == 5 && conflict[0] != -1) // Vertex conflict
        {
            constraint_set.push_back({0, conflict[0], conflict[2], conflict[3], conflict[4]});
            constraint_set.push_back({0, conflict[1], conflict[2], conflict[3], conflict[4]});
        }
        else if (conflict.size() == 7 && conflict[0] != -1) // Edge conflict
        {
            constraint_set.push_back({1, conflict[0], conflict[4], conflict[5], conflict[6]});
            constraint_set.push_back({1, conflict[1], conflict[2], conflict[3], conflict[6]});
        }
        else if (conflict.size() == 6 && conflict[0] != -1) // Stopping conflict
        {
            constraint_set.push_back({2, conflict[1], conflict[2], conflict[3], conflict[4]});
            constraint_set.push_back({2, conflict[0], conflict[2], conflict[3], conflict[4]});
        }
        else if (conflict.size() == 7 && conflict[0] == -1) // Following conflict
        {
            if (conflict[5] > 0)
                constraint_set.push_back({3, conflict[1], conflict[3], conflict[4], conflict[5]});
            constraint_set.push_back({3, conflict[2], conflict[3], conflict[4], conflict[6]});
        }
    }

    // Convert the set to a vector
    std::vector<Constraint> constraints(constraint_set.begin(), constraint_set.end());

    return constraints;
}

namespace std
{
    template <>
    struct hash<CbsNode>
    {
        std::size_t operator()(const CbsNode &node) const
        {
            std::size_t h1 = std::hash<int>{}(node.cost);
            std::size_t h2 = std::hash<std::string>{}(SerializeConstraints(node.constraints));
            return h1 ^ (h2 << 1);
        }

        std::string SerializeConstraints(const std::vector<Constraint> &constraints) const
        {
            std::string s;
            for (const auto &constraint : constraints)
            {
                s += std::to_string(constraint.type) + "," +
                     std::to_string(constraint.id) + "," +
                     std::to_string(constraint.x) + "," +
                     std::to_string(constraint.y) + "," +
                     std::to_string(constraint.time) + ";";
            }
            return s;
        }
    };
}

std::vector<CostPath> Cbs::HighLevel(const std::vector<Pair> &sources, const std::vector<Pair> &destinations) const
{
    std::priority_queue<CbsNode> open;
    std::unordered_set<CbsNode> closed;
    int best_cost = std::numeric_limits<int>::max();

    CbsNode root;
    root.constraints = {};

    auto initial_solution = LowLevel(sources, destinations, {});
    if (!initial_solution)
    {
        std::cout << "No initial solution found." << std::endl;
        return {};
    }

    root.solution = *initial_solution;
    root.cost = FindTotalCost(root.solution);
    open.push(root);

    while (!open.empty())
    {
        CbsNode current = open.top();
        open.pop();

        // Check if current node's cost exceeds the best cost found so far
        if (current.cost >= best_cost)
            continue;

        auto conflicts = FindConflicts(current.solution);

        if (conflicts.empty())
        {
            best_cost = current.cost;
            std::cout << "Solution found with total cost: " << current.cost << std::endl;
            return current.solution;
        }

        auto conflict = conflicts[0];
        auto new_constraints = GenerateConstraints({conflict});

        for (const auto &constraint : new_constraints)
        {
            CbsNode child = current;
            child.constraints.push_back(constraint);

            // Skip if child state has been visited before
            if (closed.find(child) != closed.end())
                continue;

            auto new_solution = LowLevel(sources, destinations, child.constraints);

            if (!new_solution.has_value())
                continue;

            child.solution = new_solution.value();
            child.cost = FindTotalCost(child.solution);

            open.push(child);
            closed.insert(child); // Mark this state as visited
        }
    }

    std::cout << "No feasible solution found." << std::endl;
    return {};
}

// std::vector<CostPath> Cbs::HighLevel(const std::vector<Pair> &sources, const std::vector<Pair> &destinations) const
// {

//     auto solution = LowLevel(sources, destinations, {{2 ,1, 0, 0, 2}, {2, 0, 0, 0, 2}, {2, 2, 2, 2, 2},
//     {2, 3, 2, 2, 2}, {3, 2, 2, 1, 1}});
//     std::cout << "Final Solution Paths and Costs:" << std::endl;
//     for (size_t i = 0; i < sources.size(); ++i)
//     {
//         std::cout << "Agent " << i << " - Cost: " << solution.value()[i].size() << " Path: ";
//         int x = 0;
//         for (const auto &step : solution.value()[i])
//         {
//             std::cout << "(" << step[0] << ", " << step[1] << ", " << step[2] << ", " << step[3] << ")";
//             if (!(x == solution.value()[i].size() - 1))
//                 std::cout << "--->";
//             x++;
//         }
//         std::cout << std::endl;
//     }

//     // Calculate and output the total cost of the solution
//     int total_cost = FindTotalCost(solution.value());
//     std::cout << "Total Cost: " << total_cost << std::endl;

//     // Find and output conflicts in the solution
//     std::vector<std::vector<int>> conflicts = FindConflicts(solution.value());
//     std::cout << "Conflicts: ";
//     for (const auto &conflict : conflicts)
//     {
//         std::cout << "(" << conflict[0] << ", " << conflict[1] << ", " << conflict[2] << ", " << conflict[3] << ", " << conflict[4] << ", "<< conflict[5]<<") ";
//     }
//     std::cout << std::endl;

//     // Generate and output new constraints based on conflicts
//     std::vector<Constraint> new_constraints = GenerateConstraints(conflicts);
//     std::cout << "New Constraints: ";
//     for (const auto &constraint : new_constraints)
//     {
//         if(constraint.type == 0){
//             std::cout << "Vertex Constraint: " << "(" << constraint.id << ", " << constraint.x << ", " << constraint.y << ", " << constraint.time << ") ";
//         }
//         if (constraint.type == 1)
//         {
//             std::cout << "Edge Constraint: " << "(" << constraint.id << ", " << constraint.x << ", " << constraint.y << ", " << constraint.time << ") ";
//         }
//         if (constraint.type == 2)
//         {
//             std::cout << "Stopping Constraint: " << "(" << constraint.id << ", " << constraint.x << ", " << constraint.y << ", " << constraint.time << ") ";
//         }
//         if (constraint.type == 3)
//         {
//             std::cout << "Following Constraint: " << "(" << constraint.id << ", " << constraint.x << ", " << constraint.y << ", " << constraint.time << ") ";
//         }
//     }
//     std::cout << std::endl;
//     return {};
// }
