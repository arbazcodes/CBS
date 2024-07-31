#include "cbs.h"
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <iostream>

// Constructor
Cbs::Cbs(const std::vector<std::vector<int>>& grid) : grid(grid) {}

// Low-level search using A* algorithm
std::vector<CostPath> Cbs::LowLevel(
    const std::vector<Pair>& sources,
    const std::vector<Pair>& destinations,
    const std::vector<Constraint>& constraints
) const {
    std::vector<CostPath> solution;

    std::map<int, std::vector<Constraint>> constraint_by_id;
    for (const auto& constraint : constraints) {
        constraint_by_id[constraint.id].push_back(constraint);
    }

    for (size_t i = 0; i < sources.size(); ++i) {
        std::vector<std::vector<int>> path = AStarAlgorithm(
            sources[i], destinations[i], constraint_by_id[i], grid
        );
        solution.push_back(path);
    }

    return solution;
}

// Calculate the total cost of a solution
int Cbs::FindTotalCost(const std::vector<CostPath>& solution) const {
    int total_cost = 0;

    for (const auto& path : solution) {
        total_cost += path.size();
    }

    return total_cost;
}

// Find conflicts between paths in the solution
std::vector<std::vector<int>> Cbs::FindConflicts(const std::vector<CostPath>& solution) const {
    std::vector<std::vector<int>> conflicts;

    for (size_t i = 0; i < solution.size(); ++i) {
        const auto& path_1 = solution[i];

        for (size_t j = i + 1; j < solution.size(); ++j) {
            const auto& path_2 = solution[j];

            for (size_t t = 0; t < path_1.size() && t < path_2.size(); ++t) {
                const auto& step_1 = path_1[t];
                const auto& step_2 = path_2[t];

                if (step_1.size() == 3 && step_2.size() == 3) {
                    int x1 = step_1[0];
                    int y1 = step_1[1];
                    int x2 = step_2[0];
                    int y2 = step_2[1];

                    if (x1 == x2 && y1 == y2) {
                        conflicts.push_back({static_cast<int>(i), static_cast<int>(j), x1, y1, static_cast<int>(t)});
                    }
                }
            }
        }
    }

    return conflicts;
}

// Generate constraints based on conflicts
std::vector<Constraint> Cbs::GenerateConstraints(const std::vector<std::vector<int>>& conflicts) const {
    std::vector<Constraint> constraints;

    for (const auto& conflict : conflicts) {
        constraints.push_back({conflict[0], conflict[2], conflict[3], conflict[4]});
        constraints.push_back({conflict[1], conflict[2], conflict[3], conflict[4]});
    }

    return constraints;
}

// High-level CBS algorithm
std::vector<CostPath> Cbs::HighLevel(const std::vector<Pair>& sources, const std::vector<Pair>& destinations) const{
    std::priority_queue<CbsNode> open;
    CbsNode root;
    root.constraints = {};
    root.solution = LowLevel(sources, destinations, root.constraints);
    root.cost = FindTotalCost(root.solution);
    open.push(root);

    while (!open.empty()) {
        CbsNode current = open.top();
        open.pop();

        std::vector<std::vector<int>> conflicts = FindConflicts(current.solution);

        if (conflicts.empty()) {
            return current.solution;
        }

        for (const auto& conflict : conflicts) {
            CbsNode child1 = current;
            CbsNode child2 = current;

            std::vector<Constraint> new_constraints = GenerateConstraints({conflict});
            
            child1.constraints.push_back(new_constraints[0]);
            child1.solution = LowLevel(sources, destinations, child1.constraints);
            child1.cost = FindTotalCost(child1.solution);

            child2.constraints.push_back(new_constraints[1]);
            child2.solution = LowLevel(sources, destinations, child2.constraints);
            child2.cost = FindTotalCost(child2.solution);

            // Note: Handle cases where LowLevel might not find a solution
            open.push(child1);
            open.push(child2);
        }
    }

    return {};
}



// what happens if low level doesn't find a solution?
            // std::optional<type> output; if out<type>.has_value()