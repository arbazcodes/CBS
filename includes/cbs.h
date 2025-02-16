#ifndef CBS_H
#define CBS_H

#include <vector>
#include <optional>
#include "astar.h"

class Cbs
{
public:
    explicit Cbs(const std::vector<std::vector<int>> &grid);

    int FindTotalCost(const std::vector<CostPath> &solution) const;
    std::vector<std::vector<int>> FindConflicts(const std::vector<CostPath> &solution) const;
    std::vector<Constraint> GenerateConstraints(const std::vector<std::vector<int>> &conflicts) const;
    std::optional<std::vector<CostPath>> LowLevel(
        const std::vector<Pair> &sources,
        const std::vector<Pair> &destinations,
        const std::vector<Constraint> &constraints) const;
    std::vector<CostPath> HighLevel(
        const std::vector<Pair> &sources,
        const std::vector<Pair> &destinations) const;

private:
    std::vector<std::vector<int>> grid;
    // Helper functions
    std::vector<std::vector<int>> FindConflictsVertex(const std::vector<CostPath> &solution) const;
    std::vector<std::vector<int>> FindConflictsEdge(const std::vector<CostPath> &solution) const;
    std::vector<std::vector<int>> FindStoppingConflicts(const std::vector<CostPath> &solution) const;
    std::vector<std::vector<int>> FindConflictsFollow(const std::vector<CostPath> &solution) const;
};

#endif
