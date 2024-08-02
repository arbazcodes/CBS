#ifndef CBS_H
#define CBS_H

#include <vector>
#include <utility>
#include <optional>
#include "astar.h"
#include <queue>

using CostPath = std::vector<std::vector<int>>;

struct CbsNode
{
    std::vector<CostPath> solution;
    std::vector<Constraint> constraints;
    int cost;

    bool operator<(const CbsNode &other) const
    {
        return cost > other.cost;
    }
};

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
};

#endif 
