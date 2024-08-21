#ifndef ASTAR_H
#define ASTAR_H

#include "util.h"

std::vector<std::vector<int>> AStarAlgorithm(
    const Pair &start,
    const Pair &goal,
    const std::vector<Constraint> &constraints,
    const std::vector<std::vector<int>> &grid);

std::vector<State> GetNeighbors(
    const State &current,
    const Pair &goal,
    const std::vector<std::vector<int>> &grid,
    const std::unordered_map<int, std::unordered_set<Pair>> &vertex_constraints,
    const std::unordered_map<int, std::unordered_set<Pair>> &edge_constraints,
    const std::unordered_map<int, std::unordered_set<Pair>> &following_constraints,
    const std::unordered_map<int, std::unordered_set<Pair>> &constraints);

#endif
