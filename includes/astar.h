#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <utility>

using namespace std;

typedef pair<int, int> Pair;

struct Constraint {
    int ID, x, y, time;   
};

vector<vector<int>> a_star_algorithm(const Pair& start, const Pair& goal, const vector<Constraint>& constraints, const vector<vector<int>>& grid);

#endif
