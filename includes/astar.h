#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <utility>

using namespace std;

typedef pair<int, int> Pair;

enum Direction {
    UP, DOWN, LEFT, RIGHT
};

struct Constraint {
    int ID, x, y, time;   
};

struct State {
    Pair position;
    int time_step;
    Direction direction;

    bool operator<(const State& other) const {
        return tie(position, time_step, direction) < tie(other.position, other.time_step, other.direction);
    }
};

vector<vector<int>> a_star_algorithm(const Pair& start, const Pair& goal, const vector<Constraint>& constraints, const vector<vector<int>>& grid);

#endif
