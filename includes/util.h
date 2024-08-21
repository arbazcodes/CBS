#include <vector>
#include <utility>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

typedef std::pair<int, int> Pair;
typedef std::vector<std::vector<int>> CostPath;

enum Direction
{
    UP,
    DOWN,
    LEFT,
    RIGHT,
    STAY
};

struct Conflict
{
    int type, agent_1, agent_2, x, y, time_1, time_2;
};

struct Constraint
{
    int type;
    int id;
    int x;
    int y;
    int time;

    bool operator<(const Constraint &other) const
    {
        return std::tie(type, x, y, time) > std::tie(other.type, other.x, other.y, other.time);
    }
    bool operator==(const Constraint &other) const
    {
        return type == other.type && id == other.id && x == other.x && y == other.y && time == other.time;
    }
};

struct State
{
    Pair position;
    Direction direction;
    int time_step;

    bool operator==(const State &other) const
    {
        return position == other.position && direction == other.direction && time_step == other.time_step;
    }

    bool operator<(const State &other) const
    {
        return std::tie(position, direction, time_step) < std::tie(other.position, other.direction, other.time_step);
    }
};

struct CbsNode
{
    std::vector<Constraint> constraints;
    std::vector<CostPath> solution;
    int cost;

    bool operator<(const CbsNode &other) const
    {
        return (cost > other.cost && constraints.size() > other.constraints.size()); // Min-heap
    }
    bool operator==(const CbsNode &other) const
    {
        return constraints == other.constraints && solution == other.solution && cost == other.cost;
    }
};

namespace std
{
    // Custom hash function for Pair
    template <>
    struct hash<Pair>
    {
        size_t operator()(const Pair &p) const
        {
            return hash<int>()(p.first) ^ (hash<int>()(p.second) << 1);
        }
    };

    // Custom hash function for State
    template <>
    struct hash<State>
    {
        size_t operator()(const State &s) const
        {
            return hash<Pair>()(s.position) ^ (hash<int>()(s.direction) << 1) ^ (hash<int>()(s.time_step) << 1);
        }
    };
}