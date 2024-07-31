#include <iostream>
#include "cbs.h"

int main() {
    vector<Pair> starts = {{1, 0}, {2, 0}, {2, 2}};
    vector<Pair> goals = {{2, 0}, {1, 0}, {0, 0}};
    
    vector<Constraint> constraints = {};
    
    vector<vector<int>> grid = {
        {1, 1, 1},
        {1, 1, 1},
        {1, 1, 1}
    };

    cbs cbsAlgorithm(grid);

    vector<CostPath> solution = cbsAlgorithm.high_level(starts, goals);

    std::cout << "Final Solution Paths and Costs:" << std::endl;
    for (size_t i = 0; i < starts.size(); i++) {
        std::cout << "Agent " << i << " - Cost: " << solution[i].size() << " Path: ";
        for (const auto& step : solution[i]) {
            std::cout << "(" << step[0] << ", " << step[1] << ", " << step[2] <<", "<< step[3] <<  ") ";
        }
        std::cout << std::endl;
    }

    int totalCost = cbsAlgorithm.findTotalCost(solution);
    std::cout << "Total Cost: " << totalCost << std::endl;

    vector<vector<int>> conflicts = cbsAlgorithm.findConflicts(solution);
    std::cout << "Conflicts: ";
    for (const auto& conflict : conflicts) {
        std::cout << "(" << conflict[0] << ", " << conflict[1] << ", " << conflict[2] << ", " << conflict[3] << ", " << conflict[4] << ") ";
    }
    std::cout << std::endl;

    vector<Constraint> newConstraints = cbsAlgorithm.generateConstraints(conflicts);
    std::cout << "New Constraints: ";
    for (const auto& constraint : newConstraints) {
        std::cout << "(" << constraint.ID << ", " << constraint.x << ", " << constraint.y << ", " << constraint.time << ") ";
    }
    std::cout << std::endl;

    return 0;
}
