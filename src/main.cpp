#include <iostream>
#include "cbs.h"

int main()
{
    // Define starting and goal positions for agents
    std::vector<Pair> starts = {
        {0, 1},
        {1, 1},
        {2, 0},
        {0, 0}};
    std::vector<Pair> goals = {
        {1, 1},
        {2, 1},
        {0, 2},
        {2, 2}};

    // Define constraints (empty in this case)
    std::vector<Constraint> constraints = {};

    // Define the grid where agents operate
    std::vector<std::vector<int>> grid = {
        {1, 1, 1},
        {1, 1, 1},
        {1, 1, 1}};

    // Initialize the CBS algorithm with the grid
    Cbs cbs_algorithm(grid);

    // Compute the solution using the high-level CBS algorithm
    std::vector<CostPath> solution = cbs_algorithm.HighLevel(starts, goals);

    if (solution.empty())
    {
        std::cout << "No valid solution found." << std::endl;
        return 1; // Indicate failure
    }

    // Output the solution paths and costs
    std::cout << "Final Solution Paths and Costs:" << std::endl;
    for (size_t i = 0; i < starts.size(); ++i)
    {
        std::cout << "Agent " << i << " - Cost: " << solution[i].size() << " Path: ";
        int x = 0;
        for (const auto &step : solution[i])
        {
            std::cout << "(" << step[0] << ", " << step[1] << ", " << step[2] << ", " << step[3] << ")";
            if (!(x == solution[i].size() - 1))
                std::cout << "--->";
            x++;
        }
        std::cout << std::endl;
    }

    // Calculate and output the total cost of the solution
    int total_cost = cbs_algorithm.FindTotalCost(solution);
    std::cout << "Total Cost: " << total_cost << std::endl;

    // Find and output conflicts in the solution
    std::vector<std::vector<int>> conflicts = cbs_algorithm.FindConflicts(solution);
    std::cout << "Conflicts: ";
    for (const auto &conflict : conflicts)
    {
        std::cout << "(" << conflict[0] << ", " << conflict[1] << ", " << conflict[2] << ", " << conflict[3] << ", " << conflict[4] << ") ";
    }
    std::cout << std::endl;

    // Generate and output new constraints based on conflicts
    std::vector<Constraint> new_constraints = cbs_algorithm.GenerateConstraints(conflicts);
    std::cout << "New Constraints: ";
    for (const auto &constraint : new_constraints)
    {
        std::cout << "(" << constraint.id << ", " << constraint.x << ", " << constraint.y << ", " << constraint.time << ") ";
    }
    std::cout << std::endl;

    return 0;
}