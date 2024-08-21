#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <iterator>
#include <set>
#include "cbs.h"

// Define a type alias for Pair
using Pair = std::pair<int, int>;

// Function to generate unique random positions within grid bounds
std::vector<Pair> GenerateUniqueRandomPositions(int num_positions, int grid_rows, int grid_cols, std::mt19937 &rng)
{
    if (num_positions > grid_rows * grid_cols)
    {
        throw std::invalid_argument("Number of positions requested exceeds the number of available grid cells.");
    }

    std::vector<Pair> all_positions;
    for (int row = 0; row < grid_rows; ++row)
    {
        for (int col = 0; col < grid_cols; ++col)
        {
            all_positions.emplace_back(row, col);
        }
    }

    std::shuffle(all_positions.begin(), all_positions.end(), rng);
    all_positions.resize(num_positions);
    return all_positions;
}

// Function to ensure unique start and goal positions
void EnsureUniqueStartAndGoal(std::vector<Pair> &starts, std::vector<Pair> &goals, std::mt19937 &rng, int grid_rows, int grid_cols)
{
    std::set<Pair> start_set;
    std::set<Pair> goal_set;

    // Generate unique starts
    while (start_set.size() < starts.size())
    {
        auto new_starts = GenerateUniqueRandomPositions(starts.size(), grid_rows, grid_cols, rng);
        start_set.insert(new_starts.begin(), new_starts.end());
    }
    starts.assign(start_set.begin(), start_set.end());

    // Generate unique goals
    while (goal_set.size() < goals.size())
    {
        auto new_goals = GenerateUniqueRandomPositions(goals.size(), grid_rows, grid_cols, rng);
        goal_set.insert(new_goals.begin(), new_goals.end());
    }
    goals.assign(goal_set.begin(), goal_set.end());

    // Ensure no start position is the same as any goal position
    std::set<Pair> all_positions = start_set;
    all_positions.insert(goal_set.begin(), goal_set.end());

    // Regenerate goals if there are overlaps with start positions
    while (goal_set.size() < goals.size())
    {
        auto new_goals = GenerateUniqueRandomPositions(goals.size(), grid_rows, grid_cols, rng);
        for (const auto &goal : new_goals)
        {
            if (all_positions.find(goal) == all_positions.end())
            {
                goal_set.insert(goal);
                all_positions.insert(goal);
            }
        }
    }
    goals.assign(goal_set.begin(), goal_set.end());
}

int main()
{
    const int num_agents = 4; // Number of agents
    const int grid_rows = 3;
    const int grid_cols = 3;
    const int num_iterations = 500; // Number of iterations to run

    // Initialize the grid
    std::vector<std::vector<int>> grid(grid_rows, std::vector<int>(grid_cols, 1));

    // Setup random number generation
    std::random_device rd;
    std::mt19937 rng(rd());

    for (int iteration = 0; iteration < num_iterations; ++iteration)
    {
        // Generate unique random sources and destinations
        auto starts = GenerateUniqueRandomPositions(num_agents, grid_rows, grid_cols, rng);
        auto goals = GenerateUniqueRandomPositions(num_agents, grid_rows, grid_cols, rng);

        // Ensure that starts and goals are different
        EnsureUniqueStartAndGoal(starts, goals, rng, grid_rows, grid_cols);

        // Define constraints (empty in this case)
        std::vector<Constraint> constraints = {};

        // Initialize the CBS algorithm with the grid
        Cbs cbs_algorithm(grid);

        for (int i = 0; i < starts.size(); i++)
        {
            std::cout << "Start: (" << starts[i].first << ", " << starts[i].second << ")---" << "Goal: (" << goals[i].first << ", " << goals[i].second << ")" << std::endl;
        }

        // Compute the solution using the high-level CBS algorithm
        std::vector<CostPath> solution = cbs_algorithm.HighLevel(starts, goals);

        if (solution.empty())
        {
            std::cout << "Iteration " << iteration << ": No valid solution found." << std::endl;
            continue; // Proceed to the next iteration
        }

        // Output the solution paths and costs
        std::cout << "Iteration " << iteration << ": Final Solution Paths and Costs:" << std::endl;
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
        std::cout << "Iteration " << iteration << ": Total Cost: " << total_cost << std::endl;

        // Find and output conflicts in the solution
        std::vector<std::vector<int>> conflicts = cbs_algorithm.FindConflicts(solution);
        std::cout << "Iteration " << iteration << ": Conflicts: ";
        for (const auto &conflict : conflicts)
        {
            std::cout << "(" << conflict[0] << ", " << conflict[1] << ", " << conflict[2] << ", " << conflict[3] << ", " << conflict[4] << ") ";
        }
        std::cout << std::endl;

        // Generate and output new constraints based on conflicts
        std::vector<Constraint> new_constraints = cbs_algorithm.GenerateConstraints(conflicts);
        std::cout << "Iteration " << iteration << ": New Constraints: ";
        for (const auto &constraint : new_constraints)
        {
            std::cout << "(" << constraint.id << ", " << constraint.x << ", " << constraint.y << ", " << constraint.time << ") ";
        }
        std::cout << std::endl;
    }

    return 0;
}
