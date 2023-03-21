#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <pibt.hpp>

int main() {
    std::cout << "Creating test data" << std::endl;
    std::vector<std::vector<int>> grid_map(40, std::vector<int>(40, 0));
    for (int i = 0; i < 35; ++i) {
        grid_map[20][i] = 1;
    }
    std::srand(0); // Seed random number generator
    std::vector<std::vector<int>> edge_cost(40, std::vector<int>(40));
    for (int i = 0; i < 40; ++i) {
        for (int j = 0; j < 40; ++j) {
            edge_cost[i][j] = std::rand() % 10;
        }
    }

    std::vector<std::pair<int, int>> start_pos{{0, 0},
                                               {0, 1},
                                               {0, 2},
                                               {0, 3}};
    std::vector<std::pair<int, int>> goal_pos{{39, 39},
                                              {39, 38},
                                              {39, 37},
                                              {39, 36}};
    std::vector<int> priority{0, 1, 2, 3};

    std::string instance_name = "Name";
    int seed = 0;
    int max_comp_time = 100;
    int max_timestep = 100;
    int num_agents = 4;
    bool grid_with_speed = true;

    std::cout << "Creating problem instance" << std::endl;

    MAPF_Instance problem(instance_name, seed, max_comp_time, max_timestep, num_agents, grid_with_speed,
                          grid_map,
                          edge_cost, edge_cost, edge_cost, edge_cost);

    std::cout << "Setting config" << std::endl;
    problem.setConfig(
            start_pos,
            goal_pos,
            priority
    );

    std::cout << "Creating pibt solver" << std::endl;
    PIBT pibt(&problem);
    std::cout << "Solving" << std::endl;
    pibt.solve();
    std::cout << "Done solving" << std::endl;
    if (pibt.succeed() && !pibt.getSolution().validate(&problem)) {
        std::cout << "error@mapf: invalid results" << std::endl;
        return 0;
    }
    pibt.printResult();

    return 0;
}