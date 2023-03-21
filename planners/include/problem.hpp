#pragma once

#include "graph.hpp"
#include <random>

#include "default_params.hpp"
#include "task.hpp"
#include "util.hpp"

using Config = std::vector<Node *>;  // < loc_0[t], loc_1[t], ... >
using Configs = std::vector<Config>;
using Priorities = std::vector<int>;  // < priority_0[t], priority_1[t], ... >

// check two configurations are same or not
[[maybe_unused]] static bool sameConfig(const Config &config_i,
                                        const Config &config_j) {
    if (config_i.size() != config_j.size()) return false;
    const int size_i = config_i.size();
    for (int k = 0; k < size_i; ++k) {
        if (config_i[k] != config_j[k]) return false;
    }
    return true;
}

[[maybe_unused]] static int getPathCost(const Path &path) {
    int cost = path.size() - 1;
    auto itr = path.end() - 1;
    while (itr != path.begin() && *itr == *(itr - 1)) {
        --cost;
        --itr;
    }
    return cost;
}

class Problem {
protected:
    std::string instance;  // instance name
    Graph *G;              // graph
    std::mt19937 *MT;      // seed
    Config config_s;       // initial configuration
    Config config_g;       // goal configuration
    std::vector<int> config_p;       // priority configuration
    int num_agents;        // number of agents
    int max_timestep;      // timestep limit
    int max_comp_time;     // comp_time limit, ms
    bool grid_with_speed; // Type of grid
    // utilities
    void halt(const std::string &msg) const;

    void warn(const std::string &msg) const;

public:
    Problem() {};

    Problem(const std::string &_instance) : instance(_instance) {}

    Problem(std::string _instance, Graph *_G, std::mt19937 *_MT, Config _config_s,
            Config _config_g, std::vector<int> _config_p, int _num_agents, int _max_timestep,
            int _max_comp_time);

    ~Problem() {};

    Graph *getG() { return G; }

    int getNum() { return num_agents; }

    std::mt19937 *getMT() { return MT; }

    Node *getStart(int i) const;  // return start of a_i
    Node *getGoal(int i) const;   // return  goal of a_i
    Config getConfigStart() const { return config_s; };

    Config getConfigGoal() const { return config_g; };

    Priorities getConfigPriority() const { return config_p; };

    int getMaxTimestep() { return max_timestep; };

    int getMaxCompTime() { return max_comp_time; };

    std::string getInstanceFileName() { return instance; };

    void setMaxCompTime(const int t) { max_comp_time = t; }
};

class MAPF_Instance : public Problem {
private:
    const bool instance_initialized;  // for memory manage



public:
    MAPF_Instance(const std::string &_instance);

    MAPF_Instance(const std::string &_instance_name, int _seed,
                  int _max_comp_time, int _max_timestep,
                  int _num_agents, bool _grid_with_speed,
                  const std::vector<std::vector<int>> &grid_map,
                  const std::vector<std::vector<int>> &edge_cost_moving_up,
                  const std::vector<std::vector<int>> &edge_cost_moving_down,
                  const std::vector<std::vector<int>> &edge_cost_moving_left,
                  const std::vector<std::vector<int>> &edge_cost_moving_right);

    MAPF_Instance(MAPF_Instance *P, Config _config_s, Config _config_g, std::vector<int> _config_p,
                  int _max_comp_time, int _max_timestep);

    MAPF_Instance(MAPF_Instance *P, int _max_comp_time);

    ~MAPF_Instance();

    void setConfig(const std::vector<std::pair<int, int>> &start_pos,
                   const std::vector<std::pair<int, int>> &goal_pos,
                   const std::vector<int> &priority);

    // set starts and goals randomly
    void setRandomStartsGoals();

    // set well-formed instance
    void setWellFormedInstance();

    bool isInitializedInstance() const { return instance_initialized; }

    // used when making new instance file
    void makeScenFile(const std::string &output_file);
};

class MAPD_Instance : public Problem {
private:
    float task_frequency;
    int task_num;

    int current_timestep;  // current timestep
    Tasks TASKS_OPEN;
    Tasks TASKS_CLOSED;

    Nodes LOCS_PICKUP;             // candidates of pickup locations
    Nodes LOCS_DELIVERY;           // candidates of delivery locations
    Nodes LOCS_NONTASK_ENDPOINTS;  // endpoints, not necessary for PIBT
    Nodes LOCS_ENDPOINTS;          // pickup, delivery, nontasks

    bool specify_pickup_deliv_locs;

    void setupSpetialNodes();

public:
    MAPD_Instance(const std::string &_instance);

    ~MAPD_Instance();

    void update();

    int getCurrentTimestep() const { return current_timestep; }

    float getTaskFrequency() const { return task_frequency; }

    float getTaskNum() const { return task_num; }

    Tasks getOpenTasks() { return TASKS_OPEN; }

    Tasks getClosedTasks() { return TASKS_CLOSED; }

    Nodes getEndpoints() { return LOCS_ENDPOINTS; }
};
