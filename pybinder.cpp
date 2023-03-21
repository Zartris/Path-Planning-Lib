#include "pybind11/pybind11.h"
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "planners/include/problem.hpp"
#include "pibt.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(path_planning_lib, m) {
    m.doc() = R"pbdoc(
        Pybind11 module of different path planning algorithms implemented in cpp for use in python
    )pbdoc";
// ============= MAPF_Instance =================
    py::class_<MAPF_Instance>(m, "MAPFProblemDefinition")
            .def(py::init<const std::string &,
                         int,
                         int, int,
                         int, bool,
                         const std::vector<std::vector<int>> &,
                         const std::vector<std::vector<int>> &,
                         const std::vector<std::vector<int>> &,
                         const std::vector<std::vector<int>> &,
                         const std::vector<std::vector<int>> &>(), "Constructor for mapf persistent class",
                 py::arg("_instance_name"),
                 py::arg("_seed"),
                 py::arg("_max_comp_time"),
                 py::arg("_max_timestep"),
                 py::arg("_num_agents"),
                 py::arg("_grid_with_speed"),
                 py::arg("grid_map"),
                 py::arg("edge_cost_moving_up"),
                 py::arg("edge_cost_moving_down"),
                 py::arg("edge_cost_moving_left"),
                 py::arg("edge_cost_moving_right"))
            .def("setConfig", &MAPF_Instance::setConfig, "Set the configuration of the problem", py::arg("start_pos"),
                 py::arg("goal_pos"), py::arg("priority"))
            .def("setWellFormedInstance", &MAPF_Instance::setWellFormedInstance,
                 "Set the configuration of the problem")
            .def("setRandomStartsGoals", &MAPF_Instance::setRandomStartsGoals,
                 "Set the configuration of the problem")
            .def("setMaxCompTime", [](// lambda function for converting to numpy
                    MAPF_Instance &self, // reference to the class we are using
                    const int t) {
                self.setMaxCompTime(t);
            }, "Set the max computation time", py::arg("t"));

// ============= NODE CLASS =================
    py::class_<Node>(m, "Node")
            .def(py::init<int &, int &, int &>(), "Constructor for node class", py::arg("_id"), py::arg("x"),
                 py::arg("y"))
            .def("__str__", [](const Node &self) {
                return "id: " + std::to_string(self.getID()) + " x: " + std::to_string(self.getX()) + " y: " +
                       std::to_string(self.getY()) + '\n';
            }, "Print the node")
            .def("__repr__", [](const Node &self) {
                return "id: " + std::to_string(self.getID()) + " x: " + std::to_string(self.getX()) + " y: " +
                       std::to_string(self.getY()) + '\n';
            }, "Represent the node")
            .def("getID", &Node::getID, "Get the id of the node")
            .def("getX", &Node::getX, "Get the x coordinate of the node")
            .def("getY", &Node::getY, "Get the y coordinate of the node")
            .def("getNeighbors", &Node::getNeighbors, "Get the neighbors of the node")
            .def("getNeighborCosts", &Node::getNeighborCosts, "Get the neighbor costs of the node")
            .def("getDegree", &Node::getDegree, "Get the degree of the node")
            .def("manhattanDist", [](
                    const Node &self,
                    Node &node) {
                return self.manhattanDist(node);
            }, "Get the manhattan distance to another node", py::arg("node"))
            .def("manhattanDist", [](
                    const Node &self,
                    Node *node) {
                return self.manhattanDist(node);
            }, "Get the manhattan distance to another node", py::arg("node"))
            .def("euclideanDist", [](
                    const Node &self,
                    Node &node) {
                return self.euclideanDist(node);
            }, "Get the euclidean distance to another node", py::arg("node"))
            .def("euclideanDist", [](
                    const Node &self,
                    Node *node) {
                return self.euclideanDist(node);
            }, "Get the euclidean distance to another node", py::arg("node"));

// ============= PLAN CLASS =================
    py::class_<Plan>(m, "Plan")
            .def(py::init<>(), "Constructor for plan class")
            .def("getPathCost", &Plan::getPathCost, "Get the path cost for agent i", py::arg("agent_index"))
            .def("getSOC", &Plan::getSOC, "Get the SOC")
            .def("getMakespan", &Plan::getMakespan, "Get the makespan")
            .def("len", &Plan::size, "Get the length of the plan")
            .def("is_empty", &Plan::empty, "Check if the plan is empty")
            .def("getPath", [](// lambda function for converting to numpy
                    Plan &self, // reference to the class we are using
                    int agent_index) {
                auto path = self.getPath(agent_index);
                py::array out = py::cast(path);
                return out;
            }, "Get the path for agent i", py::arg("agent_index"))
            .def("getPathXY", [](// lambda function for converting to numpy
                    Plan &self, // reference to the class we are using
                    int agent_index) {
                auto path = self.getPath(agent_index);
                std::vector<std::pair<int, int>> path_xy;
                for (auto &p: path) {
                    path_xy.emplace_back(p->getX(), p->getY());
                }
                py::array out = py::cast(path_xy);
                return out;
            }, "Get the path for agent i", py::arg("agent_index"))
            .def("getAllPaths", [](// lambda function for converting to numpy
                    Plan &self) {
                auto num_agents = self.getNumAgents();
                std::cout << "num_agents: " << num_agents << std::endl;
                std::vector<std::vector<Node *>> paths;
                for (int i = 0; i < num_agents; i++) {
                    std::cout << "i: " << i << std::endl;
                    paths.push_back(self.getPath(i));
                }
                py::array out = py::cast(paths);
                return out;
            }, "Get all the paths")
            .def("getAllPathsXY", [](// lambda function for converting to numpy
                    Plan &self) {
                auto num_agents = self.getNumAgents();
                std::vector<std::vector<std::pair<int, int>>> paths;
                for (int i = 0; i < num_agents; i++) {
                    auto path = self.getPath(i);
                    std::vector<std::pair<int, int>> path_xy;
                    for (auto &p: path) {
                        path_xy.emplace_back(p->getX(), p->getY());
                    }
                    paths.push_back(path_xy);
                }
                py::array out = py::cast(paths);
                return out;
            }, "Get all the paths x and y coordinates")
            .def("is_valid", [](// lambda function for converting to numpy
                    Plan &self, // reference to the class we are using
                    MAPF_Instance P) {
                return self.validate(&P);
            }, "Validate the plan", py::arg("problem_definition"));

// ============= PIBT SOLVER =================
    py::class_<PIBT>(m, "PIBTSolver")
            .def(py::init<MAPF_Instance *>(), "Constructor for PIBT class", py::arg("problem_definition"))
            .def("solve", &PIBT::solve, "Solve the problem")
            .def("succeed", &PIBT::succeed, "Check if the problem was solved")
            .def("getSolution", &PIBT::getSolution, "Get the solution")
            .def("printResult", [](PIBT &self) { self.printResult(); }, "Print the results");




//            .def("findPath", [](// lambda function for converting to numpy
//                         AStar::Generator &self, // reference to the class we are using
//                         int start_x, int start_y,
//                         int goal_x, int goal_y) {
//                     auto solution = self.findPath({start_x, start_y}, {goal_x, goal_y});
//                     py::array out = py::cast(solution.path);
//                     return py::make_tuple(solution.solution_found, out);
//                 }, "Find path from start to goal returning a numpy array", py::arg("start_x"), py::arg("start_y"),
//                 py::arg("goal_x"), py::arg("goal_y"));
    m.attr("__version__") = "dev";
}

// https://www.youtube.com/watch?v=_5T70cAXDJ0&t=480s for classes and numpy usage
// https://www.youtube.com/watch?v=H2wOlriHGmM for pybind11 and cmake
