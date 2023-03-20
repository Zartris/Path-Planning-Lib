#include "pybind11/pybind11.h"
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "planners/include/problem.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(path_planning_lib, m) {
    m.doc() = R"pbdoc(
        Pybind11 module of different path planning algorithms implemented in cpp for use in python
    )pbdoc";
    py::class_<MAPF_Persistent>(m, "MAPFPersistentProblemDefinition")
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
            .def("setConfig", &MAPF_Persistent::setConfig, "Set the configuration of the problem", py::arg("start_pos"),
                 py::arg("goal_pos"), py::arg("priority"))
            .def("setWellFormedInstance", &MAPF_Persistent::setWellFormedInstance,
                 "Set the configuration of the problem")
            .def("setRandomStartsGoals", &MAPF_Persistent::setRandomStartsGoals,
                 "Set the configuration of the problem")
            .def("setMaxCompTime", [](// lambda function for converting to numpy
                    MAPF_Persistent &self, // reference to the class we are using
                    const int t) {
                self.setMaxCompTime(t);
            }, "Set the max computation time", py::arg("t"));


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
