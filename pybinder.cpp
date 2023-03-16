#include "pybind11/pybind11.h"
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "source/AStar.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(path_planning, m) {
    m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------
        .. currentmodule:: cmake_example
        .. autosummary::
           :toctree: _generate
           add
           subtract
    )pbdoc";
    py::class_<AStar::Generator>(m, "AStarGenerator")
            .def(py::init<const std::vector<std::vector<int>> &>(), "Constructor for AStarGenerator",
                 py::arg("gridmap"))
            .def("findPath", [](// lambda function for converting to numpy
                         AStar::Generator &self, // reference to the class we are using
                         int start_x, int start_y,
                         int goal_x, int goal_y) {
                     auto solution = self.findPath({start_x, start_y}, {goal_x, goal_y});
                     py::array out = py::cast(solution.path);
                     return py::make_tuple(solution.solution_found, out);
                 }, "Find path from start to goal returning a numpy array", py::arg("start_x"), py::arg("start_y"),
                 py::arg("goal_x"), py::arg("goal_y"));
    m.attr("__version__") = "dev";
}

// https://www.youtube.com/watch?v=_5T70cAXDJ0&t=480s for classes and numpy usage
// https://www.youtube.com/watch?v=H2wOlriHGmM for pybind11 and cmake
