#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "tsp_solver.cpp"

namespace py = pybind11;

PYBIND11_MODULE(cpp_utils, m)
{
    m.def(
        "tsp_solver", &tsp_solver,
        py::arg("cities"), py::kw_only(), py::arg("first") = 0, py::arg("heuristic_hint") = std::optional<std::vector<unsigned>>(),
        py::call_guard<py::gil_scoped_release>());
}