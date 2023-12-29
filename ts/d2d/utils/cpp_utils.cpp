#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "config.cpp"

namespace py = pybind11;

PYBIND11_MODULE(cpp_utils, m)
{
    m.def(
        "import_truck_config", &config::TruckConfig::import,
        py::kw_only(), py::arg("maximum_velocity"), py::arg("m_t"), py::arg("coefficients"),
        py::call_guard<py::gil_scoped_release>());
    m.def(
        "import_drone_linear_config", &config::DroneLinearConfig::import,
        py::kw_only(), py::arg("takeoff_speed"), py::arg("cruise_speed"), py::arg("landing_speed"), py::arg("altitude"), py::arg("capacity"), py::arg("battery"), py::arg("speed_type"), py::arg("range"),
        py::arg("beta"), py::arg("gamma"),
        py::call_guard<py::gil_scoped_release>());
    m.def(
        "import_drone_nonlinear_config", &config::DroneNonlinearConfig::import,
        py::kw_only(), py::arg("takeoff_speed"), py::arg("cruise_speed"), py::arg("landing_speed"), py::arg("altitude"), py::arg("capacity"), py::arg("battery"), py::arg("speed_type"), py::arg("range"),
        py::arg("k1"), py::arg("k2"), py::arg("c1"), py::arg("c2"), py::arg("c4"), py::arg("c5"),
        py::call_guard<py::gil_scoped_release>());
    m.def(
        "import_drone_endurance_config", &config::DroneEnduranceConfig::import,
        py::kw_only(), py::arg("speed_type"), py::arg("range"), py::arg("capacity"), py::arg("fixed_time"), py::arg("fixed_distance"), py::arg("drone_speed"),
        py::call_guard<py::gil_scoped_release>());
    m.def(
        "import_customers", &config::Customer::import,
        py::kw_only(), py::arg("x"), py::arg("y"), py::arg("demands"), py::arg("dronable"), py::arg("drone_service_time"), py::arg("technician_service_time"),
        py::call_guard<py::gil_scoped_release>());
    m.def(
        "calculate_drone_arrival_timestamps", &calculate_drone_arrival_timestamps,
        py::arg("path"), py::kw_only(), py::arg("config_type"), py::arg("offset"),
        py::call_guard<py::gil_scoped_release>());
    m.def(
        "calculate_technician_arrival_timestamps", &calculate_technician_arrival_timestamps,
        py::arg("path"),
        py::call_guard<py::gil_scoped_release>());
    m.def(
        "calculate_drone_total_waiting_time", &calculate_drone_total_waiting_time,
        py::arg("path"), py::kw_only(), py::arg("arrival_timestamps"),
        py::call_guard<py::gil_scoped_release>());
    m.def(
        "calculate_technician_total_waiting_time", &calculate_technician_total_waiting_time,
        py::arg("path"), py::kw_only(), py::arg("arrival_timestamps"),
        py::call_guard<py::gil_scoped_release>());
}