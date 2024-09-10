// This is the pybind11 wrapper for the ManipServer class. It is used to expose the class to Python.

// clang-format off
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
// #include <pybind11/eigen/tensor.h>
#include "table_top_manip/manip_server.h"

namespace py = pybind11;
using namespace RUT;
PYBIND11_MODULE(manip_server_pybind, m)
{
    py::class_<ManipServer>(m, "ManipServer")
        .def(py::init<>())
        .def("initialize", &ManipServer::initialize)
        .def("join_threads", &ManipServer::join_threads)
        .def("is_running", &ManipServer::is_running)
        .def("is_ready", &ManipServer::is_ready)
        .def("is_bimanual", &ManipServer::is_bimanual)
        .def("get_camera_rgb", &ManipServer::get_camera_rgb,
                py::arg(), py::arg("camera_id") = 0)
        .def("get_wrench", &ManipServer::get_wrench,
                py::arg(), py::arg("sensor_id") = 0)
        .def("get_pose", &ManipServer::get_pose,
                py::arg(), py::arg("robot_id") = 0)
        .def("get_camera_rgb_timestamps_ms", &ManipServer::get_camera_rgb_timestamps_ms,
                py::arg("id") = 0)
        .def("get_wrench_timestamps_ms", &ManipServer::get_wrench_timestamps_ms,
                py::arg("id") = 0)
        .def("get_pose_timestamps_ms", &ManipServer::get_pose_timestamps_ms,
                py::arg("id") = 0)
        .def("get_timestamp_now_ms", &ManipServer::get_timestamp_now_ms)
        .def("set_high_level_maintain_position", &ManipServer::set_high_level_maintain_position)
        .def("set_high_level_free_jogging", &ManipServer::set_high_level_free_jogging)
        .def("set_target_pose", &ManipServer::set_target_pose,
                py::arg(), py::arg(), py::arg("robot_id") = 0)
        .def("set_force_controlled_axis", &ManipServer::set_force_controlled_axis,
                py::arg(), py::arg(), py::arg("robot_id") = 0)
        .def("set_stiffness_matrix", &ManipServer::set_stiffness_matrix,
                py::arg(), py::arg("robot_id") = 0)
        .def("schedule_waypoints", &ManipServer::schedule_waypoints,
                py::arg(), py::arg(), py::arg("robot_id") = 0)
        .def("schedule_stiffness", &ManipServer::schedule_stiffness,
                py::arg(), py::arg(), py::arg("robot_id") = 0);
}
// clang-format on