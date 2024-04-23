#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "jump_controller/jump_interface.h"

namespace py = pybind11;

PYBIND11_MODULE(jump_interface_py, m)
{
    py::class_<JumpInterface>(m, "JumpInterface")
        .def(py::init<const std::string &, py::object &>())
        .def_readwrite("statesM", &JumpInterface::StatesMatrix)
        .def_readwrite("statesV", &JumpInterface::StatesVector)
        .def_readwrite("action", &JumpInterface::action);
}