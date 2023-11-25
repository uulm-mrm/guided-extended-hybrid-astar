//
// Created by schumann on 12.09.22.
//

#include <pybind11/pybind11.h>

#include "gridmap_sim_lib/gridmap_sim.hpp"

namespace py = pybind11;

// The module name (example) is given as the first macro argument (it should not be in quotes).
// The second argument (m) defines a variable of type py::module_ which is the main interface for creating bindings
// The method module_::def() generates binding code that exposes the add() function to Python.
PYBIND11_MODULE(_gridmap_sim_lib_api, m)
{
  py::class_<GridMapSim>(m, "GridMapSim").def("gmSimulation", &GridMapSim::gmSimulation);
}
