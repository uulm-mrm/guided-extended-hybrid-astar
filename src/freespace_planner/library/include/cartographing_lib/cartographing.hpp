//
// Created by schumann on 5/12/23.
//

#ifndef CARTOGRAPHING_HPP
#define CARTOGRAPHING_HPP

#include "util_lib/data_structures1.hpp"
#include "util_lib/util2.hpp"

#include "collision_checker_lib/collision_checking.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

class Cartographing
{
private:
  inline static size_t patch_dim_ = 0;
  inline static Vec2DFlat<uint8_t> patch_arr_;
  inline static Vec2DFlat<uint8_t> temp_patch_;
  inline static Vec2DFlat<uint8_t> local_map_;

public:
  static void resetPatch(size_t patch_dim);

  static void cartograph(const py::array_t<uint8_t>& local_map, const Point<int>& origin, int dim);

  static void cartograph(const Vec2DFlat<uint8_t>& local_map_data, const Point<int>& origin, int dim);

  static void passLocalMap(const Point<int>& origin, int dim);

  static void loadPrevPatch(const Point<double>& prev_origin_utm, const Point<double>& origin_utm);

  static py::array_t<uint8_t> getMap();
};

#endif  // CARTOGRAPHING_HPP
