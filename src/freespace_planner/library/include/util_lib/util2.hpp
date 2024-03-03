//
// Created by schumann on 8/8/23.
//

#ifndef FREESPACE_PLANNER_UTIL2_HPP
#define FREESPACE_PLANNER_UTIL2_HPP

#include "data_structures2.hpp"
#include "deps_lib/BSpline1D.hpp"

namespace util
{
double getBilinInterp(double x, double y, const Vec2DFlat<double>& grid);

std::vector<Point<int>> drawline(const Point<int>& start_p, const Point<int>& end_p);

bool point_in_poly(const Polygon& polygon, const Point<double>& point);

template <typename T>
void saveTemp(Vec2DFlat<T>& arr, Vec2DFlat<T>& temp_arr, T val)
{
  if (!arr.is_empty())
  {
    // resize temp to size of old one
    const auto [x_dim, y_dim] = arr.getDims();
    temp_arr.resize_and_reset(x_dim, y_dim, val);
    // fill with values
    std::memcpy(temp_arr.getPtr(), arr.getPtr(), x_dim * y_dim * sizeof(T));
  }
}

template <typename T>
void copyPatch2Patch(const Point<int>& prev_origin_grid,
                     const Point<int>& origin_grid,
                     Vec2DFlat<T>& arr,
                     Vec2DFlat<T> temp_arr)
{
  const auto [prev_dim, unused1] = temp_arr.getDims();
  const auto [next_dim, unused2] = arr.getDims();

  const Point<int> prev_opp_origin_gm = prev_origin_grid + prev_dim;

  // start and end points of patches
  const Point<int> start_prev(std::max(origin_grid.x - prev_origin_grid.x, 0),
                              std::max(origin_grid.y - prev_origin_grid.y, 0));

  const Point<int> start_next(std::max(prev_origin_grid.x - origin_grid.x, 0),
                              std::max(prev_origin_grid.y - origin_grid.y, 0));

  const Point<int> end_next(std::min(prev_opp_origin_gm.x - origin_grid.x, next_dim - 1),
                            std::min(prev_opp_origin_gm.y - origin_grid.y, next_dim - 1));

  // copy parts of temp one to new one
  const Point<int> overlap_dims = end_next - start_next;

  for (int x_idx = 0; x_idx < overlap_dims.x; ++x_idx)
  {
    for (int y_idx = 0; y_idx < overlap_dims.y; ++y_idx)
    {
      const Point<int> rel_shift(x_idx, y_idx);
      const Point<int> next_patch_idx = start_next + rel_shift;
      const Point<int> prev_patch_idx = start_prev + rel_shift;

      arr(next_patch_idx) = temp_arr(prev_patch_idx);
    }
  }
}

double getPathLength(const std::vector<double>& x_list, const std::vector<double>& y_list);

}  // namespace util
#endif  // FREESPACE_PLANNER_UTIL2_HPP
