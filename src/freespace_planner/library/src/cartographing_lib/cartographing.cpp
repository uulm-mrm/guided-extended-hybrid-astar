//
// Created by schumann on 5/12/23.
//

#include "cartographing_lib/cartographing.hpp"

void Cartographing::resetPatch(size_t patch_dim)
{
  util::saveTemp(patch_arr_, temp_patch_, static_cast<uint8_t>(CollisionChecker::SENSOR_UNKNOWN));

  // reset previous patch_info
  patch_dim_ = patch_dim;
  patch_arr_.resize_and_reset(patch_dim_, patch_dim_, CollisionChecker::SENSOR_UNKNOWN);
  patch_arr_.setName("patch_arr_");
}

void Cartographing::cartograph(const py::array_t<uint8_t>& local_map, const Point<int>& origin, int dim)
{
  auto local_map_data = local_map.unchecked<2>();

  std::vector<uint8_t> values;
  for (int x_idx = 0; x_idx < dim; ++x_idx)
  {
    for (int y_idx = 0; y_idx < dim; ++y_idx)
    {
      const Point<int> loc_point(x_idx, y_idx);
      const Point<int> patch_point = origin + loc_point;

      // coordinate out of patch_info
      if (patch_point.x >= patch_dim_ or patch_point.x < 0 or patch_point.y >= patch_dim_ or patch_point.y < 0)
      {
        continue;
      }
      // get values of local map and patch_info
      const uint8_t patch_val = patch_arr_(patch_point);

      // only overwrite unknown parts in sim
      if (patch_val != CollisionChecker::SENSOR_UNKNOWN)
      {
        continue;
      }
      const uint8_t local_val = local_map_data(loc_point.y, loc_point.x);
      // only overwrite with FREE or OCC
      if (local_val == CollisionChecker::SENSOR_FREE or local_val == CollisionChecker::SENSOR_OCC)
      {
        patch_arr_(patch_point) = local_val;
      }
    }
  }
}

void Cartographing::cartograph(const Vec2DFlat<uint8_t>& local_map_data, const Point<int>& origin, int dim)
{
  //  auto local_map_data = local_map.unchecked<2>();

  std::vector<uint8_t> values;
  for (int x_idx = 0; x_idx < dim; ++x_idx)
  {
    for (int y_idx = 0; y_idx < dim; ++y_idx)
    {
      const Point<int> loc_point(x_idx, y_idx);
      const Point<int> patch_point = origin + loc_point;

      // coordinate out of patch_info
      if (patch_point.x >= patch_dim_ or patch_point.x < 0 or patch_point.y >= patch_dim_ or patch_point.y < 0)
      {
        continue;
      }
      // get values of local map and patch_info
      const uint8_t patch_val = patch_arr_(patch_point);

      // only overwrite unknown parts in sim
      if (patch_val != CollisionChecker::SENSOR_UNKNOWN)
      {
        continue;
      }
      const uint8_t local_val = local_map_data(loc_point.y, loc_point.x);
      // only overwrite with FREE or OCC
      if (local_val == CollisionChecker::SENSOR_FREE or local_val == CollisionChecker::SENSOR_OCC)
      {
        patch_arr_(patch_point) = local_val;
      }
    }
  }
}

void Cartographing::passLocalMap(const Point<int>& origin, int dim)
{
  // Copy values to local map
  local_map_.resize_and_reset(dim, dim, CollisionChecker::SENSOR_UNKNOWN);
  local_map_.setName("local_map_");

  // Get local map from cartographed one
  const int data_size = sizeof(uint8_t);

  for (size_t i = 0; i < dim; ++i)
  {
    const size_t dest_start_index = i * dim;
    const size_t src_start_index = patch_dim_ * origin.y + i * patch_dim_ + origin.x;

    std::memcpy(local_map_.getPtr() + dest_start_index * data_size,
                patch_arr_.getPtr() + src_start_index * data_size,
                dim * data_size);
  }
  // pass cartographed data to collision checker
  CollisionChecker::passLocalMapData(local_map_.getPtr(), origin, dim);
}

void Cartographing::loadPrevPatch(const Point<double>& prev_origin_utm, const Point<double>& origin_utm)
{
  const Point<int> prev_origin_gm = (prev_origin_utm * grid_tf::con2gm_).toInt();
  const Point<int> next_origin_gm = (origin_utm * grid_tf::con2gm_).toInt();

  util::copyPatch2Patch(prev_origin_gm, next_origin_gm, patch_arr_, temp_patch_);

  // Copy cartographed patch_info to collision checker
  const Point<int> origin(0, 0);
  CollisionChecker::passLocalMapData(patch_arr_.getPtr(), origin, static_cast<int>(patch_dim_));
}

py::array_t<uint8_t> Cartographing::getMap()
{
  return py::array_t<uint8_t>({ patch_dim_, patch_dim_ },  // shape
                              patch_arr_.getPtr());
}
