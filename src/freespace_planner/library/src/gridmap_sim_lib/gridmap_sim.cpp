//
// Created by schumann on 12.09.22.
//

#include "gridmap_sim_lib/gridmap_sim.hpp"

void GridMapSim::initialize()
{
  calculatePrecasts();
  is_initialized_ = true;
}

void GridMapSim::calculatePrecasts()
{
  for (size_t i_x = 0; i_x < GM_DIM; ++i_x)
  {
    for (size_t i_y = 0; i_y < GM_DIM; ++i_y)
    {
      // Calculate properties of precast
      const int precast_x = static_cast<int>(i_x) - middle_;
      const int precast_y = static_cast<int>(i_y) - middle_;
      const double dist = hypot(precast_x, precast_y);
      const double angle = atan2Pi(precast_y, precast_x);
      const int angle_id1 = static_cast<int>(floor(angle / YAW_RES));

      // Create precast object
      const PrecastDB precast = PrecastDB(dist, static_cast<int>(i_x), static_cast<int>(i_y));
      precast_list_[angle_id1].push_back(std::move(precast));
    }
  }

  // Sort precasts after distance
  for (auto& precast_vec : precast_list_)
  {
    std::sort(precast_vec.begin(), precast_vec.end());
  }
}

void GridMapSim::raytracing(const py::array_t<uint8_t>& grid_map_gt)
{
  auto grid_map_data = grid_map_gt.unchecked<2>();

  std::for_each(std::execution::unseq, precast_list_.begin(), precast_list_.end(), [&grid_map_data](const auto& prec) {
    int i_el = 0;
    bool coll_found = false;
    for (const auto element : prec)
    {
      // Collision found in grid map
      if (grid_map_data(element.ix_, element.iy_) == OCCUPIED)
      {
        pmap_[element.ix_][element.iy_] = OCCUPIED;
        coll_found = true;
        // mark all elements behind it as unknown
        for (auto it = prec.begin() + i_el + 1; it != prec.end(); ++it)
        {
          pmap_[it->ix_][it->iy_] = UNKNOWN;
        }
        // Set all elements before as free
        for (auto iter = prec.begin(); iter != prec.begin() + i_el; ++iter)
        {
          pmap_[iter->ix_][iter->iy_] = FREE;
        }
        break;
      }
      i_el++;
    }
    // If no collision exists, whole precast is free
    if (!coll_found)
    {
      for (const auto& iter : prec)
      {
        pmap_[iter.ix_][iter.iy_] = FREE;
      }
    }
  });
}

double GridMapSim::atan2Pi(double y_coord, double x_coord)
{
  double angle = atan2(y_coord, x_coord);
  if (angle < 0)
  {
    angle += 2 * PI;
  }
  return angle;
}

py::array_t<uint8_t> GridMapSim::gmSimulation(const py::array_t<uint8_t>& grid_map_gt)
{
  if (not is_initialized_)
  {
    initialize();
  }

  raytracing(grid_map_gt);

  // View as mat object (no copy)
  cv::Mat mat_pmap(GM_DIM, GM_DIM, CV_8UC1, pmap_.data());

  // Floodfill from middle point
  cv::Mat freespace = mat_pmap.clone();
  cv::floodFill(freespace, cv::Point(middle_, middle_), cv::Scalar(PLACEHOLDER));

  // Get freespace in binary form
  const cv::Mat place_mask = freespace == PLACEHOLDER;
  freespace.setTo(OCCUPIED, 1 - place_mask);
  freespace.setTo(FREE, place_mask);

  // dilate free space, so it reaches over the collisions at the border
  cv::Mat free_dilated;
  cv::erode(freespace, free_dilated, cv::Mat());

  cv::Mat collision_mask = mat_pmap == OCCUPIED;
  cv::Mat not_free_mask = free_dilated != FREE;
  cv::Mat free_mask = freespace == FREE;
  mat_pmap.setTo(FREE, free_mask);
  mat_pmap.setTo(UNKNOWN, not_free_mask);

  cv::dilate(mat_pmap, mat_pmap, kernel_);
  cv::erode(mat_pmap, mat_pmap, kernel_);

  return py::array_t<uint8_t>({ GM_DIM, GM_DIM }, mat_pmap.data);
}