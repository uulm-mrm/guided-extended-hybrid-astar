//
// Created by schumann on 12.09.22.
//

#include "gridmap_sim_lib/gridmap_sim.hpp"

void GridMapSim::initialize()
{
  calculatePrecasts();
  is_initialized_ = true;
  pmap_.setTo(UNKNOWN);
}

void GridMapSim::calculatePrecasts()
{
  // iterate through all points
  for (int i_x = 0; i_x < GM_DIM; ++i_x)
  {
    for (int i_y = 0; i_y < GM_DIM; ++i_y)
    {
      // Calculate properties of precast
      const double dist = hypot(i_x - middle_, i_y - middle_);
      const double angle = atan2Pi(i_y - middle_, i_x - middle_);
      const int angle_id1 = std::min(static_cast<int>(floor(angle / YAW_RES)), NB_PRECASTS - 1);
      const int angle_id2 = std::min(static_cast<int>(ceil(angle / YAW_RES)), NB_PRECASTS - 1);

      // Create precast object
      const PrecastDB precast = PrecastDB(dist, static_cast<int>(i_x), static_cast<int>(i_y));

      precast_list_[angle_id1].push_back(precast);
      precast_list_[angle_id2].push_back(precast);
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
  const auto grid_map_data = grid_map_gt.unchecked<2>();

  pmap_.setTo(UNKNOWN);

  for (const std::vector<PrecastDB>& prec : precast_list_)
  {
    int i_el = 0;
    bool coll_found = false;
    for (const PrecastDB& element : prec)
    {
      // Collision found in grid map
      if (grid_map_data(element.ix_, element.iy_) == OCCUPIED)
      {
        pmap_(element.ix_, element.iy_) = OCCUPIED;
        coll_found = true;
        // Set all elements before as free
        for (auto iter = prec.begin(); iter != prec.begin() + i_el; ++iter)
        {
          pmap_(iter->ix_, iter->iy_) = FREE;
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
        pmap_(iter.ix_, iter.iy_) = FREE;
      }
    }
  }
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
  // core function
  raytracing(grid_map_gt);

  return py::array_t<uint8_t>({ GM_DIM, GM_DIM }, pmap_.getPtr());
}