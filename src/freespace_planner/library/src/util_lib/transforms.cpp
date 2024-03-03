//
// Created by schumann on 8/29/23.
//

#include "util_lib/transforms.hpp"

void grid_tf::updateTransforms(const Point<double>& patch_origin_utm)
{
  updatePatchOrigin(patch_origin_utm);
}

void grid_tf::updatePatchOrigin(const Point<double>& patch_origin_utm)
{
  patch_origin_utm_ = patch_origin_utm;
}

Pose<double> grid_tf::utm2patch_utm(const Pose<double>& pose)
{
  return pose - patch_origin_utm_;
}

Point<double> grid_tf::utm2patch_utm(const Point<double>& point)
{
  return point - patch_origin_utm_;
}

std::pair<std::vector<double>, std::vector<double>> grid_tf::utm2patch_utm(const std::vector<double>& x_vec,
                                                                           const std::vector<double>& y_vec)
{
  return { utm2patch_utm_x(x_vec), utm2patch_utm_y(y_vec) };
}

/// patch utm 2 utm
Pose<double> grid_tf::patch_utm2utm(const Pose<double>& pose)
{
  return pose + patch_origin_utm_;
}

Point<double> grid_tf::patch_utm2utm(const Point<double>& point)
{
  return point + patch_origin_utm_;
}

std::pair<std::vector<double>, std::vector<double>> grid_tf::patch_utm2utm(const std::vector<double>& x_vec,
                                                                           const std::vector<double>& y_vec)
{
  return { patch_utm2utm_x(x_vec), patch_utm2utm_y(y_vec) };
}

Path grid_tf::patch_utm2utm(const Path& path)
{
  Path path_utm = path;  // copy to keep the other properties

  path_utm.x_list = patch_utm2utm_x(path.x_list);  // TODO (Schumann) here two copies are made.
  path_utm.y_list = patch_utm2utm_y(path.y_list);

  return path_utm;
}

/// patch utm 2 global grid
Pose<double> grid_tf::patch_utm2global_gm(const Pose<double>& pose)
{
  return utm2grid(patch_utm2utm(pose));
}

Point<double> grid_tf::patch_utm2global_gm(const Point<double>& point)
{
  return utm2grid(patch_utm2utm(point));
}

std::pair<std::vector<double>, std::vector<double>> grid_tf::patch_utm2global_gm(const std::vector<double>& x_vec,
                                                                                 const std::vector<double>& y_vec)
{
  // Here, the transform must be explicitly written as two functions must be applied, which is not possible by
  // overloading or function passing
  std::vector<double> x_vec_new(x_vec.size());
  std::vector<double> y_vec_new(y_vec.size());
  std::transform(std::execution::unseq, x_vec.begin(), x_vec.end(), x_vec_new.begin(), [](const double& x_val) {
    return utm2grid(patch_utm2utm_x(x_val));
  });

  std::transform(std::execution::unseq, y_vec.begin(), y_vec.end(), y_vec_new.begin(), [](const double& y_val) {
    return utm2grid(patch_utm2utm_y(y_val));
  });

  return { x_vec_new, y_vec_new };
}

/// utm 2 grid
std::pair<std::vector<double>, std::vector<double>> grid_tf::utm2global_gm(const std::vector<double>& x_vec,
                                                                           const std::vector<double>& y_vec)
{
  return { utm2grid(x_vec), utm2grid(y_vec) };
}
