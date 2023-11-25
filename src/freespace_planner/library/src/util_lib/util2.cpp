//
// Created by schumann on 8/8/23.
//
#include "util_lib/util2.hpp"

namespace util
{

bool point_in_poly(const Polygon& polygon, const Point<double>& point)
{
  int i;
  int j;
  bool c = false;
  const int nb_vert = polygon.vertices.size();
  for (i = 0, j = nb_vert - 1; i < nb_vert; j = i++)
  {
    if (((polygon.vertices[i].y > point.y) != (polygon.vertices[j].y > point.y)) &&
        (point.x < (polygon.vertices[j].x - polygon.vertices[i].x) * (point.y - polygon.vertices[i].y) /
                           (polygon.vertices[j].y - polygon.vertices[i].y) +
                       polygon.vertices[i].x))
    {
      c = !c;
    }
  }
  return c;
}

/**
 * calculate the bilinear interpolation on a given grid
 * @param x
 * @param y
 * @param grid
 * @return
 */
double getBilinInterp(double x, double y, const Vec2DFlat<double>& grid)
{
  int x1 = static_cast<int>(x);  // floor
  int x2 = static_cast<int>(std::ceil(x));
  int y1 = static_cast<int>(y);  // floor
  int y2 = static_cast<int>(std::ceil(y));
  double q11 = grid(y1, x1);
  double q12 = grid(y2, x1);
  double q21 = grid(y1, x2);
  double q22 = grid(y2, x2);
  return bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
}

std::vector<Point<int>> drawline(const Point<int>& start_p, const Point<int>& end_p)
{
  int x = start_p.x;
  int y = start_p.y;

  int dx = abs(end_p.x - start_p.x);
  int sx = start_p.x < end_p.x ? 1 : -1;
  int dy = -abs(end_p.y - start_p.y);
  int sy = start_p.y < end_p.y ? 1 : -1;
  int error = dx + dy;

  std::vector<Point<int>> points;

  while (true)
  {
    points.emplace_back(x, y);
    if (start_p.x == end_p.x and start_p.y == end_p.y)
    {
      break;
    }

    int e2 = 2 * error;
    if (e2 >= dy)
    {
      if (x == end_p.x)
      {
        break;
      }
      error += dy;
      x += sx;
    }

    if (e2 <= dx)
    {
      if (y == end_p.y)
      {
        break;
      }
      error += dx;
      y += sy;
    }
  }
  return points;
}

std::tuple<double, double, double> getSumSquaredCurvatureChange(const Path& path, double interp_res)
{
  return getSumSquaredCurvatureChangeVec(path.yaw_list, interp_res);
}

std::tuple<double, double, double> getSumSquaredCurvatureChangeVec(const std::vector<double>& yaw_list,
                                                                   double interp_res)
{
  double sum_squared_curv_change = 0;
  double max_curv_change = 0;
  const double d_s = interp_res;

  if (yaw_list.size() < 3)
  {
    return { 0, 0, 0 };
  }

  size_t nb_elements = yaw_list.size() - 2;

  for (size_t i = 0; i < nb_elements; i++)
  {
    // get yaws
    double yaw_pp = yaw_list.at(i + 2);
    double yaw_p = yaw_list.at(i + 1);
    double yaw = yaw_list.at(i);
    // get kappas
    double kappa_p = util::getAngleDiff(yaw_pp, yaw_p) / d_s;
    double kappa = util::getAngleDiff(yaw_p, yaw) / d_s;
    // get change of kappa
    double curv_change = abs(kappa_p - kappa) / d_s;

    if (curv_change > max_curv_change)
    {
      max_curv_change = curv_change;
    }
    // sum up the squares
    sum_squared_curv_change += curv_change * curv_change;
  }

  double curv_change_rms = sqrt(sum_squared_curv_change / static_cast<double>(nb_elements));

  return { sum_squared_curv_change, curv_change_rms, max_curv_change };
}

double getPathLength(const std::vector<double>& x_list, const std::vector<double>& y_list)
{
  if (x_list.size() < 2)
  {
    return 0;
  }

  double cum_dist = 0;
  for (int i = 0; i < x_list.size() - 1; i++)
  {
    const double d_x = x_list[i + 1] - x_list[i];
    const double d_y = y_list[i + 1] - y_list[i];
    const double dist = sqrt(d_x * d_x + d_y * d_y);
    cum_dist += dist;
  }
  return cum_dist;
}

}  // namespace util