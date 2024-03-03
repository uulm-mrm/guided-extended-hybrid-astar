//
// Created by schumann on 8/8/23.
//
#include "util_lib/util2.hpp"

namespace util
{

/**
 * Returns true if point was in polygon, used for geofencing
 * @param polygon
 * @param point
 * @return
 */
bool point_in_poly(const Polygon& polygon, const Point<double>& point)
{
  size_t i;
  size_t j;
  bool c = false;
  const size_t nb_vert = polygon.vertices.size();
  for (i = 0, j = nb_vert - 1; i < nb_vert; j = i++)  // don't ask me what is going on here
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
  const int x1 = std::max(static_cast<int>(x), 0);  // floor
  const int x2 = std::max(static_cast<int>(std::ceil(x)), 0);
  const int y1 = std::max(static_cast<int>(y), 0);  // floor
  const int y2 = std::max(static_cast<int>(std::ceil(y)), 0);
  const double q11 = grid(y1, x1);
  const double q12 = grid(y2, x1);
  const double q21 = grid(y1, x2);
  const double q22 = grid(y2, x2);
  return bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
}

/**
 * Bresenham algorithm
 * @param start_p
 * @param end_p
 * @return
 */
std::vector<Point<int>> drawline(const Point<int>& start_p, const Point<int>& end_p)
{
  int x = start_p.x;
  int y = start_p.y;

  const int dx = abs(end_p.x - start_p.x);
  const int sx = start_p.x < end_p.x ? 1 : -1;
  const int dy = -abs(end_p.y - start_p.y);
  const int sy = start_p.y < end_p.y ? 1 : -1;
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

/**
 * Return distance of path
 * @param x_list
 * @param y_list
 * @return
 */
double getPathLength(const std::vector<double>& x_list, const std::vector<double>& y_list)
{
  if (x_list.size() < 2)
  {
    return 0;
  }

  double cum_dist = 0;
  for (int i = 0; i < x_list.size() - 1; ++i)
  {
    const double d_x = x_list[i + 1] - x_list[i];
    const double d_y = y_list[i + 1] - y_list[i];
    const double dist = sqrt(d_x * d_x + d_y * d_y);
    cum_dist += dist;
  }
  return cum_dist;
}

}  // namespace util