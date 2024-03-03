//
// Created by schumann on 2/8/23.
//
#include "util_lib/util1.hpp"

namespace util
{
double constrainAngleZero2Pi(double angle)
{
  while (angle < 0)
  {
    angle += 2 * util::PI;
  }
  return fmod(angle, 2 * PI);
}

double constrainAngleMinPIPlusPi(double angle)
{
  while (angle < 0)
  {
    angle += 2 * util::PI;
  }
  return fmod(angle + PI, 2.0 * PI) - PI;
}

double getAngleDiff(double angle1, double angle2)
{
  angle2 = constrainAngleMinPIPlusPi(angle2);
  angle1 = constrainAngleMinPIPlusPi(angle1);

  const double diff = angle2 - angle1;

  return constrainAngleMinPIPlusPi(diff);
}

bool areAnglesEqual(double angle1, double angle2, double abs_tol = 1e-3)
{
  const double diff = getAngleDiff(angle1, angle2);
  return std::abs(diff) < abs_tol;
}

double getDrivenAngleDiff(double angle1, double angle2, char direction)
{
  double diff = util::getAngleDiff(angle1, angle2);

  if (direction == 'R' and diff >= 0)
  {
    diff -= 2 * util::PI;
  }
  else if (direction == 'L' and diff < 0)
  {
    diff += 2 * util::PI;
  }
  return diff;
}

/**
 * Get angles from (angle1, angle2], excluding angle1!
 * @param angle1
 * @param angle2
 * @param direction
 * @param angle_res
 * @return
 */
std::vector<double> angleArange(double angle1, double angle2, char direction, double angle_res)
{
  constexpr double eps = 1e-8;
  const double yaw_diff = getDrivenAngleDiff(angle1, angle2, direction);
  const double step = util::sgn(yaw_diff) * std::abs(angle_res);
  const size_t nb_steps = std::ceil(std::abs(yaw_diff) / std::abs(angle_res) - eps);

  std::vector<double> angles;
  angles.reserve(nb_steps);
  // set first angle
  double angle = angle1;
  for (size_t i = 0; i < nb_steps; ++i)
  {
    angle += step;
    angle = util::constrainAngleMinPIPlusPi(angle);
    angles.push_back(angle);
  }
  // set last angle, because loop did one more!
  angles.back() = angle2;
  return angles;
}

/**
 * lerp of euler angles
 * @param t
 * @param yaw1
 * @param yaw2
 * @return
 */
double angleLerp(double t, double yaw1, double yaw2)
{
  t = std::clamp(t, 0.0, 1.0);  // no extrapolation
  const double angle_diff = util::getAngleDiff(yaw1, yaw2);
  return util::constrainAngleMinPIPlusPi(yaw1 + t * angle_diff);
}

/**
 * interpolate angles
 * @param s
 * @param s_list
 * @param yaw_list
 * @return
 */
double angleInterpolation(double s, const std::vector<double>& s_list, const std::vector<double>& yaw_list)
{
  // catch edge cases
  if (s >= s_list.back())
  {
    return yaw_list.back();
  }
  if (s <= s_list.front())
  {
    return yaw_list.front();
  }

  size_t idx = 0;
  while (s_list[idx] <= s)
  {
    idx++;
  }

  const size_t idxm = idx - 1;
  const size_t idxp = idx;
  const double sm = s_list[idxm];
  const double t = s - sm;

  const double yaw1 = util::constrainAngleMinPIPlusPi(yaw_list[idxm]);
  const double yaw2 = util::constrainAngleMinPIPlusPi(yaw_list[idxp]);

  return angleLerp(t, yaw1, yaw2);
}

/**
 * Calculate the actual bilinear interpolatin.
 * If the point lies on one of the lines, do normal interpolation
 * @param q11
 * @param q12
 * @param q21
 * @param q22
 * @param x1
 * @param x2
 * @param y1
 * @param y2
 * @param x
 * @param y
 * @return
 */
double bilinearInterpolation(double q11,
                             double q12,
                             double q21,
                             double q22,
                             int x1,
                             int x2,
                             int y1,
                             int y2,
                             double x,
                             double y)
{
  const int x2x1 = x2 - x1;
  const int y2y1 = y2 - y1;
  const double yy1 = y - y1;
  const double xx1 = x - x1;
  constexpr double eps = 0.0001;

  double result;

  // dont interpolate
  if (abs(x2x1) == 0 and abs(y2y1) < eps)
  {
    result = q11;
  }
  else if (abs(x2x1) == 0)
  {
    // interpolate only in y direction
    const double t = yy1 / y2y1;
    result = std::lerp(q11, q12, t);
  }
  else if (abs(y2y1) < eps)
  {
    // interpolate only in x direction
    const double t = xx1 / x2x1;
    result = std::lerp(q11, q21, t);
  }
  else
  {
    // bilinear interpolation
    const double x2x = static_cast<double>(x2) - x;
    const double y2y = y2 - y;
    result = 1.0 / (x2x1 * y2y1) * (q11 * x2x * y2y + q21 * xx1 * y2y + q12 * x2x * yy1 + q22 * xx1 * yy1);
  }

  if (std::isnan(result))
  {
    result = 0.0;
  }
  return result;
}
}  // namespace util