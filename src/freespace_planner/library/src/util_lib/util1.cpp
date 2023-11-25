//
// Created by schumann on 2/8/23.
//
#include "util_lib/util1.hpp"

namespace util
{
double constrainAngleZero2Pi(double angle)
{
  angle = fmod(angle, 2 * PI);
  if (angle < 0)
  {
    angle += 2 * PI;
  }
  return angle;
}

double constrainAngleMinPIPlusPi(double angle)
{
  angle = fmod(angle + PI, 2 * PI);
  if (angle < 0)
  {
    angle += 2 * PI;
  }
  return angle - PI;
}

double getSignedAngleDiff(double angle1, double angle2)
{
  double diff = angle1 - angle2;
  if (diff > PI)
  {
    diff -= 2 * PI;
  }
  else if (diff < -PI)
  {
    diff += 2 * PI;
  }
  else
  {
    // keep a
  }
  return diff;
}

double getAngleDiff(double angle1, double angle2)
{
  return std::abs(getSignedAngleDiff(angle1, angle2));
}

double getAngleDiffInDir(double goal_angle, double start_angle, int dir)
{
  auto angle_diff = getSignedAngleDiff(goal_angle, start_angle);
  if (dir == 1 && angle_diff < 0)
  {
    angle_diff += 2 * PI;
  }
  if (dir == -1 && angle_diff >= 0)
  {
    angle_diff = std::abs(angle_diff);
  }
  return angle_diff;
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
  int x2x1 = x2 - x1;
  int y2y1 = y2 - y1;
  double yy1 = y - y1;
  double xx1 = x - x1;
  double result;
  double eps = 0.0001;

  // dont interpolate
  if (abs(x2x1) < eps and abs(y2y1) < eps)
  {
    result = q11;
  }
  else if (abs(x2x1) < eps)
  {
    // interpolate only in y direction
    double t = yy1 / y2y1;
    result = std::lerp(q11, q12, t);
  }
  else if (abs(y2y1) < eps)
  {
    // interpolate only in x direction
    double t = xx1 / x2x1;
    result = std::lerp(q11, q21, t);
  }
  else
  {
    // bilinear interpolation
    double x2x = x2 - x;
    double y2y = y2 - y;
    result = 1.0 / (x2x1 * y2y1) * (q11 * x2x * y2y + q21 * xx1 * y2y + q12 * x2x * yy1 + q22 * xx1 * yy1);
  }

  if (std::isnan(result))
  {
    result = 0.0;
  }
  return result;
}
}  // namespace util