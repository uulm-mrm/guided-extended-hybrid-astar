//
// Created by schumann on 3/14/23.
//

#include "gtest/gtest.h"
#include "util_lib/util1.hpp"
#include "util_lib/util2.hpp"
#include "util_lib/data_structures1.hpp"

TEST(UtilFunctions, constrainAngleZero2Pi)
{
  double angle_tol = 1e-8;
  double angle = 0;
  double offset = 0.0;
  double angle2check = angle + offset;
  double result = util::constrainAngleZero2Pi(angle2check);
  EXPECT_DOUBLE_EQ(result, angle2check);

  angle = util::PI;
  offset = 0.0;
  angle2check = angle + offset;
  result = util::constrainAngleZero2Pi(angle2check);
  EXPECT_DOUBLE_EQ(result, angle2check);

  angle = 2 * util::PI;
  offset = 0.0;
  angle2check = angle + offset;
  result = util::constrainAngleZero2Pi(angle2check);
  EXPECT_DOUBLE_EQ(result, 0);

  angle = 1.2;
  offset = 90 * util::PI;
  angle2check = angle + offset;
  result = util::constrainAngleZero2Pi(angle2check);
  EXPECT_NEAR(result, angle, angle_tol);

  angle = 1.2;
  offset = -90 * util::PI;
  angle2check = angle + offset;
  result = util::constrainAngleZero2Pi(angle2check);
  EXPECT_NEAR(result, angle, angle_tol);

  angle = 0;
  offset = -0.01;
  angle2check = angle + offset;
  result = util::constrainAngleZero2Pi(angle2check);
  EXPECT_GT(result, 0);
  EXPECT_LT(result, 2 * util::PI);

  angle = 2 * util::PI;
  offset = +0.01;
  angle2check = angle + offset;
  result = util::constrainAngleZero2Pi(angle2check);
  EXPECT_GT(result, 0);
  EXPECT_LT(result, 2 * util::PI);
}

TEST(UtilFunctions, constrainAngleMinPIPlusPi)
{
  double angle = -util::PI;
  double offset = -0.01;
  double angle2check = angle + offset;
  double result = util::constrainAngleMinPIPlusPi(angle2check);
  EXPECT_GT(result, -util::PI);
  EXPECT_LT(result, util::PI);

  angle = util::PI;
  offset = +0.01;
  angle2check = angle + offset;
  result = util::constrainAngleMinPIPlusPi(angle2check);
  EXPECT_GT(result, -util::PI);
  EXPECT_LT(result, util::PI);

  angle = -util::PI;
  offset = +0.01;
  angle2check = angle + offset;
  result = util::constrainAngleMinPIPlusPi(angle2check);
  EXPECT_DOUBLE_EQ(result, angle2check);

  angle = util::PI;
  offset = -0.01;
  angle2check = angle + offset;
  result = util::constrainAngleMinPIPlusPi(angle2check);
  EXPECT_DOUBLE_EQ(result, angle2check);

  double angle_eps = 1e-8;
  angle = -99 * util::PI;
  offset = +0.01;
  angle2check = angle + offset;
  result = util::constrainAngleMinPIPlusPi(angle2check);
  double expected_result = -util::PI + offset;
  EXPECT_NEAR(result, expected_result, angle_eps);

  angle = 99 * util::PI;
  offset = -0.01;
  angle2check = angle + offset;
  result = util::constrainAngleMinPIPlusPi(angle2check);
  expected_result = util::PI + offset;
  EXPECT_NEAR(result, expected_result, angle_eps);
}

TEST(UtilFunctions, getAngleDiff)
{
  double angle_tol = 1e-8;
  double angle1 = 0;
  double angle2 = util::PI;
  double result = util::getAngleDiff(angle1, angle2);
  double expected_result = -util::PI;
  EXPECT_NEAR(result, expected_result, angle_tol);

  angle1 = 2;
  angle2 = 0;
  result = util::getAngleDiff(angle1, angle2);
  expected_result = -2;
  EXPECT_NEAR(result, expected_result, angle_tol);

  angle1 = 0;
  angle2 = 2;
  result = util::getAngleDiff(angle1, angle2);
  expected_result = 2;
  EXPECT_NEAR(result, expected_result, angle_tol);

  angle1 = util::PI / 2;
  angle2 = -util::PI / 2;
  result = util::getAngleDiff(angle1, angle2);
  expected_result = -util::PI;
  EXPECT_NEAR(result, expected_result, angle_tol);

  angle1 = util::PI - 1;
  angle2 = -util::PI + 1;
  result = util::getAngleDiff(angle1, angle2);
  expected_result = 2;
  EXPECT_NEAR(result, expected_result, angle_tol);

  angle1 = -util::PI + 1;
  angle2 = util::PI - 1;
  result = util::getAngleDiff(angle1, angle2);
  expected_result = -2;
  EXPECT_NEAR(result, expected_result, angle_tol);
}

TEST(UtilFunctions, getDrivenAngleDiff)
{
  double angle_tol = 1e-8;
  double angle1 = 0;
  double angle2 = util::PI / 2;
  char direction = 'L';
  double result = util::getDrivenAngleDiff(angle1, angle2, direction);
  double expected_result = util::PI / 2;
  EXPECT_NEAR(result, expected_result, angle_tol);
  direction = 'R';
  result = util::getDrivenAngleDiff(angle1, angle2, direction);
  expected_result = -1.5 * util::PI;
  EXPECT_NEAR(result, expected_result, angle_tol);
}

TEST(UtilFunctions, angleLerp)
{
  double t;
  double yaw1;
  double yaw2;
  double result;
  double expected_result;

  // interpolation of zeros
  t = 0.0;
  yaw1 = 0.0;
  yaw2 = 0.0;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = 0.0;
  EXPECT_DOUBLE_EQ(result, expected_result);
  t = 0.5;
  yaw1 = 0.0;
  yaw2 = 0.0;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = 0.0;
  EXPECT_DOUBLE_EQ(result, expected_result);
  t = 1.0;
  yaw1 = 0.0;
  yaw2 = 0.0;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = 0.0;
  EXPECT_DOUBLE_EQ(result, expected_result);

  // interpolation not over switch
  t = 0.0;
  yaw1 = 0.0;
  yaw2 = util::PI / 2;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = 0.0;
  EXPECT_DOUBLE_EQ(result, expected_result);
  t = 0.5;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = util::PI / 4;
  EXPECT_DOUBLE_EQ(result, expected_result);
  t = 1.0;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = util::PI / 2;
  EXPECT_DOUBLE_EQ(result, expected_result);

  // interpolation not over switch minus
  t = 0.0;
  yaw1 = util::PI / 2;
  yaw2 = 0.0;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = util::PI / 2;
  EXPECT_DOUBLE_EQ(result, expected_result);
  t = 0.5;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = util::PI / 4;
  EXPECT_DOUBLE_EQ(result, expected_result);
  t = 1.0;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = 0;
  EXPECT_DOUBLE_EQ(result, expected_result);

  // interpolation over switch
  t = 0.0;
  yaw1 = 0.75 * util::PI;
  yaw2 = -0.75 * util::PI;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = yaw1;
  EXPECT_DOUBLE_EQ(result, expected_result);
  t = 0.25;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = util::PI * 7 / 8;
  EXPECT_DOUBLE_EQ(result, expected_result);
  t = 0.5;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = -util::PI;
  EXPECT_DOUBLE_EQ(result, expected_result);
  t = 0.75;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = -util::PI * 7 / 8;
  EXPECT_DOUBLE_EQ(result, expected_result);
  t = 1.0;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = yaw2;
  EXPECT_DOUBLE_EQ(result, expected_result);

  // interpolation over switch minus
  t = 0.0;
  yaw1 = -0.75 * util::PI;
  yaw2 = 0.75 * util::PI;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = yaw1;
  EXPECT_DOUBLE_EQ(result, expected_result);
  t = 0.25;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = -util::PI * 7 / 8;
  EXPECT_DOUBLE_EQ(result, expected_result);
  t = 0.5;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = -util::PI;
  EXPECT_DOUBLE_EQ(result, expected_result);
  t = 0.75;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = util::PI * 7 / 8;
  EXPECT_DOUBLE_EQ(result, expected_result);
  t = 1.0;
  result = util::angleLerp(t, yaw1, yaw2);
  expected_result = yaw2;
  EXPECT_DOUBLE_EQ(result, expected_result);
}

TEST(UtilFunctions, angleInterpolation)
{
  const double angle_tol = 1e-8;
  std::vector<double> distances = { 0, 1, 2, 3 };
  std::vector<double> angles = { util::PI / 2, util::PI * 3 / 4, -util::PI, -util::PI * 3 / 4 };
  double s;
  double result;
  double expected_result;

  s = -1;
  result = util::angleInterpolation(s, distances, angles);
  expected_result = angles[0];
  EXPECT_NEAR(result, expected_result, angle_tol);
  s = 0;
  result = util::angleInterpolation(s, distances, angles);
  expected_result = angles[0];
  EXPECT_NEAR(result, expected_result, angle_tol);
  s = 1;
  result = util::angleInterpolation(s, distances, angles);
  expected_result = angles[1];
  EXPECT_NEAR(result, expected_result, angle_tol);
  s = 2;
  result = util::angleInterpolation(s, distances, angles);
  expected_result = -util::PI;
  EXPECT_NEAR(result, expected_result, angle_tol);
  s = 2.5;
  result = util::angleInterpolation(s, distances, angles);
  expected_result = -util::PI * 7 / 8;
  EXPECT_NEAR(result, expected_result, angle_tol);
  s = 3;
  result = util::angleInterpolation(s, distances, angles);
  expected_result = angles[3];
  EXPECT_NEAR(result, expected_result, angle_tol);
  s = 4;
  result = util::angleInterpolation(s, distances, angles);
  expected_result = angles[3];
  EXPECT_NEAR(result, expected_result, angle_tol);
}

TEST(UtilFunctions, angleArange)
{
  const double angle_tol = 1e-8;
  double angle1;
  double angle2;
  char direction;
  double angle_res;
  std::vector<double> result;
  std::vector<double> expected_result;

  angle1 = 0.0;
  angle2 = -util::PI / 4;
  direction = 'L';
  angle_res = util::PI / 4;
  result = util::angleArange(angle1, angle2, direction, angle_res);
  expected_result = { util::PI / 4,      util::PI / 2,  util::PI * 3 / 4, -util::PI,
                      -util::PI * 3 / 4, -util::PI / 2, -util::PI / 4 };
  ASSERT_EQ(result.size(), expected_result.size()) << "Vectors x and y are of unequal length";
  for (int i = 0; i < result.size(); ++i)
  {
    EXPECT_NEAR(result[i], expected_result[i], angle_tol) << "Vectors x and y differ at index " << i;
  }

  angle1 = 0.0;
  angle2 = -util::PI / 4;
  direction = 'R';
  angle_res = util::PI / 8;
  result = util::angleArange(angle1, angle2, direction, angle_res);
  expected_result = { -util::PI / 8, -util::PI / 4 };

  ASSERT_EQ(result.size(), expected_result.size()) << "Vectors x and y are of unequal length";
  for (int i = 0; i < result.size(); ++i)
  {
    EXPECT_NEAR(result[i], expected_result[i], angle_tol) << "Vectors x and y differ at index " << i;
  }
}

TEST(UtilFunctions, bilinearInterpolation)
{
  int x1 = 0;
  int y1 = 0;
  int x2 = 1;
  int y2 = 1;

  double q11 = 0;
  double q12 = 0;
  double q21 = 0;
  double q22 = 0;
  double x = 0;
  double y = 0;
  double val = util::bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
  EXPECT_DOUBLE_EQ(val, 0.0);

  // depends only on x
  q11 = 0;
  q12 = 0;
  q21 = 2;
  q22 = 2;
  x = 0.0;
  y = 0.0;
  val = util::bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
  EXPECT_DOUBLE_EQ(val, 0.0);
  x = 0.0;
  y = 1.0;
  val = util::bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
  EXPECT_DOUBLE_EQ(val, 0.0);
  x = 1.0;
  y = 0.0;
  val = util::bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
  EXPECT_DOUBLE_EQ(val, 2.0);
  x = 1.0;
  y = 1.0;
  val = util::bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
  EXPECT_DOUBLE_EQ(val, 2.0);
  x = 0.5;
  y = 0.5;
  val = util::bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
  EXPECT_DOUBLE_EQ(val, 1.0);

  // depends only on y
  q11 = 0;
  q12 = 2;
  q21 = 0;
  q22 = 2;
  x = 0.0;
  y = 0.0;
  val = util::bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
  EXPECT_DOUBLE_EQ(val, 0.0);
  x = 0.0;
  y = 1.0;
  val = util::bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
  EXPECT_DOUBLE_EQ(val, 2.0);
  x = 1.0;
  y = 0.0;
  val = util::bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
  EXPECT_DOUBLE_EQ(val, 0.0);
  x = 1.0;
  y = 1.0;
  val = util::bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
  EXPECT_DOUBLE_EQ(val, 2.0);
  x = 0.5;
  y = 0.5;
  val = util::bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
  EXPECT_DOUBLE_EQ(val, 1.0);

  // depends on x and y
  q11 = 0;
  q12 = 1;
  q21 = 1;
  q22 = 2;
  x = 0.0;
  y = 0.0;
  val = util::bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
  EXPECT_DOUBLE_EQ(val, 0.0);
  x = 0.0;
  y = 1.0;
  val = util::bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
  EXPECT_DOUBLE_EQ(val, 1.0);
  x = 1.0;
  y = 0.0;
  val = util::bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
  EXPECT_DOUBLE_EQ(val, 1.0);
  x = 1.0;
  y = 1.0;
  val = util::bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
  EXPECT_DOUBLE_EQ(val, 2.0);
  x = 0.5;
  y = 0.5;
  val = util::bilinearInterpolation(q11, q12, q21, q22, x1, x2, y1, y2, x, y);
  EXPECT_DOUBLE_EQ(val, 1.0);
}

TEST(UtilFunctions, getBilinInterp)
{
  Vec2DFlat<double> test_grid;
  test_grid.resize(2, 2);
  test_grid(0, 0) = 0;
  test_grid(0, 1) = 1;
  test_grid(1, 0) = 1;
  test_grid(1, 1) = 2;

  double x = 0;
  double y = 0;
  double val = util::getBilinInterp(x, y, test_grid);
  EXPECT_DOUBLE_EQ(val, 0.0);

  x = 1.0;
  y = 0.0;
  val = util::getBilinInterp(x, y, test_grid);
  EXPECT_DOUBLE_EQ(val, 1.0);

  x = 0.0;
  y = 1.0;
  val = util::getBilinInterp(x, y, test_grid);
  EXPECT_DOUBLE_EQ(val, 1.0);

  x = 1.0;
  y = 1.0;
  val = util::getBilinInterp(x, y, test_grid);
  EXPECT_DOUBLE_EQ(val, 2.0);

  x = 0.5;
  y = 0.5;
  val = util::getBilinInterp(x, y, test_grid);
  EXPECT_DOUBLE_EQ(val, 1.0);
}
