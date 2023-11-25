//
// Created by Schumann on 12/9/22.
//

#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <iostream>
#include <vector>

#include "util_lib/data_structures2.hpp"
#include "util_lib/util2.hpp"
#include "collision_checker_lib/vehicle.hpp"
#include "a_star.hpp"

/**
 * Gradient Descent Smoother to smooth the path, who could have guessed it ;)
 */
class Smoother
{
private:
  inline static unsigned int max_iter_;
  inline static double wSmoothness_;
  inline static double wObstacle_;
  inline static double wCurvature_;
  inline static double alpha_;
  inline static double kappaMax_;
  inline static bool is_initialized_ = false;

public:
  static void init();

  static void smooth_path(Path& path);

  static void optimize_gd(Path& path, const std::vector<int>& collision_indices);

  static Point<double> smoothnessTerm(const Point<double>& xim2,
                                      const Point<double>& xim1,
                                      const Point<double>& xi0,
                                      const Point<double>& xip1,
                                      const Point<double>& xip2);

  static Point<double> curvatureTerm(const Point<double>& x_im2,
                                     const Point<double>& x_im1,
                                     const Point<double>& x_i,
                                     const Point<double>& x_ip1,
                                     const Point<double>& x_ip2);

  static Point<double> obsTerm(const Point<double>& xi0, const Point<double>& xip);
};

#endif  // SMOOTHER_H