//
// Created by schumann on 24.08.23.
// This cpp wrapper defines an c interface to the fortran routines for spline interpolation from scipy called fitpack.
// It was adapted from https://github.com/jbaayen/fitpackpp/blob/master/fitpackpp/BSplineCurve.cpp and ported to
// modern cpp.
//

#ifndef BSPLINE1D_H
#define BSPLINE1D_H

#include <vector>
#include <string>
#include <cassert>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace fitpack_wrapper
{
class BSpline1D
{
public:
  BSpline1D(std::vector<double>& x_vals, std::vector<double>& y_vals, int degree = 3, double smoothing = 0.0);

  double operator()(double x);

private:
  int k;                  // degree of spline
  int n;                  // number of knots
  int nc;                 // number of coefficients
  std::vector<double> t;  // knot coordinates
  std::vector<double> c;  // spline coefficients
};

}  // namespace fitpack_wrapper

#endif