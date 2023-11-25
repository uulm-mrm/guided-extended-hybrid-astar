//
// Created by schumann on 24.08.23.
//

#include "deps_lib/BSpline1D.hpp"

#include "FCMangle.h"

extern "C" {
void curfit(int* iopt,
            int* m,
            double* x,
            double* y,
            double* w,
            double* xb,
            double* xe,
            int* k,
            double* s,
            int* nest,
            int* n,
            double* t,
            double* c,
            double* fp,
            double* wrk,
            int* lwrk,
            int* iwrk,
            int* ier);

void splev(double* t, int* n, double* c, int* nc, int* k, double* x, double* y, int* m, int* e, int* ier);
}

namespace fitpack_wrapper
{
/**
 * @brief Constructor of the B-Spline curve
 * @details Construct a B-Spline curve interpolation for the points specified by the list of abscissae x and ordinates
 * y.
 *
 * @param x value over which spline is parametrised
 * @param y coordinate value
 * @param degree degree of the interpolating spline.
 * The actual degree is chosen such as to be one less than the number of data points, but no higher than
 * preferredDegree.
 * @param smoothing Smoothing factor.  Must be non-negative. Set to 0.0, i.e., no smoothing, by default.
 */
BSpline1D::BSpline1D(std::vector<double>& x_vals, std::vector<double>& y_vals, int degree, double smoothing)
{
  // Number of data points
  int m = static_cast<int>(x_vals.size());

  // The actual degree of the spline must be less than m
  k = degree;
  if (k >= m)
  {
    k = m - 1;

    //    std::cerr << "WARNING:  Too few data points (" << m << ") to create B-Spline curve of order " << degree
    //              << ". Reducing order to " << k << "." << std::endl;
  }

  // Configure curfit() parameters
  int iopt = 0;          // Compute a smoothing spline
  int nest = m + k + 1;  // Over-estimate the number of knots

  // weighting vector
  std::vector<double> weights(m, 1.0);

  // knots and coefficients
  t.assign(nest, 0.0);
  c.assign(nest, 0.0);
  nc = static_cast<int>(c.size());

  // Weighted sum of squared residuals
  double fp = 0.0;

  // working memory required by curfit
  int lwrk = (m * (k + 1) + nest * (7 + 3 * k));
  std::vector<double> wrk(lwrk, 0.0);
  std::vector<int> iwrk(nest, 0);

  int ier = 0;
  curfit(&iopt,
         &m,
         x_vals.data(),
         y_vals.data(),
         weights.data(),
         &x_vals.front(),
         &x_vals.back(),
         &k,
         &smoothing,
         &nest,
         &n,
         t.data(),
         c.data(),
         &fp,
         wrk.data(),
         &lwrk,
         iwrk.data(),
         &ier);

  if (ier > 0)
  {
    if (ier >= 10)
    {
      std::stringstream string_out;
      string_out << "Error fitting B-Spline curve using curfit(): " << ier;
      throw std::runtime_error(string_out.str());
    }
    std::cerr << "WARNING:  Non-fatal error while fitting B-Spline curve using curfit(): " << ier << std::endl;
  }
}

/**
 * @brief Evaluate the spline at point x
 *
 * @param x Evaluation point
 * @return Curve ordinate at point x
 */
double BSpline1D::operator()(double x)
{
  double y = 0.0;
  int m = 1;  // Evaluate a single point
  int e = 0;  // Don't clip argument to range
  int ier = 0;

  // splev also evaluates points in the exterior
  splev(t.data(), &n, c.data(), &nc, &k, &x, &y, &m, &e, &ier);
  if (ier > 0)
  {
    std::stringstream string_out;
    string_out << "Error evaluating B-Spline curve using splev() at point " << x << ": " << ier;
    throw std::runtime_error(string_out.str());
  }

  return y;
}

}  // namespace fitpack_wrapper