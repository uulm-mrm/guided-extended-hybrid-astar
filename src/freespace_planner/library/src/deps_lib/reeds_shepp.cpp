/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Guan-Horng Liu.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author:  Guan-Horng Liu
 *********************************************************************/

#include "deps_lib/reeds_shepp.hpp"

namespace
{
// The comments, variable names, etc. use the nomenclature from the Reeds & Shepp paper.

const double pi = 3.14159265358979323846;
const double twopi = 2. * pi;
const double RS_EPS = 1e-6;
const double ZERO = 10 * std::numeric_limits<double>::epsilon();

inline double mod2pi(double x)
{
  double angle = fmod(x, twopi);
  if (angle < -pi)
  {
    angle += twopi;
  }
  else if (angle > pi)
  {
    angle -= twopi;
  }
  return angle;
}
inline void polar(double x, double y, double& r, double& theta)
{
  r = sqrt(x * x + y * y);
  theta = atan2(y, x);
}
inline void tauOmega(double u, double v, double xi, double eta, double phi, double& tau, double& omega)
{
  double delta = mod2pi(u - v);
  double A = sin(u) - sin(delta);
  double B = cos(u) - cos(delta) - 1.;
  double t1 = atan2(eta * A - xi * B, xi * A + eta * B);
  double t2 = 2. * (cos(delta) - cos(v) - cos(u)) + 3;
  tau = (t2 < 0) ? mod2pi(t1 + pi) : mod2pi(t1);
  omega = mod2pi(tau - u + v - phi);
}

// formula 8.1 in Reeds-Shepp paper
inline bool LpSpLp(double x, double y, double phi, double& t, double& u, double& v)
{
  polar(x - sin(phi), y - 1. + cos(phi), u, t);
  if (t >= -ZERO)
  {
    v = mod2pi(phi - t);
    if (v >= -ZERO)
    {
      assert(fabs(u * cos(t) + sin(phi) - x) < RS_EPS);
      assert(fabs(u * sin(t) - cos(phi) + 1 - y) < RS_EPS);
      assert(fabs(mod2pi(t + v - phi)) < RS_EPS);
      return true;
    }
  }
  return false;
}
// formula 8.2
inline bool LpSpRp(double x, double y, double phi, double& t, double& u, double& v)
{
  double t1;
  double u1;
  polar(x + sin(phi), y - 1. - cos(phi), u1, t1);
  u1 = u1 * u1;
  if (u1 >= 4.)
  {
    double theta;
    u = sqrt(u1 - 4.);
    theta = atan2(2., u);
    t = mod2pi(t1 + theta);
    v = mod2pi(t - phi);
    assert(fabs(2 * sin(t) + u * cos(t) - sin(phi) - x) < RS_EPS);
    assert(fabs(-2 * cos(t) + u * sin(t) + cos(phi) + 1 - y) < RS_EPS);
    assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
    return t >= -ZERO && v >= -ZERO;
  }
  return false;
}
void CSC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath& path)
{
  double t;
  double u;
  double v;
  double Lmin = path.length();
  double L;
  if (LpSpLp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[14], t, u, v);
    Lmin = L;
  }
  if (LpSpLp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[14], -t, -u, -v);
    Lmin = L;
  }
  if (LpSpLp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[15], t, u, v);
    Lmin = L;
  }
  if (LpSpLp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[15], -t, -u, -v);
    Lmin = L;
  }
  if (LpSpRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[12], t, u, v);
    Lmin = L;
  }
  if (LpSpRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[12], -t, -u, -v);
    Lmin = L;
  }
  if (LpSpRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[13], t, u, v);
    Lmin = L;
  }
  if (LpSpRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {  // timeflip + reflect
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[13], -t, -u, -v);
  }
}
// formula 8.3 / 8.4  *** TYPO IN PAPER ***
inline bool LpRmL(double x, double y, double phi, double& t, double& u, double& v)
{
  double xi = x - sin(phi);
  double eta = y - 1. + cos(phi);
  double u1;
  double theta;
  polar(xi, eta, u1, theta);
  if (u1 <= 4.)
  {
    u = -2. * asin(.25 * u1);
    t = mod2pi(theta + .5 * u + pi);
    v = mod2pi(phi - t + u);
    assert(fabs(2 * (sin(t) - sin(t - u)) + sin(phi) - x) < RS_EPS);
    assert(fabs(2 * (-cos(t) + cos(t - u)) - cos(phi) + 1 - y) < RS_EPS);
    assert(fabs(mod2pi(t - u + v - phi)) < RS_EPS);
    return t >= -ZERO && u <= ZERO;
  }
  return false;
}
void CCC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath& path)
{
  double t;
  double u;
  double v;
  double Lmin = path.length();
  double L;
  if (LpRmL(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[0], t, u, v);
    Lmin = L;
  }
  if (LpRmL(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[0], -t, -u, -v);
    Lmin = L;
  }
  if (LpRmL(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[1], t, u, v);
    Lmin = L;
  }
  if (LpRmL(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[1], -t, -u, -v);
    Lmin = L;
  }

  // backwards
  double xb = x * cos(phi) + y * sin(phi);
  double yb = x * sin(phi) - y * cos(phi);
  if (LpRmL(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[0], v, u, t);
    Lmin = L;
  }
  if (LpRmL(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[0], -v, -u, -t);
    Lmin = L;
  }
  if (LpRmL(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[1], v, u, t);
    Lmin = L;
  }
  if (LpRmL(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {  // timeflip + reflect
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[1], -v, -u, -t);
  }
}
// formula 8.7
inline bool LpRupLumRm(double x, double y, double phi, double& t, double& u, double& v)
{
  double xi = x + sin(phi);
  double eta = y - 1. - cos(phi);
  double rho = .25 * (2. + sqrt(xi * xi + eta * eta));
  if (rho <= 1.)
  {
    u = acos(rho);
    tauOmega(u, -u, xi, eta, phi, t, v);
    assert(fabs(2 * (sin(t) - sin(t - u) + sin(t - 2 * u)) - sin(phi) - x) < RS_EPS);
    assert(fabs(2 * (-cos(t) + cos(t - u) - cos(t - 2 * u)) + cos(phi) + 1 - y) < RS_EPS);
    assert(fabs(mod2pi(t - 2 * u - v - phi)) < RS_EPS);
    return t >= -ZERO && v <= ZERO;
  }
  return false;
}
// formula 8.8
inline bool LpRumLumRp(double x, double y, double phi, double& t, double& u, double& v)
{
  double xi = x + sin(phi);
  double eta = y - 1. - cos(phi);
  double rho = (20. - xi * xi - eta * eta) / 16.;
  if (rho >= 0 && rho <= 1)
  {
    u = -acos(rho);
    if (u >= -.5 * pi)
    {
      tauOmega(u, u, xi, eta, phi, t, v);
      assert(fabs(4 * sin(t) - 2 * sin(t - u) - sin(phi) - x) < RS_EPS);
      assert(fabs(-4 * cos(t) + 2 * cos(t - u) + cos(phi) + 1 - y) < RS_EPS);
      assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
      return t >= -ZERO && v >= -ZERO;
    }
  }
  return false;
}
void CCCC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath& path)
{
  double t;
  double u;
  double v;
  double Lmin = path.length();
  double L;
  if (LpRupLumRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[2], t, u, -u, v);
    Lmin = L;
  }
  if (LpRupLumRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[2], -t, -u, u, -v);
    Lmin = L;
  }
  if (LpRupLumRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[3], t, u, -u, v);
    Lmin = L;
  }
  if (LpRupLumRm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[3], -t, -u, u, -v);
    Lmin = L;
  }

  if (LpRumLumRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[2], t, u, u, v);
    Lmin = L;
  }
  if (LpRumLumRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[2], -t, -u, -u, -v);
    Lmin = L;
  }
  if (LpRumLumRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[3], t, u, u, v);
    Lmin = L;
  }
  if (LpRumLumRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
  {  // timeflip + reflect
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[3], -t, -u, -u, -v);
  }
}
// formula 8.9
inline bool LpRmSmLm(double x, double y, double phi, double& t, double& u, double& v)
{
  double xi = x - sin(phi);
  double eta = y - 1. + cos(phi);
  double rho;
  double theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.)
  {
    double r = sqrt(rho * rho - 4.);
    u = 2. - r;
    t = mod2pi(theta + atan2(r, -2.));
    v = mod2pi(phi - .5 * pi - t);
    assert(fabs(2 * (sin(t) - cos(t)) - u * sin(t) + sin(phi) - x) < RS_EPS);
    assert(fabs(-2 * (sin(t) + cos(t)) + u * cos(t) - cos(phi) + 1 - y) < RS_EPS);
    assert(fabs(mod2pi(t + pi / 2 + v - phi)) < RS_EPS);
    return t >= -ZERO && u <= ZERO && v <= ZERO;
  }
  return false;
}
// formula 8.10
inline bool LpRmSmRm(double x, double y, double phi, double& t, double& u, double& v)
{
  double xi = x + sin(phi);
  double eta = y - 1. - cos(phi);
  double rho;
  double theta;
  polar(-eta, xi, rho, theta);
  if (rho >= 2.)
  {
    t = theta;
    u = 2. - rho;
    v = mod2pi(t + .5 * pi - phi);
    assert(fabs(2 * sin(t) - cos(t - v) - u * sin(t) - x) < RS_EPS);
    assert(fabs(-2 * cos(t) - sin(t - v) + u * cos(t) + 1 - y) < RS_EPS);
    assert(fabs(mod2pi(t + pi / 2 - v - phi)) < RS_EPS);
    return t >= -ZERO && u <= ZERO && v <= ZERO;
  }
  return false;
}
void CCSC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath& path)
{
  double t;
  double u;
  double v;
  double Lmin = path.length() - .5 * pi;
  double L;
  if (LpRmSmLm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[4], t, -.5 * pi, u, v);
    Lmin = L;
  }
  if (LpRmSmLm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[4], -t, .5 * pi, -u, -v);
    Lmin = L;
  }
  if (LpRmSmLm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[5], t, -.5 * pi, u, v);
    Lmin = L;
  }
  if (LpRmSmLm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[5], -t, .5 * pi, -u, -v);
    Lmin = L;
  }

  if (LpRmSmRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[8], t, -.5 * pi, u, v);
    Lmin = L;
  }
  if (LpRmSmRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[8], -t, .5 * pi, -u, -v);
    Lmin = L;
  }
  if (LpRmSmRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[9], t, -.5 * pi, u, v);
    Lmin = L;
  }
  if (LpRmSmRm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[9], -t, .5 * pi, -u, -v);
    Lmin = L;
  }

  // backwards
  double xb = x * cos(phi) + y * sin(phi);
  double yb = x * sin(phi) - y * cos(phi);
  if (LpRmSmLm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[6], v, u, -.5 * pi, t);
    Lmin = L;
  }
  if (LpRmSmLm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[6], -v, -u, .5 * pi, -t);
    Lmin = L;
  }
  if (LpRmSmLm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[7], v, u, -.5 * pi, t);
    Lmin = L;
  }
  if (LpRmSmLm(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[7], -v, -u, .5 * pi, -t);
    Lmin = L;
  }

  if (LpRmSmRm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[10], v, u, -.5 * pi, t);
    Lmin = L;
  }
  if (LpRmSmRm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[10], -v, -u, .5 * pi, -t);
    Lmin = L;
  }
  if (LpRmSmRm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[11], v, u, -.5 * pi, t);
    Lmin = L;
  }
  if (LpRmSmRm(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {  // timeflip + reflect
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[11], -v, -u, .5 * pi, -t);
  }
}
// formula 8.11 *** TYPO IN PAPER ***
inline bool LpRmSLmRp(double x, double y, double phi, double& t, double& u, double& v)
{
  double xi = x + sin(phi);
  double eta = y - 1. - cos(phi);
  double rho;
  double theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.)
  {
    u = 4. - sqrt(rho * rho - 4.);
    if (u <= ZERO)
    {
      t = mod2pi(atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
      v = mod2pi(t - phi);
      assert(fabs(4 * sin(t) - 2 * cos(t) - u * sin(t) - sin(phi) - x) < RS_EPS);
      assert(fabs(-4 * cos(t) - 2 * sin(t) + u * cos(t) + cos(phi) + 1 - y) < RS_EPS);
      assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
      return t >= -ZERO && v >= -ZERO;
    }
  }
  return false;
}
void CCSCC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath& path)
{
  double t;
  double u;
  double v;
  double Lmin = path.length() - pi;
  double L;
  if (LpRmSLmRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path =
        ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[16], t, -.5 * pi, u, -.5 * pi, v);
    Lmin = L;
  }
  if (LpRmSLmRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
        ReedsSheppStateSpace::reedsSheppPathType[16], -t, .5 * pi, -u, .5 * pi, -v);
    Lmin = L;
  }
  if (LpRmSLmRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path =
        ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[17], t, -.5 * pi, u, -.5 * pi, v);
    Lmin = L;
  }
  if (LpRmSLmRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {  // timeflip + reflect
    path = ReedsSheppStateSpace::ReedsSheppPath(
        ReedsSheppStateSpace::reedsSheppPathType[17], -t, .5 * pi, -u, .5 * pi, -v);
  }
}

ReedsSheppStateSpace::ReedsSheppPath reedsShepp(double x, double y, double phi)
{
  ReedsSheppStateSpace::ReedsSheppPath path;
  CSC(x, y, phi, path);
  CCC(x, y, phi, path);
  CCCC(x, y, phi, path);
  CCSC(x, y, phi, path);
  CCSCC(x, y, phi, path);
  return path;
}
}  // namespace

const std::array<std::array<ReedsSheppStateSpace::ReedsSheppPathSegmentType, 5>, 18>
    ReedsSheppStateSpace::reedsSheppPathType = { {
        { RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP },         // 0
        { RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP },        // 1
        { RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP },       // 2
        { RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP },       // 3
        { RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP },    // 4
        { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP },   // 5
        { RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP },    // 6
        { RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP },   // 7
        { RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP },   // 8
        { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP },    // 9
        { RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP },   // 10
        { RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP },    // 11
        { RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP },     // 12
        { RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP },     // 13
        { RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP },      // 14
        { RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP },    // 15
        { RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT },  // 16
        { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT }   // 17
    } };

ReedsSheppStateSpace::ReedsSheppPath::ReedsSheppPath(std::array<ReedsSheppPathSegmentType, 5> type,
                                                     double t,
                                                     double u,
                                                     double v,
                                                     double w,
                                                     double x)
  : type_(type)
{
  length_[0] = t;
  length_[1] = u;
  length_[2] = v;
  length_[3] = w;
  length_[4] = x;
  totalLength_ = fabs(t) + fabs(u) + fabs(v) + fabs(w) + fabs(x);
}

double ReedsSheppStateSpace::distance(const Pose<double>& q_0, const Pose<double>& q_1) const
{
  return rho_ * reedsShepp(q_0, q_1).length();
}

ReedsSheppStateSpace::ReedsSheppPath ReedsSheppStateSpace::reedsShepp(const Pose<double>& q_0,
                                                                      const Pose<double>& q_1) const
{
  double dx = q_1.x - q_0.x;
  double dy = q_1.y - q_0.y;
  double dth = q_1.yaw - q_0.yaw;
  double c = cos(q_0.yaw);
  double s = sin(q_0.yaw);
  double x = c * dx + s * dy;
  double y = -s * dx + c * dy;
  return ::reedsShepp(x / rho_, y / rho_, dth);
}

void ReedsSheppStateSpace::type(const Pose<double>& q_0,
                                const Pose<double>& q_1,
                                ReedsSheppPathTypeCallback callback,
                                void* user_data) const
{
  ReedsSheppPath path = reedsShepp(q_0, q_1);
  for (int i = 0; i < 5; ++i)
  {
    callback(path.type_[i], user_data);
  }
}

ReedsSheppStateSpace::ReedsSheppPath ReedsSheppStateSpace::sample(const Pose<double>& q_0,
                                                                  const Pose<double>& q_1,
                                                                  double step_size) const
{
  ReedsSheppPath path = reedsShepp(q_0, q_1);
  const double dist = rho_ * path.length();
  path.radi = rho_;

  double delta = 0.0;

  // Store lengths and types for python compatibility
  int16_t idx = 0;
  for (auto l : path.length_)
  {
    switch (path.type_[idx])
    {
      case RS_LEFT:
        delta = std::atan(1 / path.radi * wb_);
        path.ctypes.push_back('L');
        break;
      case RS_RIGHT:
        delta = std::atan(-1 / path.radi * wb_);
        path.ctypes.push_back('R');
        break;
      case RS_STRAIGHT:
        delta = 0.0;
        path.ctypes.push_back('S');
        break;
      case RS_NOP:
        break;
    }
    path.lengths.push_back(l);
    idx++;
  }

  for (double seg = 0.0; seg <= dist; seg += step_size)
  {
    interpolate(q_0, path, seg / rho_);
  }

  // Add goal pose again
  path.x_list.push_back(q_1.x);
  path.y_list.push_back(q_1.y);
  path.yaw_list.push_back(q_1.yaw);
  path.delta_list.push_back(delta);
  path.directions.push_back(path.directions.back());
  return path;
}

void ReedsSheppStateSpace::interpolate(const Pose<double>& q_0, ReedsSheppPath& path, double seg) const
{
  if (seg < 0.0)
  {
    seg = 0.0;
  }
  if (seg > path.length())
  {
    seg = path.length();
  }

  double phi;
  double v;

  Pose<double> pose = { 0.0, 0.0, 0.0 };
  pose.x = pose.y = 0.0;
  pose.yaw = q_0.yaw;
  double delta = 0.0;

  int direction = 1;

  for (unsigned int i = 0; i < 5 && seg > 0; ++i)
  {
    if (path.length_[i] < 0)
    {
      v = std::max(-seg, path.length_[i]);
      seg += v;
      direction = -1;
    }
    else
    {
      v = std::min(seg, path.length_[i]);
      seg -= v;
      direction = 1;
    }
    phi = pose.yaw;
    switch (path.type_[i])
    {
      case RS_LEFT:
        delta = std::atan(1 / path.radi * wb_);
        pose.x += (sin(phi + v) - sin(phi));
        pose.y += (-cos(phi + v) + cos(phi));
        pose.yaw = phi + v;
        break;
      case RS_RIGHT:
        delta = std::atan(-1 / path.radi * wb_);
        pose.x += (-sin(phi - v) + sin(phi));
        pose.y += (cos(phi - v) - cos(phi));
        pose.yaw = phi - v;
        break;
      case RS_STRAIGHT:
        delta = 0;
        pose.x += (v * cos(phi));
        pose.y += (v * sin(phi));
        break;
      case RS_NOP:
        break;
    }
  }

  pose.x = pose.x * rho_ + q_0.x;
  pose.y = pose.y * rho_ + q_0.y;

  // Add values to path
  path.x_list.push_back(pose.x);
  path.y_list.push_back(pose.y);
  path.yaw_list.push_back(pose.yaw);
  path.delta_list.push_back(delta);
  path.directions.push_back(direction);
}