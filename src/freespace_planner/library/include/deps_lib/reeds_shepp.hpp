/*********************************************************************
 * Adapted from The following file. Thanks a lot!
 *
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

#ifndef SPACES_REEDS_SHEPP_STATE_SPACE_
#define SPACES_REEDS_SHEPP_STATE_SPACE_

#include <cmath>
#include <array>
#include <vector>
#include <cassert>

#include <util_lib/data_structures1.hpp>

using ReedsSheppPathTypeCallback = int (*)(int, void*);

class ReedsSheppStateSpace
{
public:
  /** \brief The Reeds-Shepp path segment types */
  enum ReedsSheppPathSegmentType
  {
    RS_NOP = 0,
    RS_LEFT = 1,
    RS_STRAIGHT = 2,
    RS_RIGHT = 3
  };

  /** \brief Reeds-Shepp path types */
  //  static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];
  static const std::array<std::array<ReedsSheppStateSpace::ReedsSheppPathSegmentType, 5>, 18> reedsSheppPathType;

  /** \brief Complete description of a ReedsShepp path */
  class ReedsSheppPath
  {
  public:
    explicit ReedsSheppPath(std::array<ReedsSheppPathSegmentType, 5> type = reedsSheppPathType[0],
                            double t = std::numeric_limits<double>::max(),
                            double u = 0.,
                            double v = 0.,
                            double w = 0.,
                            double x = 0.);

    [[nodiscard]] double length() const
    {
      return totalLength_;
    }

    // operator to allow sorting by length for non-holonomic no obs heuristics
    bool operator<(const ReedsSheppPath& other) const
    {
      return totalLength_ < other.totalLength_;
    }

    /** Path segment types */
    std::array<ReedsSheppPathSegmentType, 5> type_;
    /** Path segment lengths */
    std::array<double, 5> length_;  // TODO (Schumann) Why on earth is this false. The value is corrected later

    /** Total length */
    double totalLength_;
    double radi;
    std::vector<char> ctypes;
    std::vector<int> directions;
    std::vector<double> lengths;
    std::vector<double> x_list;
    std::vector<double> y_list;
    std::vector<double> yaw_list;
    double cost = -1;
  };

  explicit ReedsSheppStateSpace(double turningRadius) : rho_(turningRadius)
  {
  }

  [[nodiscard]] double distance(const Pose<double>& q_0, const Pose<double>& q_1) const;

  void
  type(const Pose<double>& q_0, const Pose<double>& q_1, ReedsSheppPathTypeCallback callback, void* user_data) const;

  ReedsSheppPath sample(const Pose<double>& q_0, const Pose<double>& q_1, double step_size);

  /** \brief Return the shortest Reeds-Shepp path from SE(2) state state1 to SE(2) state state2 */
  [[nodiscard]] ReedsSheppPath reedsShepp(const Pose<double>& q_0, const Pose<double>& q_1) const;

protected:
  void interpolate(const Pose<double>& q_0, ReedsSheppPath& path, double seg) const;

  /** \brief Turning radius */
  double rho_;
};

#endif