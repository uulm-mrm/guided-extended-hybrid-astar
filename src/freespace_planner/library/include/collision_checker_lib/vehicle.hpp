//
// Created by schumann on 11.05.22.
//
#ifndef PLANNING_BINDINGS_VEHICLE_HPP
#define PLANNING_BINDINGS_VEHICLE_HPP

#include "util_lib/util2.hpp"
#include "util_lib/data_structures2.hpp"

#include "yaml-cpp/yaml.h"

#include <cmath>
#include <utility>
#include <vector>
#include <array>
#include <iostream>

class Vehicle
{
public:
  inline static Pose<double> pose_utm_;
  inline static bool is_ushift_;
  inline static double max_steer_;
  inline static double max_curvature_;
  inline static double w_b_;
  inline static double lf_;
  inline static double lb_;
  inline static double length_;
  inline static double width_;
  inline static double geo_center_;
  inline static Point<double> f_r_corner_;
  inline static Point<double> f_l_corner_;
  inline static Point<double> r_r_corner_;
  inline static Point<double> r_l_corner_;
  inline static std::vector<Point<double>> vehicle_vertices_;

  static void setPose(const Pose<double>& pose);

  static void initialize(double max_steer, double w_b, double l_f, double l_b, double width, bool is_ushift);

  static std::vector<Point<double>> getVehicleVertices();

  static void move(Pose<double>& pose, double distance, double steer);

  static MotionPrimitive
  move_car_some_steps(const Pose<double>& pose, double arc_l, double motion_res, int direction, double steer);

  static MotionPrimitive turn_on_rear_axis(const Pose<double>& pose, double delta_angle, double yaw_res_coll);
};

#endif  // PLANNING_BINDINGS_VEHICLE_HPP
