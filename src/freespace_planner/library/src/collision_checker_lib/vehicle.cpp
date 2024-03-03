//
// Created by schumann on 11.05.22.
//
#include "collision_checker_lib/vehicle.hpp"

void Vehicle::setPose(const Pose<double>& pose)
{
  pose_utm_ = pose;
};

void Vehicle::initialize(double max_steer, double w_b, double l_f, double l_b, double width, bool is_ushift)
{
  // Dimensions
  max_steer_ = max_steer;
  w_b_ = w_b;
  lf_ = l_f;
  lb_ = l_b;
  length_ = l_f + l_b;
  width_ = width;
  is_ushift_ = is_ushift;
  max_curvature_ = tan(max_steer_) / w_b_;

  // Calculate corners
  f_r_corner_ = Point<double>(lf_, -width_ / 2);
  f_l_corner_ = Point<double>(lf_, width_ / 2);
  r_r_corner_ = Point<double>(-lb_, -width_ / 2);
  r_l_corner_ = Point<double>(-lb_, width_ / 2);
  geo_center_ = (lb_ + lf_) / 2 - lb_;

  vehicle_vertices_ = { f_l_corner_, f_r_corner_, r_r_corner_, r_l_corner_, f_l_corner_ };
  vis_vehicle_vertices_ = { f_l_corner_, f_r_corner_, r_r_corner_, r_l_corner_, f_l_corner_ };
}

void Vehicle::move(Pose<double>& pose, double distance, double steer)
{
  /**
   * Integrate a certain distance and steering angle forward
   */
  pose.x += distance * cos(pose.yaw);
  pose.y += distance * sin(pose.yaw);
  pose.yaw += distance * tan(steer) / w_b_;
  pose.yaw = util::constrainAngleMinPIPlusPi(pose.yaw);
}

MotionPrimitive Vehicle::turn_on_rear_axis(const Pose<double>& pose, double delta_angle, double yaw_res_coll)
{
  // State size
  MotionPrimitive motion_primitive;

  // get start and goal
  double yaw = pose.yaw;
  double goal_yaw = util::constrainAngleMinPIPlusPi(yaw + delta_angle);
  double angle_margin;

  // rotate in steps of angle resolution
  yaw_res_coll = yaw_res_coll * util::TO_RAD;

  size_t est_nb_elements = ceil(std::abs(delta_angle) / yaw_res_coll);
  motion_primitive.yaw_list_.reserve(est_nb_elements);

  // rotate
  do
  {
    // Calculate and insert yaw
    yaw = util::constrainAngleMinPIPlusPi(yaw + util::sgn(delta_angle) * yaw_res_coll);
    motion_primitive.yaw_list_.push_back(yaw);

    // exit if angle margin is smaller than resolution
    angle_margin = std::abs(util::getAngleDiff(yaw, goal_yaw));

  } while (angle_margin >= yaw_res_coll);

  // Ensure that final yaw is correct
  if (motion_primitive.yaw_list_.back() != goal_yaw)
  {
    motion_primitive.yaw_list_.back() = goal_yaw;
  }

  // Calculate actual number of elements
  motion_primitive.nb_elements_ = motion_primitive.yaw_list_.size();

  // Add non changing coordinates
  motion_primitive.x_list_.resize(motion_primitive.nb_elements_, pose.x);
  motion_primitive.y_list_.resize(motion_primitive.nb_elements_, pose.y);
  motion_primitive.dir_list_.resize(motion_primitive.nb_elements_, 1);
  double delta;
  if (delta_angle > 0)
  {
    delta = util::PI / 2.0;
  }
  else
  {
    delta = -util::PI / 2.0;
  }
  motion_primitive.delta_list_.resize(motion_primitive.nb_elements_, delta);

  return motion_primitive;
}

MotionPrimitive
Vehicle::move_car_some_steps(const Pose<double>& pose, double arc_l, double motion_res, int direction, double steer)
{
  // Number of columns for pose vector
  const auto nb_elements = static_cast<size_t>(round(arc_l / motion_res));

  MotionPrimitive motion_primitive = MotionPrimitive(nb_elements);

  Pose<double> next_pose = pose;
  for (size_t i = 0; i < nb_elements; ++i)
  {
    // Move car and insert values
    move(next_pose, motion_res * direction, steer);

    motion_primitive.x_list_.push_back(next_pose.x);
    motion_primitive.y_list_.push_back(next_pose.y);
    motion_primitive.yaw_list_.push_back(next_pose.yaw);
    motion_primitive.dir_list_.push_back(direction);
    motion_primitive.delta_list_.push_back(steer);
  }

  return motion_primitive;
}
