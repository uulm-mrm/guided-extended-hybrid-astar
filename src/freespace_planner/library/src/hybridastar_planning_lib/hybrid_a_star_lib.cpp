//
// Created by schumann on 11.05.22.
//
#include "hybridastar_planning_lib/hybrid_a_star_lib.hpp"

/**
 * Must be initialized whenever the patch_info size changes
 * @param patch_dim
 * @param path2config
 * @param path2data
 */
void HybridAStar::initialize(int patch_dim, const Point<double>& patch_origin_utm, const std::string& lib_share_dir)
{
  // Load config
  const std::string path2config = lib_share_dir + "/config/config.yml";
  YAML::Node config = YAML::LoadFile(path2config);

  //  auto log_level = config["LOG_LEVEL_HASTAR"].as<std::string>();

  //  std::string stream_name = "freespace_planner->HybridAStar";
  //  hybrid_astar::_initLogger();
  //  hybrid_astar::_setStreamName(stream_name);
  //  hybrid_astar::_setLogLevel(log_level);
  //  hybrid_astar::_setShowOrigin(true);

  // Seed for random positions of RS extension
  int seed = 42;
  srand(seed);

  patch_origin_utm_ = patch_origin_utm;

  path2data_ = lib_share_dir + "/data";

  gm_res_ = GM_RES;
  astar_res_ = ASTAR_RES;
  arc_l_ = astar_res_ * 1.5;  // arc length must be longer than the diagonal distance of a cell

  astar_yaw_res_deg_ = config["YAW_RES"].as<int>();
  astar_yaw_res_ = astar_yaw_res_deg_ * util::TO_RAD;
  astar_yaw_dim_ = 360 / astar_yaw_res_deg_;
  min_yaw_idx_ = static_cast<int>(round(-util::PI / astar_yaw_res_) - 1);
  a_star_yaw_res_inv_ = 1 / astar_yaw_res_;
  steering_inputs_ = calcSteeringInputs();
  if (config["ONLY_FORWARD"].as<bool>())
  {
    direction_inputs_ = { 1, 1 };  // inefficient but is a quickhack for testing anyways
  }
  else
  {
    direction_inputs_ = { 1, -1 };
  }
  comfort_brake_acc_ = config["COMFORT_BRAKE_ACC"].as<double>();
  approx_goal_dist2_ = pow(config["APPROX_GOAL_DIST"].as<double>(), 2);
  approx_goal_angle_rad_ = config["APPROX_GOAL_ANGLE"].as<double>() * util::TO_RAD;
  waypoint_dist_ = config["WAYPOINT_DIST"].as<int>();
  max_dist4waypoints_ = config["MAX_DIST4WAYPOINTS"].as<int>();
  waypoint_type_ = static_cast<WaypointType>(config["WAYPOINT_TYPE"].as<int>());
  dist_thresh_analytic_ = config["DIST_THRESH_ANALYTIC_M"].as<double>();
  second_rs_steer_factor_ = config["RS_2ND_STEER"].as<double>();
  extra_steer_cost_analytic_ = config["EXTRA_STEER_COST_ANALYTIC"].as<double>();
  max_extra_nodes_ = config["MAX_EXTRA_NODES_HASTAR"].as<size_t>();
  can_turn_on_point_ = Vehicle::is_ushift_;
  turn_on_point_angle_ = config["TURN_ON_POINT_ANGLE"].as<double>();
  turn_on_point_horizon_ = config["TURN_ON_POINT_HORIZON"].as<double>();
  yaw_res_coll_ = config["YAW_RES_COLL"].as<double>();
  rear_axis_cost_ = config["REAR_AXIS_COST"].as<double>();
  timeout_ms_ = config["TIMEOUT"].as<int>();
  motion_res_min_ = config["MOTION_RES_MIN"].as<double>();
  motion_res_max_ = config["MOTION_RES_MAX"].as<double>();
  interp_res_ = config["INTERP_RES"].as<double>();
  rear_axis_freq_ = config["RA_FREQ"].as<int>();
  non_h_no_obs_patch_dim_ = config["NON_H_NO_OBS_PATCH_DIM"].as<int>();
  if (!non_h_no_obs_calculated_)
  {
    calculateNonhnoobs();
  }

  grid_tf::updateTransforms(patch_origin_utm_);

  AStar::initialize(patch_dim, patch_origin_utm, path2config);
  CollisionChecker::initialize(patch_dim, path2config);
  updateLaneGraph(patch_origin_utm, patch_dim);

  if (is_sim_)
  {
    Cartographing::resetPatch(patch_dim);
  }
}

/**
 * reinits astar and collision checking lib on patch_info size change
 * @param patch_dim
 */
void HybridAStar::reinit(const Point<double>& patch_origin_utm, int patch_dim)
{
  patch_origin_utm_ = patch_origin_utm;

  grid_tf::updateTransforms(patch_origin_utm_);

  AStar::reinit(patch_origin_utm, patch_dim);
  CollisionChecker::resetPatch(patch_dim);

  updateLaneGraph(patch_origin_utm, patch_dim);

  if (is_sim_)
  {
    Cartographing::resetPatch(patch_dim);
  }
}

/**
 * Calculates the steering inputs depending on the maximum steering angle and the number of steers
 * @return
 */
std::array<double, HybridAStar::NB_STEER> HybridAStar::calcSteeringInputs()
{
  const double delta_steer = (2 * Vehicle::max_steer_) / static_cast<double>(NB_STEER - 1);
  std::array<double, HybridAStar::NB_STEER> steering_inputs;

  const int n_steer_side = static_cast<int>(floor(NB_STEER / 2));
  double steer;
  // Add steering to the left
  for (int j = 0, i = -n_steer_side; i <= n_steer_side; ++i, ++j)
  {
    steer = i * delta_steer;
    steering_inputs[j] = steer;
  }
  return steering_inputs;
}

/**
 * Calculates flattened index of 3D index
 * @param x_index
 * @param y_index
 * @param yaw_index
 * @return
 */
size_t HybridAStar::calculateIndex(size_t x_index, size_t y_index, int yaw_index)
{
  return static_cast<size_t>(yaw_index - min_yaw_idx_) * (AStar::astar_dim_) * (AStar::astar_dim_) +
         y_index * (AStar::astar_dim_) + x_index;
}

/**
 * verify if index is on astar map
 * @param x_index
 * @param y_index
 * @return
 */
bool HybridAStar::verifyIndex(int x_index, int y_index)
{
  return (-1 < x_index && x_index < static_cast<int>(AStar::astar_dim_)) and
         (-1 < y_index && y_index < static_cast<int>(AStar::astar_dim_));
}

/**
 * Get non holonomic and no obstacles heuristic from previously created patches
 * @param x_idx_s
 * @param y_idx_s
 * @param yaw_idx_s
 * @param x_idx_g
 * @param y_idx_g
 * @param yaw_idx_g
 * @return
 */
double HybridAStar::getNonhnoobsVal(const NodeHybrid& start_node, const NodeHybrid& goal_node)
{
  // check if vehicle is inside the patch_info! Verify only the smallest euclidian distance (width/2) == circular around
  const int x_diff = goal_node.x_index - start_node.x_index;
  const int y_diff = goal_node.y_index - start_node.y_index;
  const double max_dist = pow(non_h_no_obs_patch_dim_ / 2, 2);
  const double dist = pow(x_diff, 2) + pow(y_diff, 2);

  if (dist > max_dist)
  {
    return 0;
  }

  // Calculate indices on patch_info
  const int middle_index = static_cast<int>((non_h_no_obs_patch_dim_ - 1)) / 2;
  int x_idx_s_patch = middle_index - x_diff;
  int y_idx_s_patch = middle_index - y_diff;
  const int x_idx_g_patch = middle_index;
  const int y_idx_g_patch = middle_index;

  //  LOG_INF("Node is inside patch_info");

  // get angle diff in indices
  int yaw_idx_diff = start_node.yaw_index - goal_node.yaw_index;
  if (yaw_idx_diff < min_yaw_idx_)
  {
    yaw_idx_diff += static_cast<int>(astar_yaw_dim_);
  }
  yaw_idx_diff = static_cast<int>(yaw_idx_diff - min_yaw_idx_) % astar_yaw_dim_ + min_yaw_idx_;

  //  double angle_diff = yaw_idx_diff * astar_yaw_res_;
  const double goal_angle = goal_node.yaw_index * astar_yaw_res_;
  //  LOG_INF("Angle diff in indices is " << yaw_idx_diff);
  //  LOG_INF("This is an angle of ~= " << angle_diff * util::TO_DEGREES);

  //  LOG_INF("X Start was " << x_idx_s_patch);
  //  LOG_INF("Y Start was " << y_idx_s_patch);
  //  LOG_INF("X Goal was " << x_idx_g_patch);
  //  LOG_INF("Y Goal was " << y_idx_g_patch);

  // rotate start coordinates around goal coordinates by rotation of goal
  const double sin_angle = sin(-goal_angle);
  const double cos_angle = cos(-goal_angle);
  // translate point back to origin:
  x_idx_s_patch -= x_idx_g_patch;
  y_idx_s_patch -= y_idx_g_patch;
  // rotate point
  const double xnew = x_idx_s_patch * cos_angle - y_idx_s_patch * sin_angle;
  const double ynew = x_idx_s_patch * sin_angle + y_idx_s_patch * cos_angle;
  // translate point back:
  x_idx_s_patch = static_cast<int>(std::round(xnew + x_idx_g_patch));
  y_idx_s_patch = static_cast<int>(std::round(ynew + y_idx_g_patch));

  //  LOG_INF("X is " << x_idx_s_patch);
  //  LOG_INF("Y is " << y_idx_s_patch);

  const int yaw_idx = yaw_idx_diff - min_yaw_idx_;
  //  LOG_INF("Yaw idx is " << yaw_idx);

  //  DebugHelper::show_3d_vec_slice("Current patch_info", non_h_no_obs_, yaw_idx);
  return non_h_no_obs_(yaw_idx, y_idx_s_patch, x_idx_s_patch);
}

/**
 * Checks if node is on the heuristic, if yes it
 * Calculates the cost out of the distance heuristic cost and non-holonomic no obstacle cost and weights it with
 * the movement cost up to here
 * @param node
 * @param goal_node
 * @param h_dp
 * @return
 */
double HybridAStar::calcCost(const NodeHybrid& node,
                             const NodeHybrid& goal_node,
                             const std::unordered_map<size_t, NodeDisc>& h_dp)
{  // Calculate index of 2D node and check if it is within the calculated heuristic
  const size_t ind = AStar::calcIndex(node.x_index, node.y_index);
  auto search_result = h_dp.find(ind);
  if (search_result == h_dp.end())
  {
    return OUT_OF_HEURISTIC;
  }
  // estimated cost from current node to goal from distance heuristic and non-holonomic no obstacle heuristic
  //  const double h_dist = search_result->second.cost_dist_;
  const double h_dist = search_result->second.cost_;

  // get nonh no obs cost
  const double h_non_h_no_obs = getNonhnoobsVal(node, goal_node);

  const double heuristic_cost = std::max(h_dist, h_non_h_no_obs) * h_dist_cost_;

  return heuristic_cost + node.cost;
}

/**
 *
 * @param yaw
 * @return
 */
double HybridAStar::getProxOfCorners(const Pose<double>& point)
{
  const double cos_yaw = cos(point.yaw);
  const double sin_yaw = sin(point.yaw);

  const Point<double> offset(point.x, point.y);
  const Point<double> f_r = Vehicle::f_r_corner_.rotatePreCalc(cos_yaw, sin_yaw) + offset;
  const Point<double> f_l = Vehicle::f_l_corner_.rotatePreCalc(cos_yaw, sin_yaw) + offset;
  const Point<double> r_r = Vehicle::r_r_corner_.rotatePreCalc(cos_yaw, sin_yaw) + offset;
  const Point<double> r_l = Vehicle::r_l_corner_.rotatePreCalc(cos_yaw, sin_yaw) + offset;

  const double v_1 = util::getBilinInterp(f_r.x, f_r.y, AStar::h_prox_arr_);
  const double v_2 = util::getBilinInterp(f_l.x, f_l.y, AStar::h_prox_arr_);
  const double v_3 = util::getBilinInterp(r_r.x, r_r.y, AStar::h_prox_arr_);
  const double v_4 = util::getBilinInterp(r_l.x, r_l.y, AStar::h_prox_arr_);

  const double max_value = std::max({ v_1, v_2, v_3, v_4 });

  if (std::isnan(max_value))
  {
    return 0.0;
  }

  return max_value;
}

/**
 * Calculates the costs of a path caused by its movement and proximity to objects
 * @param x_ind
 * @param y_ind
 * @param yaw
 * @param node
 * @param steer
 * @param direction
 * @param arc_l
 * @return
 */
double HybridAStar::getPathCosts(int x_ind,
                                 int y_ind,
                                 double yaw,
                                 const NodeHybrid& node,
                                 double steer,
                                 int direction,
                                 double arc_l)
{
  // Get movement Penalties
  double control_cost = 0;
  // changes in direction are penalized
  if (direction != node.discrete_direction)
  {
    control_cost += switch_cost_;
  }
  // steer penalty
  control_cost += steer_cost_ * std::abs(steer);
  // steer change penalty
  control_cost += steer_change_cost_ * std::abs(node.delta_list[0] - steer);

  // Update costs to reach the new node. Consists of costs of previous costs, including additional costs caused by
  // steering and the travelled distance, denoted by the arc length
  const double movement_weigth = AStar::movement_cost_map_(y_ind, x_ind);

  double distance_cost;
  if (direction == -1)
  {
    distance_cost = arc_l * movement_weigth * back_cost_;
  }
  else
  {
    distance_cost = arc_l * movement_weigth;
  }

  const Pose<double> pose = { node.x_list.back(), node.y_list.back(), yaw };
  const double prox_cost = getProxOfCorners(pose * grid_tf::con2star_) * h_prox_cost_ * arc_l;

  const double movement_cost = control_cost + distance_cost + prox_cost;

  return movement_cost;
}

/**
 * Calculates the next node by trying to turn on the rear axis
 * @param node
 * @param delta_angle
 * @return
 */
std::optional<NodeHybrid> HybridAStar::calcRearAxisNode(const NodeHybrid& node, double delta_angle)
{
  // Move the car in continuous coordinates for a specific arc length
  const double arc_l = 1.0;  // artificial length to scale costs

  // Move vehicle on motion primitive
  const Pose<double> state{ node.x_list.back(), node.y_list.back(), node.yaw_list.back() };
  const MotionPrimitive motion_primitive = Vehicle::turn_on_rear_axis(state, delta_angle, yaw_res_coll_);

  // Check if car collided
  if (!CollisionChecker::checkPathCollision(
          motion_primitive.x_list_, motion_primitive.y_list_, motion_primitive.yaw_list_))
  {
    return {};
  }

  // remove element from collision_indices
  //  reachable_indices_.insert(AStar::calcIndex(node.x_index, node.y_index));

  // Calculate discrete coordinates of reached position to set the reached node
  const int direction = motion_primitive.dir_list_[0];
  const Pose<int> disc_pose = mapCont2Disc(motion_primitive);

  // New costs caused by this node
  // penalize turning on rear axis
  const double rear_axis_cost = getTurnCost(delta_angle);
  const double movement_cost =
      rear_axis_cost + getPathCosts(disc_pose.x, disc_pose.y, state.yaw, node, 0.0, direction, arc_l);

  // Node costs until here
  const double cost = node.cost + movement_cost;

  // distribute costs over number of steps with given motion inputs to truncate costs when path is driven along later
  const int nb_elements = static_cast<int>(motion_primitive.nb_elements_);
  //  std::vector<double> cont_cost_list(nb_elements, movement_cost / static_cast<double>(nb_elements));

  // Set type of path segment
  const std::vector<PATH_TYPE> types(nb_elements, PATH_TYPE::REAR_AXIS);

  const auto parent_idx = static_cast<int64_t>(calculateIndex(node.x_index, node.y_index, node.yaw_index));
  return { { disc_pose.x,
             disc_pose.y,
             disc_pose.yaw,
             direction,
             motion_primitive.dir_list_,
             motion_primitive.x_list_,
             motion_primitive.y_list_,
             motion_primitive.yaw_list_,
             motion_primitive.delta_list_,
             types,
             parent_idx,
             cost,
             node.dist + 0.0 } };
}

/**
 * Map continuous pose to discrete state of A* algorithm
 * @param motion_primitive
 * @return
 */
Pose<int> HybridAStar::mapCont2Disc(const MotionPrimitive& motion_primitive)
{
  // Calculate discrete coordinates of reached position to set the reached node
  const int x_ind = static_cast<int>(motion_primitive.x_list_.back() * grid_tf::con2star_);
  const int y_ind = static_cast<int>(motion_primitive.y_list_.back() * grid_tf::con2star_);
  const int yaw_ind = static_cast<int>(round(motion_primitive.yaw_list_.back() * a_star_yaw_res_inv_));
  return { x_ind, y_ind, yaw_ind };
}

/**
 * Get next node by epanding the current motion primitive
 * @param node
 * @param steer
 * @param direction
 * @param motion_res
 * @return
 */
std::optional<NodeHybrid>
HybridAStar::calcNextNode(const NodeHybrid& node, double steer, int direction, double motion_res, double arc_len)
{
  // Move the car in continuous coordinates for a specific arc length
  // Generate motion primitives, Move car some steps
  const double yaw = node.yaw_list.back();
  const Pose<double> pose{ node.x_list.back(), node.y_list.back(), yaw };

  const MotionPrimitive motion_primitive = Vehicle::move_car_some_steps(pose, arc_len, motion_res, direction, steer);

  // Get discrete pose
  const Pose<int> disc_pose = mapCont2Disc(motion_primitive);

  // Verify node on 2d grid
  if (!verifyIndex(disc_pose.x, disc_pose.y))
  {
    return {};
  }

  // Check if car collided
  if (!CollisionChecker::checkPathCollision(
          motion_primitive.x_list_, motion_primitive.y_list_, motion_primitive.yaw_list_))
  {
    return {};
  }

  // calculate costs of path segment of node
  const double path_cost = getPathCosts(disc_pose.x, disc_pose.y, yaw, node, steer, direction, arc_len);

  // Accumulate costs up to this node
  const double cost = node.cost + path_cost;

  // distribute costs over number of steps with given motion inputs to truncate costs when path is driven along later
  const size_t nb_elements = motion_primitive.nb_elements_;
  //  std::vector<double> cont_cost_list(nb_elements, path_cost / static_cast<double>(nb_elements));

  const std::vector<int> directions(nb_elements, direction);

  const auto parent_index = static_cast<int64_t>(calculateIndex(node.x_index, node.y_index, node.yaw_index));

  // Set type of path segment
  const std::vector<PATH_TYPE> types(nb_elements, PATH_TYPE::HASTAR);

  return { { disc_pose.x,
             disc_pose.y,
             disc_pose.yaw,
             direction,
             directions,
             motion_primitive.x_list_,
             motion_primitive.y_list_,
             motion_primitive.yaw_list_,
             motion_primitive.delta_list_,
             types,
             parent_index,
             cost,
             node.dist + arc_len } };
}

/**
 * Create all neighbors based on the motion primitives
 * @param current
 * @param neighbors
 * @param motion_res
 * @param nb_nodes
 */
void HybridAStar::setNeighbors(const NodeHybrid& current, std::vector<NodeHybrid>& neighbors, double motion_res)
{
  neighbors_.clear();
  neighbors_.reserve(NB_CONTROLS);

  // iterate through steering inputs
  std::for_each(std::execution::unseq,
                steering_inputs_.begin(),
                steering_inputs_.end(),
                [&current, &motion_res, &neighbors](auto steer) {
                  for (const int direction : direction_inputs_)
                  {
                    // Node is valid
                    if (auto next_node = calcNextNode(current, steer, direction, motion_res, arc_l_))
                    {
                      neighbors.push_back(std::move(*next_node));
                    }
                  }
                });

  if (can_turn_on_point_)
  {
    // TODO (Schumann) compare distance heuristic by nonh no obs heuristic
    const double turn_on_point_angle_rad = turn_on_point_angle_ * util::TO_RAD;
    if (closed_set_.size() % rear_axis_freq_ == 0)
    {
      // try turning by degree steps
      for (double delta_angle = -2 * util::PI + turn_on_point_angle_rad;
           delta_angle <= 2 * util::PI - turn_on_point_angle_rad;
           delta_angle += turn_on_point_angle_rad)
      {
        if (delta_angle == 0)
        {
          continue;
        }
        if (auto next_node = calcRearAxisNode(current, delta_angle))
        {
          neighbors.push_back(std::move(*next_node));
        }
      }
    }
  }
}
/**
 * * Cost function of turning on the rear axis
* angle diffs smaller 180 are penalized, angle diffs higher even more
*
*
cost
 |                                            /
 -\                                          /
 | ---\                                     /
 |     --\                                 /
 |        --\                             /
 |           ---\                        /
 |               --\                    |
 |                  ---\                /
 |                      --\            /
 |                         --\        /
 |                            ---\   /
 |                                --/
 +---------------------------------|---------------
 0 deg                          180 deg          angle_diff

 * @param delta_angle
 * @return
 */
double HybridAStar::getTurnCost(double delta_angle)
{
  delta_angle = abs(delta_angle);
  const double pi_diff_cost = (std::abs(util::PI - delta_angle) + 0.5 * delta_angle - util::PI / 2) / util::PI;
  return rear_axis_cost_ * (1 + pi_diff_cost);
}

/**
 * Costs of a node that turns on the rear axis
 * @param path
 * @return
 */
double HybridAStar::getRAPathCosts(const ReedsSheppStateSpace::ReedsSheppPath& path)
{
  double cost = 0;
  // length cost
  for (const auto length : path.lengths)
  {
    if (length >= 0)
    {
      cost += length;
    }
    else
    {
      cost += abs(length) * back_cost_;
    }
  }

  // Insert cost for proximity in analytic expansion as well
  double prox_cost = 0;
  for (unsigned int i = 0; i < path.x_list.size(); ++i)
  {
    const Pose<double> pose = { path.x_list[i], path.y_list[i], path.yaw_list[i] };
    prox_cost += getProxOfCorners(pose * grid_tf::con2star_) * h_prox_cost_ * interp_res_;
  }
  cost += prox_cost;

  const double yaw_diff = util::getDrivenAngleDiff(path.yaw_list.front(), path.yaw_list.back(), path.ctypes[1]);

  // General penalty for turning on rear axis, angles different from 180 are penalized more!
  cost += getTurnCost(yaw_diff);

  return cost;
}

/**
 * Calculate costs of a node that is a Reed Shepp etension node
 * @param path
 * @return
 */
double HybridAStar::getRSPathCosts(const ReedsSheppStateSpace::ReedsSheppPath& path)
{
  double cost = 0;
  const double max_steer = atan(Vehicle::w_b_ / path.radi);
  // length cost
  for (auto length : path.lengths)
  {
    if (length >= 0)
    {
      cost += length;
    }
    else
    {
      cost += abs(length) * back_cost_;
    }
  }

  // switch back penalty
  for (unsigned int i = 0; i < path.lengths.size() - 1; ++i)
  {
    if (path.lengths[i] * path.lengths[i + 1] < 0)
    {
      cost += switch_cost_;
    }
  }

  // steer penalty
  size_t idx_ctype = 0;
  for (auto ctype : path.ctypes)
  {
    if (ctype != 'S')
    {
      cost += extra_steer_cost_analytic_ * steer_cost_ * abs(max_steer) * abs(path.lengths[idx_ctype]);
    }
    idx_ctype++;
  }

  // steer change penalty
  std::vector<double> u_list;
  u_list.reserve(path.ctypes.size());
  for (auto ctype : path.ctypes)
  {
    if (ctype == 'R')
    {
      u_list.push_back(max_steer);
    }
    else if (ctype == 'L')
    {
      u_list.push_back(-max_steer);
    }
    else
    {
      u_list.push_back(0);
    }
  }
  for (unsigned int i = 0; i < u_list.size() - 1; ++i)
  {
    cost += steer_change_cost_ * abs(u_list[i + 1] - u_list[i]);
  }

  // Insert cost for proximity in analytic expansion as well
  double prox_cost = 0;
  for (unsigned int i = 0; i < path.x_list.size(); ++i)
  {
    const Pose<double> pose = { path.x_list[i], path.y_list[i], path.yaw_list[i] };
    prox_cost += getProxOfCorners(pose * grid_tf::con2star_) * h_prox_cost_ * interp_res_;
  }
  cost += prox_cost;

  return cost;
}

/**
 * Get path from Reed Shepp extension
 * @param start
 * @param goal
 * @param step_size
 * @param rho
 * @return
 */
ReedsSheppStateSpace::ReedsSheppPath HybridAStar::getReedSheppPath(const Pose<double>& start,
                                                                   const Pose<double>& goal,
                                                                   double step_size,
                                                                   double rho,
                                                                   double wb)
{
  const auto rss = ReedsSheppStateSpace(rho, wb);
  auto path = rss.sample(start, goal, step_size);

  // update length to real one
  // TODO (Schumann) Why is the sum of the sub lengths even wrong? This must be a bug in OMPL
  recalculateLength(path);

  return path;
}

/**
 * Return final path from the list of nodes
 * @return
 */
Path HybridAStar::getFinalPath(const NodeHybrid& final_node, const std::unordered_map<size_t, NodeHybrid>& closet_set)
{
  // get lists
  std::vector<double> reversed_x = final_node.x_list;
  std::vector<double> reversed_y = final_node.y_list;
  std::vector<double> reversed_yaw = final_node.yaw_list;
  std::vector<double> reversed_delta = final_node.delta_list;
  std::vector<int> reversed_direct_cont = final_node.dir_list_cont;
  std::vector<PATH_TYPE> reversed_types = final_node.types;

  // do reverse
  reverse(reversed_x.begin(), reversed_x.end());
  reverse(reversed_y.begin(), reversed_y.end());
  reverse(reversed_yaw.begin(), reversed_yaw.end());
  reverse(reversed_delta.begin(), reversed_delta.end());
  reverse(reversed_direct_cont.begin(), reversed_direct_cont.end());
  reverse(reversed_types.begin(), reversed_types.end());

  // save start point of analytic expansion
  const int len_analytic = static_cast<int>(reversed_x.size());

  int64_t nid = final_node.parent_index;
  while (nid != -1)
  {
    NodeHybrid node = closet_set.at(nid);
    reverse(node.x_list.begin(), node.x_list.end());
    reverse(node.y_list.begin(), node.y_list.end());
    reverse(node.yaw_list.begin(), node.yaw_list.end());
    reverse(node.delta_list.begin(), node.delta_list.end());
    reverse(node.dir_list_cont.begin(), node.dir_list_cont.end());
    reverse(node.types.begin(), node.types.end());
    nid = node.parent_index;

    util::extend_vector(reversed_x, node.x_list);
    util::extend_vector(reversed_y, node.y_list);
    util::extend_vector(reversed_yaw, node.yaw_list);
    util::extend_vector(reversed_delta, node.delta_list);
    util::extend_vector(reversed_direct_cont, node.dir_list_cont);
    util::extend_vector(reversed_types, node.types);
  }

  reverse(reversed_x.begin(), reversed_x.end());
  reverse(reversed_y.begin(), reversed_y.end());
  reverse(reversed_yaw.begin(), reversed_yaw.end());
  reverse(reversed_delta.begin(), reversed_delta.end());
  reverse(reversed_direct_cont.begin(), reversed_direct_cont.end());
  reverse(reversed_types.begin(), reversed_types.end());

  // adjust first direction
  if (reversed_direct_cont.size() > 1)
  {
    reversed_direct_cont[0] = reversed_direct_cont[1];
  }

  // Get index of analytic part
  int idx_start_analytic = -1;
  if (final_node.is_analytic)
  {
    const int nb_elements = static_cast<int>(reversed_x.size());
    idx_start_analytic = nb_elements - len_analytic + 1;
  }

  return { reversed_x,           reversed_y,      reversed_yaw,       reversed_delta,
           reversed_direct_cont, final_node.cost, idx_start_analytic, reversed_types };
}

/**
 * Return distance of a node in a given heuristic
 * @param node
 * @param h_dp
 * @return
 */
double HybridAStar::getDistance2goal(const NodeHybrid& node, const std::unordered_map<size_t, NodeDisc>& h_dp)
{
  size_t ind = AStar::calcIndex(node.x_index, node.y_index);
  auto search = h_dp.find(ind);
  if (search == h_dp.end())
  {
    // index is not valid
    ind = AStar::findValidNeighborIndex(ind, AStar::closed_set_guidance_);

    if (ind == -1)
    {
      return OUT_OF_HEURISTIC;
    }
    const auto backup_search = h_dp.find(ind);
    return backup_search->second.cost_dist_;
  }
  return search->second.cost_dist_;
}

/**
 * Return distance of a node to the global goal
 * @param node
 * @return
 */
double HybridAStar::getDistance2GlobalGoal(const NodeHybrid& node)
{
  return getDistance2goal(node, AStar::closed_set_guidance_);
}

/**
 * Verifies if the analytic expansion (Reeds Shepp curvves) should be triggered
 * @param node
 * @param h_dp
 * @return
 */
bool HybridAStar::check4Expansions(const NodeHybrid& node, const std::unordered_map<size_t, NodeDisc>& h_dp)
{
  const double dist2goal = getDistance2goal(node, h_dp);
  const double dist = (dist_thresh_analytic_ - dist2goal) / dist_thresh_analytic_;
  const double probability = 0 < dist ? dist : 0;
  const double random_number = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
  return (random_number < probability);
}

std::optional<ReedsSheppStateSpace::ReedsSheppPath> HybridAStar::getRSExpansionPath(const NodeHybrid& current,
                                                                                    const NodeHybrid& goal)
{
  const Pose<double> start_pose = { current.x_list.back(), current.y_list.back(), current.yaw_list.back() };
  const Pose<double> goal_pose = { goal.x_list.back(), goal.y_list.back(), goal.yaw_list.back() };

  const ReedsSheppStateSpace::ReedsSheppPath path1 =
      getReedSheppPath(start_pose, goal_pose, motion_res_min_, 1 / Vehicle::max_curvature_, Vehicle::w_b_);
  const ReedsSheppStateSpace::ReedsSheppPath path2 = getReedSheppPath(
      start_pose, goal_pose, motion_res_min_, 1 / (Vehicle::max_curvature_ * second_rs_steer_factor_), Vehicle::w_b_);

  const std::vector<ReedsSheppStateSpace::ReedsSheppPath> paths = { path1, path2 };

  // No paths were found
  if (paths.empty())
  {
    return {};
  }

  // Get best path
  ReedsSheppStateSpace::ReedsSheppPath best_path;
  double best_cost = -1;
  for (const auto& path : paths)
  {
    if (CollisionChecker::checkPathCollision(path.x_list, path.y_list, path.yaw_list))
    {
      // calculate costs
      const double cost = getRSPathCosts(path);

      // Set as best path if costs are lower
      if (best_cost == -1 || cost < best_cost)
      {
        best_cost = cost;
        best_path = path;
        best_path.cost = best_cost;
      }
    }
  }

  // If collision free path was found
  if (best_cost > -1)
  {
    return best_path;
  }

  return {};
}

NodeHybrid HybridAStar::getFinalNodeFromPath(const NodeHybrid& current,
                                             const ReedsSheppStateSpace::ReedsSheppPath& analytic_path,
                                             PATH_TYPE path_type,
                                             double res)
{
  // Final node
  // calc cost of analytic path
  const double f_cost = current.cost + analytic_path.cost;
  const size_t f_parent_index = calculateIndex(current.x_index, current.y_index, current.yaw_index);

  // Set type of path segment
  size_t vec_size = analytic_path.x_list.size() - 1;
  const std::vector<PATH_TYPE> types(vec_size, path_type);

  return { current.x_index,
           current.y_index,
           current.yaw_index,
           current.discrete_direction,
           { analytic_path.directions.begin() + 1, analytic_path.directions.end() },
           { analytic_path.x_list.begin() + 1, analytic_path.x_list.end() },
           { analytic_path.y_list.begin() + 1, analytic_path.y_list.end() },
           { analytic_path.yaw_list.begin() + 1, analytic_path.yaw_list.end() },
           { analytic_path.delta_list.begin() + 1, analytic_path.delta_list.end() },
           types,
           static_cast<int64_t>(f_parent_index),
           f_cost,
           static_cast<double>(vec_size) * res };
}

/**
 * Apply Reed Shepp expansion and get node from RS-Path
 * @param current
 * @param goal
 * @return
 */
std::optional<NodeHybrid> HybridAStar::getRSExpansion(const NodeHybrid& current, const NodeHybrid& goal)
{
  if (const auto analytic_path = getRSExpansionPath(current, goal))
  {
    return getFinalNodeFromPath(current, *analytic_path, PATH_TYPE::REEDS_SHEPP, motion_res_min_);
  }
  return {};
}

void HybridAStar::recalculateLength(ReedsSheppStateSpace::ReedsSheppPath& path)
{
  /**
   * This is a bugfix function to recalculate the length of the RS curves which is probably false in omp
   */
  // Recalculate length because the stored one is false!
  double len_calc = 0;
  for (unsigned int i = 0; i < path.yaw_list.size() - 1; ++i)
  {
    const double diff_x = path.x_list[i + 1] - path.x_list[i];
    const double diff_y = path.y_list[i + 1] - path.y_list[i];
    len_calc += sqrt(pow(diff_x, 2) + pow(diff_y, 2));
  }
  // Correct also vector of partly lengths
  const double length_correction_factor = len_calc / path.totalLength_;
  for (auto& len : path.lengths)
  {
    len *= length_correction_factor;
  }

  path.totalLength_ = len_calc;
}

void HybridAStar::calculateNonhnoobs()
{
  // Prepare non holon no obs heuristic
  non_h_no_obs_.resize_and_reset(astar_yaw_dim_, non_h_no_obs_patch_dim_, non_h_no_obs_patch_dim_, 0);
  non_h_no_obs_.setName("non_h_no_obs");
  non_h_no_obs_calculated_ = true;

  const std::string filename = "/nonh_noobs.data";
  //  LOG_DEB("Trying to find data in " << path2data_ + filename);
  if (std::filesystem::exists(path2data_ + filename))
  {
    //    LOG_DEB("Reading existing non-holonomic no obstacle heuristic");
    std::ifstream input_file(path2data_ + filename);

    input_file.seekg(0);
    input_file.read(reinterpret_cast<char*>(non_h_no_obs_.data_ref().data()),
                    non_h_no_obs_.data_ref().size() * sizeof(double));
    input_file.close();

    return;
  }

  //  LOG_DEB("Calculating non-holonomic no obstacle heuristic");
  const int goal_x = std::floor(non_h_no_obs_patch_dim_ / 2);
  const int goal_y = goal_x;
  constexpr double goal_yaw = 0.0;

  const double max_radius = 1 / Vehicle::max_curvature_;

  for (int angle_idx = 0, angle = -180; angle < 180; angle += astar_yaw_res_deg_, ++angle_idx)
  {
    for (int x_ind = 0; x_ind < non_h_no_obs_patch_dim_; ++x_ind)
    {
      for (int y_ind = 0; y_ind < non_h_no_obs_patch_dim_; ++y_ind)
      {
        const double angle_rad = util::TO_RAD * angle;

        // if goal is equal start set length to zero and continue
        if ((x_ind == goal_x) && (y_ind == goal_y) && (angle_rad == goal_yaw))
        {
          non_h_no_obs_(angle_idx, y_ind, x_ind) = 0.0;
          continue;
        }

        // set start and goal coordinates in meters to ensure the length is in meters
        const Pose<double> start = { static_cast<double>(x_ind) * astar_res_,
                                     static_cast<double>(y_ind) * astar_res_,
                                     angle_rad };
        const Pose<double> goal = { static_cast<double>(goal_x) * astar_res_,
                                    static_cast<double>(goal_y) * astar_res_,
                                    goal_yaw };

        // do Reeds Shepp planning
        ReedsSheppStateSpace::ReedsSheppPath path =
            getReedSheppPath(start, goal, motion_res_min_, max_radius, Vehicle::w_b_);
        non_h_no_obs_(angle_idx, y_ind, x_ind) = path.totalLength_;
      }
    }
  }

  // write vector to file
  std::ofstream output_file(path2data_ + filename);
  output_file.write(reinterpret_cast<char*>(non_h_no_obs_.data_ref().data()),
                    non_h_no_obs_.data_ref().size() * sizeof(double));
  output_file.close();
}

double HybridAStar::det(const Point<double>& vec1, const Point<double>& vec2)
{
  return vec1.x * vec2.y - vec1.y * vec2.x;
}

bool HybridAStar::lineIntersection(const Line2D<double>& line1,
                                   const Line2D<double>& line2,
                                   const Pose<double>& start,
                                   const Pose<double>& goal,
                                   double length,
                                   Point<double>& intersect_point,
                                   Point<double>& s2intersect)
{
  const Point<double> xdiff = { line1.p1.x - line1.p2.x, line2.p1.x - line2.p2.x };
  const Point<double> ydiff = { line1.p1.y - line1.p2.y, line2.p1.y - line2.p2.y };

  const double div = det(xdiff, ydiff);
  if (div == 0)
  {
    return false;
  }

  const Point<double> det_point = { det(line1.p1, line1.p2), det(line2.p1, line2.p2) };
  const double x_val = det(det_point, xdiff) / div;
  const double y_val = det(det_point, ydiff) / div;

  // Get parametric position of intersection point from x_val component
  double div_x = cos(start.yaw);
  double div_y;
  double s_1;
  double s_2;
  if (abs(div_x) > 0.01)
  {
    s_1 = (x_val - start.x) / div_x;
  }
  else
  {
    div_y = sin(start.yaw);
    s_1 = (y_val - start.y) / div_y;
  }

  div_x = cos(goal.yaw);
  if (abs(div_x) > 0.01)
  {
    s_2 = (x_val - goal.x) / div_x;
  }
  else
  {
    div_y = sin(goal.yaw);
    s_2 = (y_val - goal.y) / div_y;
  }

  // Intersection within lines
  if ((length > s_1) && (s_1 > -length) && (length > s_2) && (s_2 > -length))
  {
    intersect_point.x = x_val;
    intersect_point.y = y_val;
    s2intersect.x = s_1;
    s2intersect.y = s_2;
    return true;
  }
  // Intersection outside of lines
  return false;
}

/**
 *  Try to intersect two lines. If they do, this is the turning point
 * @param current_node
 * @param goal_node
 * @return
 */
std::optional<NodeHybrid> HybridAStar::getRearAxisPath(const NodeHybrid& current_node, const NodeHybrid& goal_node)
{
  const Pose<double> start = { current_node.x_list.back(), current_node.y_list.back(), current_node.yaw_list.back() };
  const Pose<double> goal = { goal_node.x_list.back(), goal_node.y_list.back(), goal_node.yaw_list.back() };

  const double turn_s = turn_on_point_horizon_;

  // line 1
  const Point<double> pointA = { start.x - turn_s * cos(start.yaw), start.y - turn_s * sin(start.yaw) };
  const Point<double> pointB = { start.x + turn_s * cos(start.yaw), start.y + turn_s * sin(start.yaw) };
  const Line2D<double> line1 = { pointA, pointB };

  // line 2
  const Point<double> pointC = { goal.x - turn_s * cos(goal.yaw), goal.y - turn_s * sin(goal.yaw) };
  const Point<double> pointD = { goal.x + turn_s * cos(goal.yaw), goal.y + turn_s * sin(goal.yaw) };
  const Line2D<double> line2 = { pointC, pointD };

  Point<double> intersect_points = { 0, 0 };
  Point<double> s2intersect = { 0, 0 };
  if (lineIntersection(line1, line2, start, goal, turn_s, intersect_points, s2intersect))
  {
    // Get coordinate where lines intersect
    const double s_1 = s2intersect.x;
    const double s_2 = s2intersect.y;
    double ds1 = (s_1 >= 0) ? interp_res_ : -interp_res_;
    size_t nb_el_1 = ceil(s_1 / ds1);
    ds1 = s_1 / static_cast<double>(nb_el_1 - 1);
    double ds2 = (s_2 >= 0) ? interp_res_ : -interp_res_;
    size_t nb_el_2 = ceil(s_2 / ds2);
    ds2 = s_2 / static_cast<double>(nb_el_2 - 1);

    // Get angles of turning point
    const double diff = util::getAngleDiff(start.yaw, goal.yaw);
    const char turn_direction = (diff > 0) ? 'L' : 'R';
    const std::vector<double> yaw_fill =
        util::angleArange(start.yaw, goal.yaw, turn_direction, yaw_res_coll_ * util::TO_RAD);
    const size_t nb_fill = yaw_fill.size();
    const size_t total_size = nb_el_1 + nb_fill + nb_el_2;
    std::vector<double> x_list(total_size, 0);
    std::vector<double> y_list(total_size, 0);
    std::vector<double> yaw_list(total_size, 0);
    std::vector<double> delta_list(total_size, 0);
    std::vector<int> dir_list(total_size, 2 * static_cast<int>((ds1 > 0)) - 1);
    const size_t idx_first = 0;
    const size_t idx_fill = nb_el_1;
    const size_t idx_second = nb_el_1 + nb_fill;

    // interpolate and insert second part
    // set first elements
    int direction = 2 * static_cast<int>(ds1 > 0) - 1;
    double s_val = 0;
    for (size_t i = idx_first; i < idx_fill; ++i)
    {
      x_list[i] = start.x + s_val * cos(start.yaw);
      y_list[i] = start.y + s_val * sin(start.yaw);
      yaw_list[i] = start.yaw;
      delta_list[i] = 0.0;
      dir_list[i] = direction;
      s_val += ds1;
    }
    const double last_x = x_list[idx_fill - 1];
    const double last_y = y_list[idx_fill - 1];

    // interpolate angle
    for (size_t i = idx_fill; i < idx_second; ++i)
    {
      x_list[i] = last_x;
      y_list[i] = last_y;
      yaw_list[i] = yaw_fill[i - idx_fill];
      dir_list[i] = direction;

      if (turn_direction == 'L')
      {
        delta_list[i] = util::PI / 2;
      }
      else
      {
        delta_list[i] = -util::PI / 2;
      }
    }

    // interpolate and insert first part
    // set first elements
    direction = 2 * static_cast<int>(ds2 < 0) - 1;
    s_val = 0;
    for (size_t i = total_size - 1; i > idx_second - 1; --i)
    {
      x_list[i] = goal.x + s_val * cos(goal.yaw);
      y_list[i] = goal.y + s_val * sin(goal.yaw);
      yaw_list[i] = goal.yaw;
      delta_list[i] = 0.0;
      dir_list[i] = direction;
      s_val += ds2;
    }

    if (CollisionChecker::checkPathCollision(x_list, y_list, yaw_list))
    {
      // TODO (Schumann) create constructor for this
      ReedsSheppStateSpace::ReedsSheppPath path;
      path.x_list = std::move(x_list);
      path.y_list = std::move(y_list);
      path.yaw_list = std::move(yaw_list);
      path.delta_list = std::move(delta_list);
      path.directions = std::move(dir_list);
      path.ctypes = { 'S', turn_direction, 'S' };
      path.lengths = { s_1, 0, s_2 };
      path.totalLength_ = s_1 + s_2;
      path.cost = getRAPathCosts(path);

      return getFinalNodeFromPath(current_node, path, PATH_TYPE::REAR_AXIS, interp_res_);
    }
    return {};
  }
  return {};
}

/**
 * Creates a node for the python side
 * @param x
 * @param y
 * @param yaw
 * @return
 */
NodeHybrid HybridAStar::createNode(const Pose<double>& pose, double steer)
{
  return { static_cast<int>(round(pose.x * grid_tf::con2star_)),
           static_cast<int>(round(pose.y * grid_tf::con2star_)),
           static_cast<int>(round(pose.yaw / astar_yaw_res_)),
           1,
           { 1 },
           { pose.x },
           { pose.y },
           { pose.yaw },
           { steer },
           { PATH_TYPE::HASTAR },
           -1,
           0,
           0 };
}

/**
 * Recalculate the complete planning env.
 * The distance heuristic is calculated from the goal to the current ego position to allow the discrete A* path to be
 * used as a navigation help. Inside the planning alg. the heuristic is calculated explicitly from the planned pose
 * @param goal_node
 * @param ego_node
 */
void HybridAStar::recalculateEnv(const NodeHybrid& goal_node, const NodeHybrid& ego_node)
{
  AStar::calcAstarGridCuda();

  const Point<int> ego_index = { ego_node.x_index, ego_node.y_index };

  AStar::calcVoronoiPotentialField(ego_index);

  AStar::calcDistanceHeuristic({ goal_node.x_index, goal_node.y_index }, { ego_node.x_index, ego_node.y_index }, false);
}

void HybridAStar::resetLaneGraph()
{
  lane_graph_.reset();
}

void HybridAStar::updateLaneGraph(const Point<double>& origin_utm, double patch_dim)
{
  lane_graph_.init(origin_utm, patch_dim);

  AStar::resetMovementMap();

  AStar::setMovementMap(lane_graph_.edges_);
}

void HybridAStar::smoothLaneNodes(std::vector<LaneNode>& nodes)
{
  size_t nb_elements = nodes.size();
  std::vector<double> s_list;
  std::vector<double> x_list;
  std::vector<double> y_list;
  s_list.reserve(nb_elements);
  x_list.reserve(nb_elements);
  y_list.reserve(nb_elements);

  double s_dist = 0.0;
  int idx = 0;
  // TODO (Schumann) replace with enumerate from c++23
  for (const auto& node : nodes)
  {
    const auto& [x, y] = node.point_utm;

    if (idx > 0)
    {
      const double x_diff = x - x_list.back();
      const double y_diff = y - y_list.back();
      const double dist = x_diff * x_diff + y_diff * y_diff;
      s_dist += dist;
    }
    s_list.push_back(s_dist);
    x_list.push_back(x);
    y_list.push_back(y);

    idx++;
  }

  int constexpr degree = 3;
  int constexpr smoothing = 1.0;
  auto spline_x = fitpack_wrapper::BSpline1D(s_list, x_list, degree, smoothing);
  auto spline_y = fitpack_wrapper::BSpline1D(s_list, y_list, degree, smoothing);

  // insert elements into positions
  idx = 0;
  for (const auto& s_val : s_list)
  {
    nodes[idx].point_utm = { spline_x(s_val), spline_y(s_val) };
    idx++;
  }
}

void HybridAStar::smoothLaneGraph(LaneGraph& lane_graph)
{
  smoothLaneNodes(lane_graph.nodes_);
  lane_graph.reinit();
}

void HybridAStar::interpolateLaneGraph(LaneGraph& lane_graph)
{
  interpolateLaneNodes(lane_graph.nodes_);
  lane_graph.reinit();
}

void HybridAStar::interpolateLaneNodes(std::vector<LaneNode>& nodes)
{
  size_t nb_elements = nodes.size();
  std::vector<double> s_list;
  std::vector<double> x_list;
  std::vector<double> y_list;
  s_list.reserve(nb_elements);
  x_list.reserve(nb_elements);
  y_list.reserve(nb_elements);

  double s_dist = 0.0;
  int idx = 0;
  // TODO (Schumann) replace with enumerate from c++23
  for (const auto& node : nodes)
  {
    const auto& [x, y] = node.point_utm;

    if (idx > 0)
    {
      const double x_diff = x - x_list.back();
      const double y_diff = y - y_list.back();
      const double dist = x_diff * x_diff + y_diff * y_diff;
      s_dist += dist;
    }
    s_list.push_back(s_dist);
    x_list.push_back(x);
    y_list.push_back(y);
    idx++;
  }

  // very rudimentary interpolation, distance is approximately right
  int constexpr degree = 3;
  int constexpr smoothing = 1.0;
  auto spline_x = fitpack_wrapper::BSpline1D(s_list, x_list, degree, smoothing);
  auto spline_y = fitpack_wrapper::BSpline1D(s_list, y_list, degree, smoothing);

  // insert interpolated elements
  const int nb_els = static_cast<int>(s_list.back() / motion_res_max_);
  nodes.clear();
  nodes.reserve(nb_els);

  s_dist = 0;
  while (s_dist < s_list.back())
  {
    nodes.emplace_back(Point<double>(spline_x(s_dist), spline_y(s_dist)));
    s_dist += motion_res_max_;
  }
}

std::optional<NodeHybrid> HybridAStar::hAstarCore(const NodeHybrid& ego_node,
                                                  const NodeHybrid& start_node,
                                                  const NodeHybrid& goal_node,
                                                  bool do_analytic)
{
  open_set_.clear();
  closed_set_.clear();
  connected_closed_nodes_.first.clear();   // reset for correct vis
  connected_closed_nodes_.second.clear();  // reset for correct vis

  //  std::unordered_map<size_t, NodeDisc>* dist_heuristic;

  // Calc distance heuristic prior to path calculation
  AStar::calcDistanceHeuristic(
      { goal_node.x_index, goal_node.y_index }, { start_node.x_index, start_node.y_index }, true);
  //  dist_heuristic = &AStar::closed_set_path_;

  // Get distance from ego position
  const double start_heur_cost = getDistance2goal(ego_node, AStar::closed_set_guidance_);

  // Add start node to frontier to explore from
  const size_t start_index = calculateIndex(start_node.x_index, start_node.y_index, start_node.yaw_index);
  open_queue_.put(start_index, calcCost(start_node, goal_node, AStar::closed_set_path_));
  open_set_.insert({ start_index, start_node });

  size_t curr_open_idx;
  std::vector<NodeHybrid> final_nodes;
  size_t nb_nodes_since_final = 0;

  const auto t_1 = std::chrono::high_resolution_clock::now();
  const std::chrono::duration<double, std::milli> timeout = std::chrono::milliseconds(timeout_ms_);
  while (true)
  {
    // open set is empty
    if (open_queue_.empty())
    {
      //      LOG_ERR("Cannot find path, No open set");
      std::cout << "Cannot find path, no open set" << std::endl;
      return {};
    }
    // Execution took too long
    const auto t_2 = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double, std::milli> ms_double = t_2 - t_1;
    if (ms_double > timeout)
    {
      //      LOG_ERR("Execution took too long, no path was found");
      std::cout << "Execution took too long, no path was found" << std::endl;
      return {};
    }

    // Get node with the lowest costs of open list to be investigated
    curr_open_idx = open_queue_.get();

    // Node is in open list --> move to explored ones
    const auto search_current = open_set_.find(curr_open_idx);
    if (search_current != open_set_.end())
    {
      const NodeHybrid current_node = search_current->second;
      closed_set_.insert({ curr_open_idx, current_node });
      open_set_.erase(curr_open_idx);

      if (do_analytic)
      {
        /// Heuristic reduction
        const double current_heur_cost = getDistance2goal(current_node, AStar::closed_set_path_);
        if (waypoint_type_ == HEUR_RED and start_heur_cost > max_dist4waypoints_)
        {
          // Early stopping based on heuristic decrease
          const double heur_diff = start_heur_cost - current_heur_cost;
          if (heur_diff > waypoint_dist_)
          {
            return current_node;
          }
        }

        /// Try to reach final goal with extensions
        if (check4Expansions(current_node, AStar::closed_set_path_))
        {
          // A valid final path was found by RS curves
          if (const auto final_rs_node = getRSExpansion(current_node, goal_node))
          {
            final_nodes.push_back(std::move(*final_rs_node));
          }

          // Try finding final node by turning on rear axis
          if (can_turn_on_point_)
          {
            // Final path was found
            if (const auto final_ra_node = getRearAxisPath(current_node, goal_node))
            {
              final_nodes.push_back(std::move(*final_ra_node));
            }
          }

          // Breakout after x additional nodes were found
          if (!final_nodes.empty())
          {
            nb_nodes_since_final++;
            if (nb_nodes_since_final > max_extra_nodes_)
            {
              // Take final nodes with the lowest cost
              std::sort(final_nodes.begin(), final_nodes.end());
              NodeHybrid final_node = final_nodes.front();
              final_node.set_analytic();

              return final_node;
            }
          }
        }
      }
      // Only approximate goal
      else
      {
        /// Waypoint path
        const double node_angle = current_node.yaw_list.back();
        const double goal_angle = goal_node.yaw_list.front();

        // If no analytic solution is necessary, waypoints must be reached only approximately
        const double dist2 = pow((current_node.x_list.back() - goal_node.x_list.front()), 2) +
                             pow((current_node.y_list.back() - goal_node.y_list.front()), 2);
        if (dist2 < approx_goal_dist2_ && util::areAnglesEqual(goal_angle, node_angle, approx_goal_angle_rad_))
        {
          return current_node;
        }
      }

      // Change the motion resolution depending on the free space
      const double motion_res = AStar::motion_res_map_(current_node.y_index, current_node.x_index);

      // Get neighbour nodes that can be reached from current one by applying steering
      setNeighbors(current_node, neighbors_, motion_res);
      for (const auto& neighbor : neighbors_)
      {
        // Get unique index
        const size_t next_idx = calculateIndex(neighbor.x_index, neighbor.y_index, neighbor.yaw_index);

        // Node was already visited
        if (closed_set_.contains(next_idx))
        {
          // The update of a closed node cannot be implemented because it would cause steps in the path
          continue;
        }

        // Node is already in open list
        const auto search_next = open_set_.find(next_idx);
        if (search_next != open_set_.end())
        {
          const NodeHybrid found_node = search_next->second;
          if (found_node.cost > neighbor.cost)
          {
            // Add to cost
            const double node_cost = calcCost(neighbor, goal_node, AStar::closed_set_path_);

            open_queue_.put(next_idx, node_cost);
            open_set_.erase(next_idx);
            open_set_.emplace(next_idx, neighbor);
          }
        }
        // Node is not in open list and hence unknown, add it to open list!
        else
        {
          // Add to cost
          const double node_cost = calcCost(neighbor, goal_node, AStar::closed_set_path_);

          // Check if node is out of heuristic, if yes don't add it to open list
          if (node_cost == OUT_OF_HEURISTIC)
          {
            continue;
          }

          open_queue_.put(next_idx, node_cost);
          open_set_.insert({ next_idx, neighbor });
        }
      }
      // Node from heap is not in open list, This happens by updating the nodes in the open list!
    }
    else
    {
      // Ignore it, it was already processed
      continue;
    }
  }
}

std::optional<Path> HybridAStar::hybridAStarPlanning(const NodeHybrid& ego_node,
                                                     const NodeHybrid& start_node,
                                                     const NodeHybrid& goal_node,
                                                     bool do_analytic,
                                                     double speed)
{
  if (const auto final_node = hAstarCore(ego_node, start_node, goal_node, do_analytic))
  {
    Path path = getFinalPath(*final_node, closed_set_);

    // Smooth path with gradient descent
    //    Smoother::smooth_path(path);

    // interpolate path with B-Splines
    interpolatePath(path, interp_res_);

    // create nodes for vis
    calcConnectedClosedNodes();

    return path;
  }

  // TODO (Schumann) create emergency path if no path is found!
  return getEmergencyPath(start_node, speed);
}

/**
 * Find a valid pose next to the original goal to plan to
 * @param ego_pose
 * @param goal_pose
 * @return
 */
std::optional<Pose<double>> HybridAStar::getValidClosePose(const Pose<double>& ego_pose, const Pose<double>& goal_pose)
{
  // 1. do forward sweep to goal
  const double steer = 0;
  const NodeHybrid goal_node = createNode(goal_pose, steer);
  const NodeHybrid ego_node = createNode(ego_pose, steer);

  bool for_path = false;
  bool get_only_near = true;
  // inversed on purpose because we want to sweep from start to goal
  AStar::calcDistanceHeuristic(
      { ego_node.x_index, ego_node.y_index }, { goal_node.x_index, goal_node.y_index }, for_path, get_only_near);

  // 2. take closest node
  double min_dist = 1e9;
  NodeDisc closest_node = ego_node.getNodeDisc();
  for (auto [dist, node] : AStar::nodes_near_goal_)
  {
    if (dist < min_dist)
    {
      min_dist = dist;
      closest_node = node;
    }
  }

  // 3. find collision free pose around it
  const Pose<double> center_pose = { closest_node.pos.x * grid_tf::star2con_,
                                     closest_node.pos.y * grid_tf::star2con_,
                                     goal_node.yaw_list[0] };

  const double angle_weight = 20.0;
  const double dist_weight = 1.0;
  const double prox_weight = 1.0;

  const double dxy_max = 5.0;
  const double dphi_max = util::PI;

  const double phi_res = yaw_res_coll_ * util::TO_RAD;

  // default to ego pose
  Pose<double> free_goal_pose = ego_pose;

  double min_cost = std::numeric_limits<double>::max();

  const int dxy_max_idx = static_cast<int>(std::floor(dxy_max) / gm_res_);
  const int dphi_max_idx = static_cast<int>(std::floor(dphi_max) / phi_res);

  for (int dxi = -dxy_max_idx; dxi < dxy_max_idx; ++dxi)
  {
    for (int dyi = -dxy_max_idx; dyi < dxy_max_idx; ++dyi)
    {
      for (int dphii = -dphi_max_idx; dphii < dphi_max_idx; ++dphii)
      {
        const double x_diff = dxi * gm_res_;
        const double y_diff = dyi * gm_res_;
        const double phi_diff = dphii * phi_res;

        // Calculate costs: distance to actual goal
        const double dist2 = x_diff * x_diff + y_diff * y_diff;

        const Pose<double> pose = { center_pose.x + x_diff, center_pose.y + y_diff, center_pose.yaw + phi_diff };

        const double prox_cost = getProxOfCorners(grid_tf::utm2astar(pose));

        // naive cost function
        const double cost = dist2 * dist_weight + std::abs(phi_diff) * angle_weight + prox_cost * prox_weight;

        if (cost > min_cost)
        {
          continue;
        }

        // check if inside list of nearest goal
        const Point<int> index2d = grid_tf::utm2astar(pose.getPoint()).toInt();
        const size_t index = AStar::calcIndex(index2d.x, index2d.y);

        bool found = false;
        for (const auto& [dist, node] : AStar::nodes_near_goal_)
        {
          if (index == AStar::calcIndex(node))
          {
            found = true;
            break;
          }
        }
        if (not found)
        {
          continue;
        }

        // check against collision
        if (CollisionChecker::checkPose(pose))
        {
          free_goal_pose = pose;
          min_cost = cost;
        }
      }
    }
  }

  return free_goal_pose;
}

/**
 * Create coordinates of closed nodes if they were just calculated
 * @return
 */
void HybridAStar::calcConnectedClosedNodes()
{
  // reconnect all nodes with the last coordinate of their parent
  connected_closed_nodes_.first.resize(closed_set_.size() * 2);
  connected_closed_nodes_.second.resize(closed_set_.size() * 2);
  size_t index = 0;
  for (auto& node_pair : closed_set_)
  {
    const NodeHybrid node = node_pair.second;
    if (node.parent_index != -1)
    {
      const NodeHybrid parent_node = closed_set_.at(node.parent_index);

      // Add to list for vis
      connected_closed_nodes_.first.at(2 * index) = grid_tf::patch_utm2utm_x(parent_node.x_list.back());
      connected_closed_nodes_.first.at(2 * index + 1) = grid_tf::patch_utm2utm_x(node.x_list.back());
      connected_closed_nodes_.second.at(2 * index) = grid_tf::patch_utm2utm_y(parent_node.y_list.back());
      connected_closed_nodes_.second.at(2 * index + 1) = grid_tf::patch_utm2utm_y(node.y_list.back());
      index++;
    }
  }
}

/**
 *  Do a graph expansion for as many iterations as necessary to the current maximal acceleration
 * @param start_node
 * @param ego_v
 * @return
 */
Path HybridAStar::getEmergencyPath(const NodeHybrid& start_node, double ego_v)
{
  std::unordered_map<size_t, NodeHybrid> closed_set;
  std::unordered_map<size_t, NodeHybrid> open_set;
  PriorityQueue<size_t, double> open_queue;

  constexpr double v_eps = 0.1;
  if (ego_v < v_eps)
  {
    return Path({ start_node.x_list.back() },
                { start_node.y_list.back() },
                { start_node.yaw_list.back() },
                { start_node.delta_list.back() },
                { start_node.dir_list_cont.back() },
                start_node.cost,
                -1,
                { start_node.types.back() });
  }
  const int direction = (ego_v >= 0) ? 1 : -1;
  const double motion_res = interp_res_;
  const double comfort_arc_l = std::max(pow(ego_v, 2) / (2 * comfort_brake_acc_), motion_res);

  // Add start to open set
  const size_t start_idx = calculateIndex(start_node.x_index, start_node.y_index, start_node.yaw_index);
  open_set.insert({ start_idx, start_node });
  open_queue.put(start_idx, start_node.cost);

  const auto t_1 = std::chrono::high_resolution_clock::now();
  const std::chrono::duration<double, std::milli> timeout = std::chrono::milliseconds(100);

  // Do a graph expansion
  NodeHybrid final_node;
  while (true)
  {
    if (open_queue.empty())
    {
      //            LOG_ERR("Cannot find emergency path, No open set");
      std::cout << "Cannot find emergency path, No open set. The best I can do is returning the current node as path. "
                   "Good luck"
                << std::endl;
      return Path({ start_node.x_list.back() },
                  { start_node.y_list.back() },
                  { start_node.yaw_list.back() },
                  { start_node.delta_list.back() },
                  { start_node.dir_list_cont.back() },
                  start_node.cost,
                  -1,
                  { start_node.types.back() });
    }

    // Get node with the lowest costs of open list to be investigated
    const size_t curr_open_idx = open_queue.get();

    // Node is not yet explored --> move to explored ones
    const auto search_current = open_set.find(curr_open_idx);
    if (search_current != open_set.end())
    {
      const NodeHybrid current_node = search_current->second;
      closed_set.insert({ curr_open_idx, current_node });
      open_set.erase(curr_open_idx);

      // Execution took too long
      const auto t_2 = std::chrono::high_resolution_clock::now();
      const std::chrono::duration<double, std::milli> ms_double = t_2 - t_1;

      // timeout or comfort arc length reached
      if (ms_double > timeout or current_node.dist > comfort_arc_l)

      //      if (current_node.dist > min_arc_l)
      {
        std::cout << "Emergency path found. remaining s = " << current_node.dist << std::endl;
        return getFinalPath(current_node, closed_set);
      }

      // Explore
      for (const double steer : steering_inputs_)
      {
        if (const auto next_node = calcNextNode(current_node, steer, direction, motion_res, arc_l_))
        {
          if (verifyIndex(next_node->x_index, next_node->y_index))
          {
            const size_t next_idx = calculateIndex(next_node->x_index, next_node->y_index, next_node->yaw_index);
            // Node was already visited
            if (closed_set.find(next_idx) != closed_set.end())
            {
              // The update of a closed node cannot be implemented because it would cause steps in the path
              continue;
            }

            // Node is not in open list and hence unknown, add it to open list!
            const auto search = open_set.find(next_idx);
            if (search == open_set.end())
            {
              open_queue.put(next_idx, next_node->cost);
              open_set.insert({ next_idx, *next_node });

              // Node is already in open list
            }
            else
            {
              const NodeHybrid found_node = search->second;
              if (found_node.cost > next_node->cost)
              {
                open_queue.put(next_idx, next_node->cost);
                open_set.erase(next_idx);
                open_set.emplace(next_idx, *next_node);
              }
            }
          }
        }
      }
    }
    else
    {
      // Node from heap is not in open list, This happens by updating the nodes in the open list!
      // Ignore it, it was already processed
      continue;
    }
  }
}

std::tuple<Pose<double>, int, double> HybridAStar::projEgoOnPath(const Pose<double>& pose,
                                                                 const Path& path,
                                                                 int ego_idx)
{
  const int max_horizon = 60;
  const int nb_elements = static_cast<int>(path.x_list.size());

  // nb of elements ahead is zero
  if (nb_elements - ego_idx <= 0)
  {
    return { pose, 0, 0 };
  }

  const int start_idx = std::max(ego_idx - max_horizon, 0);
  const int end_idx = std::min(ego_idx + max_horizon, nb_elements);

  // calculate euclidian and angular distance
  double min_dist_metric = std::numeric_limits<double>::max();
  int proj_idx = ego_idx;
  for (int i = start_idx; i < end_idx; ++i)
  {
    double x_diff = (pose.x - path.x_list[i]);
    double y_diff = (pose.y - path.y_list[i]);
    double dist2 = x_diff * x_diff + y_diff * y_diff;
    double dist_yaw = util::getAngleDiff(path.yaw_list[i], pose.yaw);

    double dist_metric = dist2 + std::abs(dist_yaw) * 0.0001;

    if (dist_metric < min_dist_metric)
    {
      proj_idx = i;
      min_dist_metric = dist_metric;
    }
  }

  // get pose
  const Pose<double> proj_pose(path.x_list[proj_idx], path.y_list[proj_idx], path.yaw_list[proj_idx]);

  return { proj_pose, proj_idx, min_dist_metric };
}

void HybridAStar::interpolatePathSegment(Path& path_segment, const Segment& segment_info, double interp_res)
{
  size_t path_length = path_segment.x_list.size();

  // Skip if path is empty or contains only one element
  if (path_length <= 1)
  {
    return;
  }
  // Calculate distance vector
  std::vector<double> distances;
  distances.reserve(path_length);
  std::vector<double> path_x_filt;
  path_x_filt.reserve(path_length);
  std::vector<double> path_y_filt;
  path_y_filt.reserve(path_length);
  std::vector<double> path_yaw_filt;
  path_yaw_filt.reserve(path_length);
  std::vector<double> path_delta_filt;
  path_delta_filt.reserve(path_length);
  std::vector<PATH_TYPE> path_types_filt;
  path_types_filt.reserve(path_length);

  // Insert first element
  path_x_filt.push_back(path_segment.x_list.front());
  path_y_filt.push_back(path_segment.y_list.front());
  path_yaw_filt.push_back(path_segment.yaw_list.front());
  path_delta_filt.push_back(path_segment.delta_list.front());
  path_types_filt.push_back(path_segment.types.front());

  double cum_dist = 0;
  distances.push_back(0);
  for (size_t idx = 1; idx < path_length; ++idx)
  {
    const double x_diff = path_segment.x_list[idx] - path_segment.x_list[idx - 1];
    const double y_diff = path_segment.y_list[idx] - path_segment.y_list[idx - 1];
    const double dist = sqrt(x_diff * x_diff + y_diff * y_diff);

    // ignore duplicate points
    if (dist < 0.01)
    {
      continue;
    }
    cum_dist += dist;
    // add distance
    distances.push_back(cum_dist);
    // add coordinates
    path_x_filt.push_back(path_segment.x_list[idx]);
    path_y_filt.push_back(path_segment.y_list[idx]);
    path_yaw_filt.push_back(path_segment.yaw_list[idx]);
    path_delta_filt.push_back(path_segment.delta_list[idx]);
    path_types_filt.push_back(path_segment.types[idx]);
  }

  // Update path length
  path_length = path_x_filt.size();
  // Nothing to interpolate
  if (path_length <= 1)
  {
    return;
  }

  const int spline_degree = std::min(MAX_INTERPOLATION_DEG, static_cast<int>(path_length - 1));
  // create spline functions
  auto spline_x = fitpack_wrapper::BSpline1D(distances, path_x_filt, spline_degree, 0.0);
  auto spline_y = fitpack_wrapper::BSpline1D(distances, path_y_filt, spline_degree, 0.0);
  auto spline_delta = fitpack_wrapper::BSpline1D(distances, path_delta_filt, spline_degree, 0.0);

  // Save last values to set the values at the end of segment again
  const double last_x = path_segment.x_list.back();
  const double last_y = path_segment.y_list.back();
  const double last_yaw = path_segment.yaw_list.back();
  const double last_delta = path_segment.delta_list.back();
  const int last_dir = path_segment.direction_list.back();
  const PATH_TYPE last_type = path_segment.types.back();

  // clear path_segment to add new ones. Keep first element
  path_segment.x_list.resize(1);
  path_segment.y_list.resize(1);
  path_segment.yaw_list.resize(1);
  path_segment.delta_list.resize(1);
  path_segment.direction_list.resize(1);
  path_segment.types.resize(1);

  const PATH_TYPE type = segment_info.path_type;

  exact_dist_interpolation(
      path_segment, cum_dist, spline_x, spline_y, spline_delta, distances, path_yaw_filt, last_dir, type, interp_res);

  // ensure last pose and values
  path_segment.x_list.back() = last_x;
  path_segment.y_list.back() = last_y;
  path_segment.yaw_list.back() = last_yaw;
  path_segment.delta_list.back() = last_delta;
  path_segment.direction_list.back() = last_dir;
  path_segment.types.back() = last_type;
}

void HybridAStar::exact_dist_interpolation(Path& path_segment,
                                           double cum_dist,
                                           fitpack_wrapper::BSpline1D& spline_x,
                                           fitpack_wrapper::BSpline1D& spline_y,
                                           fitpack_wrapper::BSpline1D& spline_delta,
                                           const std::vector<double>& distances,
                                           const std::vector<double>& path_yaw_filt,
                                           int last_dir,
                                           const PATH_TYPE type,
                                           double interp_res)
{
  // reserve a guess
  size_t approx_nb_new_elements = ceil(cum_dist / interp_res);
  path_segment.x_list.reserve(approx_nb_new_elements);
  path_segment.y_list.reserve(approx_nb_new_elements);
  path_segment.yaw_list.reserve(approx_nb_new_elements);
  path_segment.delta_list.reserve(approx_nb_new_elements);
  path_segment.direction_list.reserve(approx_nb_new_elements);
  path_segment.types.reserve(approx_nb_new_elements);

  auto prev_test_point = Point<double>(path_segment.x_list.front(), path_segment.y_list.front());

  // Interpolate path
  double s_eval = 0;
  double dist_exact_point = 0;
  // we hope this is small enough. actually one should intersect a circle with the spline
  const double step_s = interp_res / 10;
  // last distance will always be between 2*interp_res and interp_res
  while (s_eval < cum_dist)
  {
    // eval point of spline
    s_eval += step_s;
    const auto test_point = Point<double>(spline_x(s_eval), spline_y(s_eval));

    // sum little step distances
    const double dist_prev_test_p = test_point.dist2(prev_test_point);
    dist_exact_point += dist_prev_test_p;

    // Valid point
    if (dist_exact_point > interp_res)
    {
      // interpolate between test points
      const double dist_too_far = dist_exact_point - interp_res;
      const double t = step_s - dist_too_far;
      const double x = std::lerp(prev_test_point.x, test_point.x, t);
      const double y = std::lerp(prev_test_point.y, test_point.y, t);

      // interpolate delta
      const double delta = spline_delta(s_eval + t);

      // interpolate yaw normally
      const double yaw = util::angleInterpolation(s_eval + t, distances, path_yaw_filt);

      // add values
      path_segment.x_list.push_back(x);
      path_segment.y_list.push_back(y);
      path_segment.yaw_list.push_back(yaw);
      path_segment.delta_list.push_back(delta);
      path_segment.direction_list.push_back(last_dir);
      path_segment.types.push_back(type);

      // start search again from this point
      dist_exact_point = dist_too_far;
    }
    // set point as prev_point
    prev_test_point = Point<double>(test_point);
  }
}

void HybridAStar::interpolatePath(Path& path, double interp_res)
{
  const size_t path_length = path.x_list.size();

  // Copy original path and clear the one later
  const Path orig_path(path);

  // Skip if path is empty or contains only one element
  if (path_length <= 1)
  {
    return;
  }
  // Get path segments
  std::vector<Segment> segments;
  PATH_TYPE path_type = PATH_TYPE::UNKNOWN;
  size_t segment_idx = 0;
  for (size_t idx = 0; idx < path_length; ++idx)
  {
    // Path type changed or path direction changes
    const PATH_TYPE curr_type = path.types[idx];
    const bool point_is_cusp = (path.direction_list[std::min(idx + 1, path_length - 1)] != path.direction_list[idx]);
    if (curr_type != path_type or point_is_cusp)
    {
      // create segment info
      Segment path_segment;
      path_segment.path_type = curr_type;
      path_segment.start_idx = idx;  // end_idx is not known yet, will be changed later
      segments.push_back(std::move(path_segment));

      // Change end of previous segment end
      if (segment_idx > 0)
      {
        segments.at(segment_idx - 1).end_idx = idx;
      }
      // save segment type for next segment comparison
      path_type = curr_type;
      segment_idx++;
    }
  }

  // End of path reached close last segment
  segments.back().end_idx = path_length - 1;

  // clear path to add new ones
  path.x_list.clear();
  path.y_list.clear();
  path.yaw_list.clear();
  path.delta_list.clear();
  path.direction_list.clear();
  path.types.clear();
  path.idx_analytic = -1;  // invalidated. This should not be read anymore

  // Iterate pairwise through direction changes
  for (const Segment& segment_info : segments)
  {
    Path path_segment =
        orig_path.slice(static_cast<int>(segment_info.start_idx), static_cast<int>(segment_info.end_idx));
    path_segment.cost = 0.0;
    path_segment.idx_analytic = -1;

    // Don't interpolate segment if it turns on rear axis
    if (segment_info.path_type != PATH_TYPE::REAR_AXIS)
    {
      // Do actual interpolation
      interpolatePathSegment(path_segment, segment_info, interp_res);
    }

    // if last segment
    bool inclusive_end = false;
    if (segment_info.end_idx == (path_length - 1))
    {
      inclusive_end = true;
    }
    // Extend paths by path segments
    util::extend(path.x_list, path_segment.x_list, inclusive_end);
    util::extend(path.y_list, path_segment.y_list, inclusive_end);
    util::extend(path.yaw_list, path_segment.yaw_list, inclusive_end);
    util::extend(path.delta_list, path_segment.delta_list, inclusive_end);
    util::extend(path.direction_list, path_segment.direction_list, inclusive_end);
    util::extend(path.types, path_segment.types, inclusive_end);
  }
}
