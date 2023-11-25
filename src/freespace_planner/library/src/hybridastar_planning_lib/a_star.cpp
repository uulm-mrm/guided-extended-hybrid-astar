//
// Created by Schumann on 23.06.22.
//
#include "hybridastar_planning_lib/a_star.hpp"

/**
 * Must be called whenever the path dim changes, initializes all data structures
 * @param patch_dim
 * @param path2config
 */
void AStar::initialize(int patch_dim, const Point<double>& patch_origin_utm, const std::string& path2config)
{
  // Load config
  path2config_ = path2config;
  YAML::Node config = YAML::LoadFile(path2config_);
  //  auto log_level = config["LOG_LEVEL_ASTAR"].as<std::string>();

  //  std::string stream_name = "freespace_planner->AStar";
  //  astar::_initLogger();
  //  astar::_setStreamName(stream_name);
  //  astar::_setLogLevel(log_level);
  //  astar::_setShowOrigin(true);
  //
  //  LOG_DEB("Initialize AStar");

  // Patch dimension
  patch_origin_astar_ = (patch_origin_utm * grid_tf::con2star_).toInt();
  patch_dim_ = patch_dim;
  unknown_cost_w_ = config["ASTAR_UNKNOWN_COST"].as<double>();
  gm_res_ = config["GM_RES"].as<double>();
  astar_res_ = config["PLANNER_RES"].as<double>();
  astar_dim_ = std::floor(static_cast<double>(patch_dim_) * gm_res_ / astar_res_);
  heuristic_early_exit_ = config["HEURISTIC_EARLY_EXIT"].as<bool>();
  max_extra_nodes_ = config["MAX_EXTRA_NODES_ASTAR"].as<unsigned int>();

  // voronoi potential field
  motion_res_min_ = config["MOTION_RES_MIN"].as<double>();
  motion_res_max_ = config["MOTION_RES_MAX"].as<double>();
  dist_val_min_ = config["MIN_DIST_VAL"].as<double>();
  dist_val_max_ = config["MAX_DIST_VAL"].as<double>();

  movement_distances_ = getMovementDists();

  for (size_t y_ind = 0; y_ind < VOR_DIM; ++y_ind)
  {
    for (size_t x_ind = 0; x_ind < VOR_DIM; ++x_ind)
    {
      vor_coords_[y_ind * VOR_DIM + x_ind][0] = static_cast<double>(x_ind);
      vor_coords_[y_ind * VOR_DIM + x_ind][1] = static_cast<double>(y_ind);
    }
  }

  for (size_t y_ind = 0; y_ind < VOR_DIM_SAMPLING; ++y_ind)
  {
    for (size_t x_ind = 0; x_ind < VOR_DIM_SAMPLING; ++x_ind)
    {
      vor_coords_sampling_[y_ind * VOR_DIM_SAMPLING + x_ind][0] = static_cast<double>(x_ind);
      vor_coords_sampling_[y_ind * VOR_DIM_SAMPLING + x_ind][1] = static_cast<double>(y_ind);
    }
  }

  astar_grid_.setName("astar_grid_");
  movement_cost_map_.setName("movement_cost_map_");

  h_prox_arr_.setName("h_prox_arr_");
  temp_h_prox_arr_.setName("temp_h_prox_arr_");
  obs_x_grad_.setName("obs_x_grad_");
  temp_obs_x_grad_.setName("obs_x_grad_");
  obs_y_grad_.setName("temp_obs_x_grad_");
  temp_obs_y_grad_.setName("temp_obs_y_grad_");
  motion_res_map_.setName("motion_res_map_");
  temp_motion_res_map_.setName("temp_motion_res_map_");

  init_structs(patch_dim);

  const int pool_dim = std::ceil(grid_tf::star2gm_);
  Pooling::init(pool_dim);
}

void AStar::init_structs(int patch_dim)
{
  // Patch dimension
  patch_dim_ = patch_dim;
  astar_dim_ = std::floor(static_cast<double>(patch_dim_) * gm_res_ / astar_res_);

  // astar grid
  astar_grid_.resize_and_reset(astar_dim_, astar_dim_, CollisionChecker::UNKNOWN);

  // Voronoi proximity heuristic
  h_prox_arr_.resize_and_reset(astar_dim_, astar_dim_, 0.0);
  // distance fields with gradients
  obs_x_grad_.resize_and_reset(astar_dim_, astar_dim_, 0.0);
  obs_y_grad_.resize_and_reset(astar_dim_, astar_dim_, 0.0);
  // motion res map
  motion_res_map_.resize_and_reset(astar_dim_, astar_dim_, motion_res_max_);

  resetMovementMap();
}

void AStar::reinit(const Point<double>& patch_origin_utm, int patch_dim)
{
  // Patch dimension
  patch_dim_ = patch_dim;
  astar_dim_ = std::floor(static_cast<double>(patch_dim_) * gm_res_ / astar_res_);
  patch_origin_utm_ = patch_origin_utm;

  // astar grid
  astar_grid_.resize_and_reset(astar_dim_, astar_dim_, CollisionChecker::UNKNOWN);

  /// Save maps prior to resizing
  // Voronoi proximity heuristic
  util::saveTemp(h_prox_arr_, temp_h_prox_arr_, 0.0);
  h_prox_arr_.resize_and_reset(astar_dim_, astar_dim_, 0.0);
  // distance fields with gradients
  util::saveTemp(obs_x_grad_, temp_obs_x_grad_, 0.0);
  obs_x_grad_.resize_and_reset(astar_dim_, astar_dim_, 0.0);
  util::saveTemp(obs_y_grad_, temp_obs_y_grad_, 0.0);
  obs_y_grad_.resize_and_reset(astar_dim_, astar_dim_, 0.0);
  // motion res map
  util::saveTemp(motion_res_map_, temp_motion_res_map_, motion_res_max_);
  motion_res_map_.resize_and_reset(astar_dim_, astar_dim_, motion_res_max_);

  resetMovementMap();

  const Point<int> next_origin_astar = (patch_origin_utm * grid_tf::con2star_).toInt();

  util::copyPatch2Patch(patch_origin_astar_, next_origin_astar, h_prox_arr_, temp_h_prox_arr_);
  util::copyPatch2Patch(patch_origin_astar_, next_origin_astar, motion_res_map_, temp_motion_res_map_);
  util::copyPatch2Patch(patch_origin_astar_, next_origin_astar, obs_x_grad_, temp_obs_x_grad_);
  util::copyPatch2Patch(patch_origin_astar_, next_origin_astar, obs_y_grad_, temp_obs_y_grad_);

  patch_origin_astar_ = next_origin_astar;
}

/**
 * calculate the distance heuristic. It can be chosen if the heuristic should be calculated to the next local goal or
 * the final global goal [for_path or not]
 * @param g_x
 * @param g_y
 * @param s_x
 * @param s_y
 * @param for_path
 */
void AStar::calcDistanceHeuristic(const Point<int>& goal_pos,
                                  const Point<int>& start_pos,
                                  bool for_path,
                                  bool get_only_near)
{
  // Check if start is unknown
  bool goal_unknown = (astar_grid_(goal_pos) == CollisionChecker::UNKNOWN);

  // create node object of distance heuristics, only astar grid coords (indices)
  const NodeDisc goal_node = NodeDisc(goal_pos, 0.0, 0.0, -1, goal_unknown);

  // for early exit
  const NodeDisc start_node = NodeDisc(start_pos, 0.0, 0.0, -1, false);

  const size_t start_id = calcIndex(start_node);

  // To be visited and visited set of nodes
  std::unordered_map<size_t, NodeDisc> open_set;  // maps indices and nodes
  std::unordered_map<size_t, NodeDisc>* closed_set;

  // Chooses the set of nodes, one for the guidance and one for the path
  if (for_path)
  {
    closed_set = &closed_set_path_;
  }
  else
  {
    closed_set = &closed_set_guidance_;
  }

  closed_set->clear();
  nodes_near_goal_.clear();

  // Add goal node to open set
  const size_t goal_id = AStar::calcIndex(goal_node);
  open_set.insert({ goal_id, goal_node });

  // Create priority queue
  PriorityQueue<size_t, double> frontier;
  // Add goal node
  frontier.put(goal_id, 0);

  bool start_found = (start_id == goal_id);
  unsigned int nr_extra_nodes = 0;

  // Expansion by dynamic programming
  bool is_near = false;
  while (true)
  {
    if (closed_set->size() > static_cast<size_t>(round((0.95 * (astar_dim_ * astar_dim_)))))
    {
      //      LOG_WARN("Distance heuristic seems to explore too many nodes! "
      //               "This should never happen! The start should be found by the heuristic");
      break;
    }
    if (frontier.empty())
    {
      break;
    }

    // get next element
    const size_t c_id = frontier.get();

    // if in open set, remove it and put in closed set
    const auto search_curr = open_set.find(c_id);
    if (search_curr != open_set.end())
    {
      const NodeDisc current = search_curr->second;
      open_set.erase(c_id);
      closed_set->insert({ c_id, current });

      // goal is to get only near the goal
      if (get_only_near)
      {
        double x_diff = current.pos.x - start_node.pos.x;
        double y_diff = current.pos.y - start_node.pos.y;
        double dist = sqrt(x_diff * x_diff + y_diff * y_diff);

        if (dist < 10)
        {
          nodes_near_goal_.emplace_back(dist, current);
          is_near = true;
        }
      }

      // If node is found, early exit after all open nodes were processed
      if (heuristic_early_exit_)
      {
        int index_diff = std::abs(static_cast<int>(c_id) - static_cast<int>(start_id));
        if (index_diff == 0)
        {
          start_found = true;
        }
        else if (index_diff == 1 || index_diff == astar_dim_)
        {
          is_near = true;
        }
      }

      // Allow the lasting open nodes to be explored plus extra nodes, then quit
      if (heuristic_early_exit_ or get_only_near)
      {
        if (start_found || is_near)
        {
          if (nr_extra_nodes > max_extra_nodes_)
          {
            continue;
          }
          nr_extra_nodes++;
        }
      }

      // expand search grid based on motion model
      for (int i = 0; i < NB_GRID_MOTIONS; ++i)
      {
        // Calculate next index
        const Point<int> position = current.pos + motion_[i];

        // If it is not valid --> Ignore
        if (!verifyNode(position.x, position.y))
        {
          continue;
        }

        // get index of created node to identify it on grid with one integer
        const size_t n_id = calcIndex(position.x, position.y);

        // If it was already in closed set --> Ignore
        if (closed_set->contains(n_id))
        {
          continue;
        }

        // new costs caused by movement itself
        const double movement_weigth = movement_cost_map_(position);
        const double movement_costs = movement_distances_[i] * movement_weigth;

        // Track current distance for next heuristic
        const double cost_dist = current.cost_dist_ + movement_distances_[i];

        // new costs caused by proximity to objects
        const double prox_cost = h_prox_arr_(current.pos) * astar_prox_cost_;
        // costs caused by unknown area
        bool is_unknown = (astar_grid_(current.pos) == CollisionChecker::UNKNOWN);
        const double unknown_cost = is_unknown ? unknown_cost_w_ : 0;

        // New costs up to this node
        const double current_cost = current.cost_ + movement_costs + prox_cost + unknown_cost;

        // Euclidean heuristic for guidance with lane cost as lowest estimate
        const double h_euclid_dist = astar_lane_movement_cost_ * current.pos.dist2(start_node.pos) * astar_res_;

        // Combine heuristic costs and costs until here
        const double estimated_costs = h_euclid_dist + current_cost;

        const NodeDisc node = NodeDisc(position, current_cost, cost_dist, static_cast<int>(c_id), is_unknown);

        // Node is valid, continue the search on the neighbors
        const auto search_next = open_set.find(n_id);
        // is not in open set
        if (search_next == open_set.end())
        {
          open_set.insert({ n_id, node });
          frontier.put(n_id, estimated_costs);

          // Node was already in open set, compare costs and replace it if new costs are lower
        }
        else
        {
          const NodeDisc found_node = search_next->second;
          if (found_node.cost_ > node.cost_)
          {
            open_set.erase(n_id);
            open_set.emplace(n_id, node);
            frontier.put(n_id, estimated_costs);
          }
        }
      }
      // If not in open set, ignore it, can only happen because of updating nodes with lower costs
    }
    else
    {
      continue;
    }
  }
}

/**
 * Movement costs == travelled distance
 * @return
 */
std::array<double, AStar::NB_GRID_MOTIONS> AStar::getMovementDists()
{
  std::array<double, NB_GRID_MOTIONS> movement_dists{ 1, 1, 1, 1, sqrt(2), sqrt(2), sqrt(2), sqrt(2) };

  for (auto& dist : movement_dists)
  {
    dist *= astar_res_;
  }
  return movement_dists;
}

/**
 * Calculate flattened index
 * @param x_ind
 * @param y_ind
 * @return
 */
size_t AStar::calcIndex(size_t x_ind, size_t y_ind)
{
  return y_ind * astar_dim_ + x_ind;
}

/**
 * Overloaded function of the calcIndex above
 * @param node
 * @return
 */
size_t AStar::calcIndex(const NodeDisc& node)
{
  return calcIndex(node.pos.x, node.pos.y);
}

/**
 * Return True if node if valid, otherwise return false
 * @param x_ind
 * @param y_ind
 * @param max_idx
 * @return
 */
bool AStar::verifyNode(int x_ind, int y_ind)
{
  // node is out of bounds or occupied
  if ((x_ind < 0 || y_ind < 0) || (x_ind >= astar_dim_ || y_ind >= astar_dim_))
  {
    return false;
  }
  // node collides
  if (astar_grid_(y_ind, x_ind) == CollisionChecker::OCC)
  {
    return false;
  }

  return true;
}

/**
 * Check if a neighboring grid cell is valid and if yes return its index
 * @param start_idx
 * @param heuristic
 * @return
 */
int AStar::findValidNeighborIndex(int start_idx, std::unordered_map<size_t, NodeDisc> heuristic)
{
  int backup_idx = start_idx + 1;
  //  LOG_INF("Checking node to the right");
  if (heuristic.find(backup_idx) == heuristic.end())
  {
    backup_idx = start_idx - 1;
    //    LOG_INF("Checking node to the left");
    if (heuristic.find(backup_idx) == heuristic.end())
    {
      backup_idx = start_idx + astar_dim_;
      //      LOG_INF("Checking node above");
      if (heuristic.find(backup_idx) == heuristic.end())
      {
        backup_idx = start_idx - astar_dim_;
        //        LOG_INF("Checking node below");
        if (heuristic.find(backup_idx) == heuristic.end())
        {
          //          LOG_ERR("No node around the ego/start position was in heuristic, this should never be the case");
          backup_idx = -1;
        }
      }
    }
  }
  // Insert backup index as start
  return backup_idx;
}

/**
 * get astar path from previously calculated heuristic
 * @param x_index
 * @param y_index
 * @return
 */
std::pair<std::vector<int>, std::vector<int>> AStar::getAstarPath(int x_index, int y_index)
{
  std::vector<int> x_list;
  std::vector<int> y_list;

  size_t start_idx = calcIndex(x_index, y_index);

  // find node in closed set
  if (closed_set_guidance_.find(start_idx) == closed_set_guidance_.end())
  {
    //    LOG_WARN("Start was not in heuristic! Checking for neighbor!");
    int index = findValidNeighborIndex(static_cast<int>(start_idx), closed_set_guidance_);
    if (index == -1)
    {
      return { x_list, y_list };
    }
    start_idx = static_cast<size_t>(index);
  }

  // Add start node node
  const NodeDisc node = closed_set_guidance_.at(start_idx);
  x_list.push_back(node.pos.x);
  y_list.push_back(node.pos.y);

  // Iterate through all nodes
  int parent_idx = node.parent_index_;
  while (parent_idx != -1)
  {
    const NodeDisc next_node = closed_set_guidance_.at(parent_idx);
    x_list.push_back(next_node.pos.x);
    y_list.push_back(next_node.pos.y);
    parent_idx = next_node.parent_index_;
  }

  return { x_list, y_list };
}

/**
 * * calculate the voronoi potential field around the ego vehicle
 * This consists of getting the obstacles from the astar grid, calculated a voronoi diagram,
 * followed by final calculations
 * @param ego_index
 */
void AStar::calcVoronoiPotentialField(const Point<int>& ego_index)
{
  const auto origin_extract = getCurrentMapOrigin(ego_index, VOR_DIM);
  const auto origin_sampling = getCurrentMapOrigin(ego_index, VOR_DIM_SAMPLING);
  const auto opp_origin_sampling = origin_sampling + VOR_DIM_SAMPLING;

  // Setup data of voronoi. Reserve with size of previous turn
  vor_samples_.clear();
  vor_samples_.reserve(vor_size_);

  // find all occupied pixels
  const cv::Mat astar_mat(astar_dim_, astar_dim_, CV_8UC1, astar_grid_.getPtr());
  cv::Mat matSampling = astar_mat(cv::Range(origin_sampling.y, opp_origin_sampling.y),
                                  cv::Range(origin_sampling.x, opp_origin_sampling.x));

  const cv::Mat occ_mask = (matSampling == CollisionChecker::OCC);
  std::vector<cv::Point> indices;
  cv::findNonZero(occ_mask, indices);

  // extract points
  obs_samples_.clear();
  obs_samples_.resize(indices.size());
  int idx = 0;
  for (const auto& point : indices)
  {
    obs_samples_[idx] = { static_cast<double>(point.x + origin_sampling.x),
                          static_cast<double>(point.y + origin_sampling.y) };
    idx++;
  }

  //  cv::Mat dist;
  //  cv::Mat bin;
  //  const cv::Mat occ_bin = (matSampling == CollisionChecker::OCC);
  //  distanceTransform(occ_bin, dist, cv::DIST_L2, 3);
  //  normalize(dist, dist, 0, 1.0, cv::NORM_MINMAX);
  //
  //  cv::namedWindow("distance transform w", cv::WINDOW_NORMAL);
  //  cv::imshow("distance transform w", dist);
  //  cv::waitKey(0);
  //
  //  std::memcpy(h_prox_arr_.getPtr(), dist.data, astar_dim_ * astar_dim_ * sizeof(uint8_t));

  //  std::for_each(
  //    std::execution::unseq,
  //    vor_coords_.begin(),
  //    vor_coords_.end(),
  //    [&origin_extract, &dist](const auto& point) {
  //        int x_ins = point[0] + origin_extract.x;
  //        int y_ins = point[1] + origin_extract.y;
  //        h_prox_arr_(y_ins, x_ins) = dist.at<uint8_t>(static_cast<int>(point[0]), static_cast<int>(point[1]));
  //});
  //  h_prox_arr_.vec = dist.data;

  obs_size_ = obs_samples_.size();
  obs_samples_.shrink_to_fit();

  // No obstacles, no vertices, no heuristic!
  if (obs_size_ == 0)
  {
    return;
  }

  // Generate voronoi diagram
  vdg::VoronoiDiagramGenerator::generateVoronoi_wrapper(obs_samples_,
                                                        vor_samples_,
                                                        origin_sampling.x,
                                                        static_cast<double>(origin_sampling.x + VOR_DIM_SAMPLING),
                                                        origin_sampling.y,
                                                        static_cast<double>(origin_sampling.y + VOR_DIM_SAMPLING),
                                                        do_min_);
  vor_size_ = vor_samples_.size();
  vor_samples_.shrink_to_fit();

  // construct a kd-tree index:
  const my_kd_tree_t obs_mat_index(MAP_DIM, obs_samples_, 10);
  const my_kd_tree_t vor_mat_index(MAP_DIM, vor_samples_, 10);

  std::for_each(
      std::execution::unseq,
      vor_coords_.begin(),
      vor_coords_.end(),
      [&origin_extract, &obs_mat_index, &vor_mat_index](const auto& point) {
        calcVorFieldElement({ point[0] + origin_extract.x, point[1] + origin_extract.y }, obs_mat_index, vor_mat_index);
      });
}

/**
 * Calculation of the elements of the proximity heuristic and motion res map
 * @param query_pt
 * @param obs_mat_index
 * @param vor_mat_index
 */
void AStar::calcVorFieldElement(const std::array<double, 2>& query_vec,
                                const my_kd_tree_t& obs_mat_index,
                                const my_kd_tree_t& vor_mat_index)
{
  std::vector<size_t> ret_indexes_obs(NUM_RESULTS);
  std::vector<double> out_dists_sqr_obs(NUM_RESULTS);
  nanoflann::KNNResultSet<double> resultSet_obs(NUM_RESULTS);

  std::vector<size_t> ret_indexes_vor(NUM_RESULTS);
  std::vector<double> out_dists_sqr_vor(NUM_RESULTS);
  nanoflann::KNNResultSet<double> resultSet_vor(NUM_RESULTS);

  resultSet_obs.init(ret_indexes_obs.data(), out_dists_sqr_obs.data());
  obs_mat_index.index->findNeighbors(resultSet_obs, query_vec.data());
  const double d_o = sqrt(out_dists_sqr_obs[0]);

  resultSet_vor.init(ret_indexes_vor.data(), out_dists_sqr_vor.data());
  vor_mat_index.index->findNeighbors(resultSet_vor, query_vec.data());
  const double d_v = sqrt(out_dists_sqr_vor[0]);

  const int x_ind = static_cast<int>(query_vec[0]);
  const int y_ind = static_cast<int>(query_vec[1]);

  // calculate the voronoi potential fields
  double val = 0;
  double dist_x = 0;
  double dist_y = 0;
  if (d_o < do_max_)
  {
    // get distance
    val = (alpha_ / (alpha_ + d_o)) * (d_v / (d_o + d_v)) * std::pow((d_o - do_max_), 2) / std::pow(do_max_, 2);

    if (d_o != 0)
    {
      // get distance components to next obstacle
      dist_x = query_vec[0] - obs_mat_index.kdtree_get_pt(ret_indexes_obs[0], 0);
      dist_y = query_vec[1] - obs_mat_index.kdtree_get_pt(ret_indexes_obs[0], 1);

      // Invert and normalize that the gradient is longer on close objects
      dist_x = util::sgn(dist_x) * (do_max_ - util::sgn(dist_x) * dist_x) / do_max_;
      dist_y = util::sgn(dist_y) * (do_max_ - util::sgn(dist_y) * dist_y) / do_max_;
    }
  }

  // threshold small values to 0 due to discretization of array
  if (val < 0.08)
  {
    val = 0;
  }

  // set proximity array and gradients
  h_prox_arr_(y_ind, x_ind) = val;
  obs_x_grad_(y_ind, x_ind) = dist_x;
  obs_y_grad_(y_ind, x_ind) = dist_y;

  const double input = std::clamp(d_o, dist_val_min_, dist_val_max_);
  const double motion_res = motion_res_min_ + ((motion_res_max_ - motion_res_min_) / (dist_val_max_ - dist_val_min_)) *
                                                  (input - dist_val_min_);
  motion_res_map_(y_ind, x_ind) = std::max(motion_res, motion_res_min_);
}

/**
 * Get 2d index from flattened 1d index
 * @param idx
 * @return
 */
std::pair<size_t, size_t> AStar::reverse2DIndex(size_t idx)
{
  size_t first_index = idx % astar_dim_;
  return { first_index, (idx - first_index) / astar_dim_ };
}

/**
 * return previously calculated proximity heuristic array
 * @return
 */
py::array_t<double> AStar::getObsGradX()
{
  return py::array_t<double>({ astar_dim_, astar_dim_ },  // shape
                             obs_x_grad_.data().data()    // the data pointer
  );
}

/**
 * return previously calculated proximity heuristic array
 * @return
 */
py::array_t<double> AStar::getObsGradY()
{
  return py::array_t<double>({ astar_dim_, astar_dim_ },  // shape
                             obs_y_grad_.data().data()    // the data pointer
  );
}

Point<int> AStar::getCurrentMapOrigin(const Point<int>& ego_pos, size_t dim)
{
  const int offset = std::floor(dim / 2);
  return { std::max(ego_pos.x - offset, 0), std::max(ego_pos.y - offset, 0) };
}

/**
 * return previously calculated heuristic
 * @param for_path
 * @return
 */
std::unordered_map<size_t, NodeDisc> AStar::getDistanceHeuristic(bool for_path)
{
  if (for_path)
  {
    return closed_set_path_;
  }
  return closed_set_guidance_;
}

void AStar::calcAstarGridCuda()
{
  Pooling::execute(CollisionChecker::patch_safety_arr_.getPtr(),
                   astar_grid_.getPtr(),
                   static_cast<int>(patch_dim_),
                   static_cast<int>(astar_dim_));
}

///**
// * Do a max pooling on the patch to receive the astar grid
// */
// void AStar::calcAstarGrid()
//{
//  const size_t patch_dim = CollisionChecker::getPatchDim();
//  const size_t pool_dim = std::ceil(grid_tf::star2gm);
//
//  size_t idx_y_patch;
//  size_t idx_x_patch;
//  size_t x_gm_idx;
//  size_t y_gm_idx;
//  dtype maximum;
//  dtype value;
//
//  // max pooling
//  for (size_t y_ind = 0; y_ind < astar_dim_; ++y_ind)
//  {
//    for (size_t x_ind = 0; x_ind < astar_dim_; ++x_ind)
//    {
//      // Start with minimal value == 0 = FREE
//      maximum = CollisionChecker::FREE;
//
//      // Calculate index in grid map
//      x_gm_idx = static_cast<size_t>(round(static_cast<double>(x_ind) * grid_tf::star2gm));
//      y_gm_idx = static_cast<size_t>(round(static_cast<double>(y_ind) * grid_tf::star2gm));
//
//      // begin pooling
//      for (size_t y_shift = 0; y_shift < pool_dim; ++y_shift)
//      {
//        // Set and verify y grid index on patch_info
//        idx_y_patch = y_gm_idx + y_shift;
//        if (idx_y_patch > patch_dim - 1)
//        {
//          continue;
//        }
//
//        for (size_t x_shift = 0; x_shift < pool_dim; ++x_shift)
//        {
//          // Set and verify x grid index on patch_info
//          idx_x_patch = x_gm_idx + x_shift;
//          if ((idx_x_patch > patch_dim - 1))
//          {
//            continue;
//          }
//
//          // get value of collision array for the check on occupied
//          value = CollisionChecker::getPatchValue(idx_x_patch, idx_y_patch);
//
//          // do the actual maximum comparison
//          if (value > maximum)
//          {
//            maximum = value;
//          }
//        }
//      }  // end of pooling
//      astar_grid_(y_ind, x_ind) = maximum;
//    }
//  }
//}

void AStar::resetMovementMap()
{
  movement_cost_map_.resize_and_reset(astar_dim_, astar_dim_, astar_movement_cost_);
}

void AStar::setMovementMap(const LaneGraph::edges_t& edges)
{
  const std::vector<Point<int>> p_diffs = { { 0, 0 },  { 0, 1 },  { 0, -1 }, { 1, 0 },  { 1, 1 },
                                            { 1, -1 }, { -1, 0 }, { -1, 1 }, { -1, -1 } };

  for (const auto& edge : edges)
  {
    const auto& [n1_opt, node_center, n3_opt] = edge;

    const Point<int> p_center = (node_center.point_utm_patch * grid_tf::con2star_).toInt();

    if (n1_opt)
    {
      auto n_1 = *n1_opt;
      const Point<int> p_1 = (n_1.point_utm_patch * grid_tf::con2star_).toInt();
      const std::vector<Point<int>> line1 = util::drawline(p_center, p_1);

      for (const auto& point : line1)
      {
        for (const auto& p_diff : p_diffs)
        {
          const Point<int> insert_point = point + p_diff;
          if (insert_point.x < 0 or insert_point.x >= astar_dim_ or insert_point.y < 0 or insert_point.y >= astar_dim_)
          {
            continue;
          }
          AStar::movement_cost_map_(insert_point) = AStar::astar_lane_movement_cost_;
        }
      }
    }

    if (n3_opt)
    {
      auto n_3 = *n3_opt;
      const Point<int> p_3 = (n_3.point_utm_patch * grid_tf::con2star_).toInt();
      const std::vector<Point<int>> line2 = util::drawline(p_center, p_3);

      for (const auto& point : line2)
      {
        for (const auto& p_diff : p_diffs)
        {
          const Point<int> insert_point = point + p_diff;
          if (insert_point.x < 0 or insert_point.x >= astar_dim_ or insert_point.y < 0 or insert_point.y >= astar_dim_)
          {
            continue;
          }

          AStar::movement_cost_map_(point + p_diff) = AStar::astar_lane_movement_cost_;
        }
      }
    }
  }
}
