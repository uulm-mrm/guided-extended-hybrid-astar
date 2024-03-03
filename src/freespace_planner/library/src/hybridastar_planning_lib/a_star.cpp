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
  astar_dim_ = std::floor(static_cast<double>(patch_dim_) * GM_RES / ASTAR_RES);
  heuristic_early_exit_ = config["HEURISTIC_EARLY_EXIT"].as<bool>();
  max_extra_nodes_ = config["MAX_EXTRA_NODES_ASTAR"].as<unsigned int>();

  // voronoi potential field
  motion_res_min_ = config["MOTION_RES_MIN"].as<double>();
  motion_res_max_ = config["MOTION_RES_MAX"].as<double>();
  dist_val_min_ = config["MIN_DIST_VAL"].as<double>();
  dist_val_max_ = config["MAX_DIST_VAL"].as<double>();

  astar_grid_.setName("astar_grid_");
  movement_cost_map_.setName("movement_cost_map_");
  restr_geofence_grid_.setName("restr_geofence_grid_");

  h_prox_arr_.setName("h_prox_arr_");
  temp_h_prox_arr_.setName("temp_h_prox_arr_");
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
  astar_dim_ = std::floor(static_cast<double>(patch_dim_) * GM_RES / ASTAR_RES);

  // astar grid
  astar_grid_.resize_and_reset(astar_dim_, astar_dim_, CollisionChecker::UNKNOWN);

  h_prox_arr_.resize_and_reset(astar_dim_, astar_dim_, 0.0);
  motion_res_map_.resize_and_reset(astar_dim_, astar_dim_, motion_res_max_);
  restr_geofence_grid_.resize_and_reset(astar_dim_, astar_dim_, 0);
  movement_cost_map_.resize_and_reset(astar_dim_, astar_dim_, astar_movement_cost_);
}

void AStar::reinit(const Point<double>& patch_origin_utm, int patch_dim)
{
  // Patch dimension
  patch_dim_ = patch_dim;
  astar_dim_ = std::floor(static_cast<double>(patch_dim_) * GM_RES / ASTAR_RES);
  patch_origin_utm_ = patch_origin_utm;

  // astar grid
  astar_grid_.resize_and_reset(astar_dim_, astar_dim_, CollisionChecker::UNKNOWN);

  obstacle_dist_.resize_and_reset(VOR_DIM, VOR_DIM, CollisionChecker::FREE);
  voronoi_dist_.resize_and_reset(VOR_DIM, VOR_DIM, CollisionChecker::FREE);

  /// Save maps prior to resizing
  util::saveTemp(h_prox_arr_, temp_h_prox_arr_, 0.0);
  h_prox_arr_.resize_and_reset(astar_dim_, astar_dim_, 0.0);
  util::saveTemp(motion_res_map_, temp_motion_res_map_, motion_res_max_);
  motion_res_map_.resize_and_reset(astar_dim_, astar_dim_, motion_res_max_);

  movement_cost_map_.resize_and_reset(astar_dim_, astar_dim_, astar_movement_cost_);

  processGeofence();

  const Point<int> next_origin_astar = (patch_origin_utm * grid_tf::con2star_).toInt();

  util::copyPatch2Patch(patch_origin_astar_, next_origin_astar, h_prox_arr_, temp_h_prox_arr_);
  util::copyPatch2Patch(patch_origin_astar_, next_origin_astar, motion_res_map_, temp_motion_res_map_);

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
        const double x_diff = current.pos.x - start_node.pos.x;
        const double y_diff = current.pos.y - start_node.pos.y;
        const double dist = sqrt(x_diff * x_diff + y_diff * y_diff);

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
        const double h_euclid_dist = astar_lane_movement_cost_ * current.pos.dist2(start_node.pos) * ASTAR_RES;

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

  if (restr_geofence_grid_(y_ind, x_ind) == 1)
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
int AStar::findValidNeighborIndex(int start_idx, const std::unordered_map<size_t, NodeDisc>& heuristic)
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
          std::cout << "No node around the ego/start position was in heuristic, this should never be the case"
                    << std::endl;
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
 * Create distance transform
 */
void AStar::createDistanceTransform(const cv::Mat& input_mat, cv::Mat& dist_map, double val4dist)
{
  // distance transform
  cv::distanceTransform(1 - (val4dist == input_mat), dist_map, cv::DIST_L2, cv::DIST_MASK_5);

  // convert to cost in meters
  dist_map *= grid_tf::star2con_;
}

/**
 * Create voronoi potentialfield by creating the voronoi edges and calculating distance transforms to
 * the obstacles and voronoi edges
 * @param ego_index
 */
void AStar::calcVoronoiPotentialField(const Point<int>& ego_index)
{
  // 1. Partial array to parse
  //  const auto origin_extract = getCurrentMapOrigin(ego_index, VOR_DIM);
  const auto origin_sampling = getCurrentMapOrigin(ego_index, VOR_DIM);
  const auto opp_origin_sampling = origin_sampling + VOR_DIM;

  const cv::Mat astar_mat(astar_dim_, astar_dim_, CV_8UC1, astar_grid_.getPtr());
  const cv::Range roi_y(origin_sampling.y, opp_origin_sampling.y);
  const cv::Range roi_x(origin_sampling.x, opp_origin_sampling.x);
  const cv::Mat matSampling = astar_mat(roi_y, roi_x);

  // 2. create obstacle distance transform
  cv::Mat obs_dist_mat(VOR_DIM, VOR_DIM, CV_32FC1, obstacle_dist_.getPtr());
  createDistanceTransform(matSampling, obs_dist_mat, CollisionChecker::OCC);

  // 3. create voronoi diagram
  const cv::Mat occ_mask = (matSampling == CollisionChecker::OCC);
  std::vector<cv::Point> indices;
  cv::findNonZero(occ_mask, indices);

  // No obstacles, no vertices, no heuristic!
  if (indices.empty())
  {
    return;
  }

  // 3.1 Generate voronoi diagram
  vdg::VoronoiDiagramGenerator::generateVoronoi_wrapper(
      indices, voronoi_dist_, VOR_DIM - 1, VOR_DIM - 1, do_min_, CollisionChecker::OCC);

  // 3.2 create voronoi distance field
  cv::Mat vor_dist_mat(VOR_DIM, VOR_DIM, CV_32FC1, voronoi_dist_.getPtr());
  AStar::createDistanceTransform(vor_dist_mat, vor_dist_mat, CollisionChecker::OCC);

  for (int i = 0; i < VOR_DIM * VOR_DIM; ++i)
  {
    setCostmapValue(i, origin_sampling);
  }
}

/**
 * Calculate elements of distance maps
 * @param i
 * @param origin_sampling
 */
void AStar::setCostmapValue(int i, Point<int> origin_sampling)
{
  const auto index2D = reverse2DIndex(i, VOR_DIM);
  const int x = index2D.x;
  const int y = index2D.y;

  if (obstacle_dist_(y, x) < do_max_)
  {
    h_prox_arr_(y + origin_sampling.y, x + origin_sampling.x) =
        (alpha_ / (alpha_ + obstacle_dist_(y, x))) *
        (voronoi_dist_(y, x) / (obstacle_dist_(y, x) + voronoi_dist_(y, x))) *
        std::pow((obstacle_dist_(y, x) - do_max_), 2) / std::pow(do_max_, 2);
  }
  else
  {
    h_prox_arr_(y + origin_sampling.y, x + origin_sampling.x) = 0.0;
  }
  const double input =
      std::clamp(obstacle_dist_(y, x), static_cast<float>(dist_val_min_), static_cast<float>(dist_val_max_));
  const double motion_res = motion_res_min_ + ((motion_res_max_ - motion_res_min_) / (dist_val_max_ - dist_val_min_)) *
                                                  (input - dist_val_min_);
  motion_res_map_(y + origin_sampling.y, x + origin_sampling.x) = std::max(motion_res, motion_res_min_);
}

/**
 * Get 2d index from flattened 1d index
 * @param idx
 * @return
 */
Point<int> AStar::reverse2DIndex(int idx, int dim)
{
  const int first_index = idx % dim;
  return { first_index, (idx - first_index) / dim };
}

/**
 * Get origin of map around current position
 * @param ego_pos
 * @param dim
 * @return
 */
Point<int> AStar::getCurrentMapOrigin(const Point<int>& ego_pos, size_t dim)
{
  const int offset = std::floor(dim / 2);
  return { std::max(ego_pos.x - offset, 0), std::max(ego_pos.y - offset, 0) };
}

/**
 * Pool map with cudnn pooling filters
 */
void AStar::calcAstarGridCuda()
{
  Pooling::execute(CollisionChecker::patch_safety_arr_.getPtr(),
                   astar_grid_.getPtr(),
                   static_cast<int>(patch_dim_),
                   static_cast<int>(astar_dim_));
}

void AStar::resetMovementMap()
{
  movement_cost_map_.resize_and_reset(astar_dim_, astar_dim_, astar_movement_cost_);
}

/**
 * Set pixel around lane graph to reduced movement costs
 * @param edges
 */
void AStar::setMovementMap(const LaneGraph::edges_t& edges)
{
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
        for (const auto& p_diff : patch_coords_)
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
        for (const auto& p_diff : patch_coords_)
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

/**
 *
 */
void AStar::processGeofence()
{
  AStar::restr_geofence_grid_.resize_and_reset(AStar::astar_dim_, AStar::astar_dim_, 0);

  // Get min and max coordinates
  int min_x = AStar::astar_dim_;
  int min_y = AStar::astar_dim_;
  int max_x = 0;
  int max_y = 0;
  for (auto& point : AStar::restr_geofence_.vertices)
  {
    const Point<double> point_patch = point - patch_origin_utm_;
    const Point<int> point_astar = (point_patch * grid_tf::con2star_).toInt();
    if (point_astar.x < min_x)
    {
      min_x = point_astar.x;
    }
    if (point_astar.y < min_y)
    {
      min_y = point_astar.y;
    }
    if (point_astar.x > max_x)
    {
      max_x = point_astar.x;
    }
    if (point_astar.y > max_y)
    {
      max_y = point_astar.y;
    }
  }

  // iterate through min max
  for (int x_idx = min_x; x_idx < max_x; ++x_idx)
  {
    for (int y_idx = min_x; y_idx < max_y; ++y_idx)
    {
      const Point<int> point_astar(x_idx, y_idx);
      const Point<double> point_utm = point_astar.toDouble() * grid_tf::star2con_ + patch_origin_utm_;

      if (util::point_in_poly(restr_geofence_, point_utm))
      {
        AStar::restr_geofence_grid_(point_astar) = 1;
      }
    }
  }
}
