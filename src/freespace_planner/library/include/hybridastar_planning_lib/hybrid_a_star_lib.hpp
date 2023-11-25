//
// Created by schumann on 11.05.22.
//
#ifndef HYBRID_A_STAR_LIB_HPP
#define HYBRID_A_STAR_LIB_HPP

#include <cmath>
#include <utility>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>

#include "util_lib/data_structures2.hpp"
#include "util_lib/util1.hpp"
#include "util_lib/transforms.hpp"

#include "cartographing_lib/cartographing.hpp"

#include "collision_checker_lib/collision_checking.hpp"
#include "collision_checker_lib/vehicle.hpp"

#include "deps_lib/reeds_shepp.hpp"
#include "deps_lib/BSpline1D.hpp"

#include "a_star.hpp"
#include "smoother.hpp"

/**
 * Main class of the Hybrid A* Algorithm that contains all necessary functions and variables
 */
class HybridAStar
{
private:
  enum WaypointType
  {
    ASTAR_PATH,
    HEUR_RED,
    NONE
  };
  inline static constexpr double OUT_OF_HEURISTIC = 999999999;
  inline static std::string path2data_;

  inline static Point<double> patch_origin_utm_;

  inline static constexpr int NB_STEER = 13;
  inline static constexpr int NB_DIR = 2;
  inline static constexpr int NB_CONTROLS = NB_STEER * NB_DIR;

  // Parameters
  inline static double gm_res_;
  inline static double astar_res_;
  inline static double astar_yaw_res_;
  inline static int astar_yaw_res_deg_;
  inline static size_t astar_yaw_dim_;
  inline static double a_star_yaw_res_inv_;
  inline static int min_yaw_idx_;
  inline static double arc_l_;
  inline static size_t max_extra_nodes_;
  inline static std::array<double, NB_STEER> steering_inputs_;
  inline static std::array<int, 2> direction_inputs_;
  inline static double approx_goal_angle_rad_;
  inline static double approx_goal_dist2_;
  inline static double dist_thresh_analytic_;
  inline static double max_brake_acc_;
  inline static double second_rs_steer_factor_;
  inline static double extra_steer_cost_analytic_;
  inline static bool can_turn_on_point_;
  inline static double turn_on_point_horizon_;
  inline static double yaw_res_coll_;
  inline static double rear_axis_cost_;
  inline static int timeout_ms_;
  inline static double motion_res_min_;
  inline static double motion_res_max_;
  inline static double interp_res_;
  inline static double turn_on_point_angle_;
  inline static int rear_axis_freq_;
  inline static int waypoint_dist_;
  inline static WaypointType waypoint_type_;
  inline static bool is_sim_ = false;

  inline static size_t non_h_no_obs_patch_dim_;
  inline static bool non_h_no_obs_calculated_ = false;

  inline static std::set<size_t> visited_nodes_indices_;
  inline static std::unordered_set<size_t> narrow_set_;
  inline static std::unordered_set<size_t> collision_indices_;
  inline static std::unordered_set<size_t> reachable_indices_;

  // To be visited and visited set of nodes
  inline static std::unordered_map<size_t, NodeHybrid> closed_set_;
  inline static std::unordered_map<size_t, NodeHybrid> open_set_;
  inline static std::vector<NodeHybrid> neighbors_;

  // Vis states
  inline static std::pair<std::vector<double>, std::vector<double>> connected_closed_nodes_;

  // Create priority queue
  inline static PriorityQueue<size_t, double> open_queue_;

  inline static Vec3DFlat<double> non_h_no_obs_;

public:
  inline static double switch_cost_;
  inline static double steer_cost_;
  inline static double steer_change_cost_;
  inline static double h_dist_cost_;
  inline static double back_cost_;
  inline static double h_prox_cost_;

  // Lanes that can be used
  inline static LaneGraph lane_graph_;

  static void initialize(int patch_dim, const Point<double>& patch_origin_utm, const std::string& lib_share_dir);

  static void reinit(const Point<double>& patch_origin_utm, int patch_dim);

  static void setSim(bool is_sim)
  {
    is_sim_ = is_sim;
  }

  static NodeHybrid createNode(const Pose<double>& pose, double steer);

  static void recalculateEnv(const NodeHybrid& goal_node, const NodeHybrid& ego_node);

  static std::optional<Path> hybridAStarPlanning(const NodeHybrid& ego_node,
                                                 const NodeHybrid& start_node,
                                                 const NodeHybrid& goal_node,
                                                 bool to_final_pose,
                                                 bool do_analytic);

  static std::unordered_map<size_t, NodeHybrid> getClosedSet();

  static std::pair<std::vector<double>, std::vector<double>> getConnectedClosedNodes();

  static std::unordered_map<size_t, NodeHybrid> getOpenSet();

  static double getNonhnoobsVal(const NodeHybrid& start_node, const NodeHybrid& goal_node);

  static std::pair<double, double> getMaxMeanProximity(const Path& path);

  static std::pair<double, double> getMaxMeanProximityVec(const std::vector<double>& x_list,
                                                          const std::vector<double>& y_list);

  static double getDistance2GlobalGoal(const NodeHybrid& node);

  static std::tuple<Pose<double>, int, double> projEgoOnPath(const Pose<double>& pose, const Path& path, int ego_idx);

  static std::optional<Pose<double>> getValidClosePose(const Pose<double>& ego_pose, const Pose<double>& goal_pose);

  static void resetLaneGraph();

  static void updateLaneGraph(const Point<double>& origin_utm, double patch_dim);

  static void smoothPositions(std::vector<Point<double>>& positions);

  static void smoothLaneNodes(std::vector<LaneNode>& nodes);

  static void interpolateLaneNodes(std::vector<LaneNode>& nodes);

  static void smoothLaneGraph(LaneGraph& lane_graph);

  static void interpolateLaneGraph(LaneGraph& lane_graph);

private:
  static std::array<double, HybridAStar::NB_STEER> calcSteeringInputs();

  static size_t calculateIndex(size_t x_index, size_t y_index, int yaw_index);

  static double calcCost(const NodeHybrid& node,
                         const NodeHybrid& goal_node,
                         const std::unordered_map<size_t, NodeDisc>& h_dp);

  static bool anglesApproxEqual02Pi(double angle1, double angle2);

  static bool verifyIndex(int x_index, int y_index);

  static double getProxOfCorners(const Pose<double>& point);

  static double
  getPathCosts(int x_ind, int y_ind, double yaw, const NodeHybrid& node, double steer, int direction, double arc_l);

  static std::optional<NodeHybrid> calcRearAxisNode(const NodeHybrid& node, double delta_angle);

  static Pose<int> mapCont2Disc(const MotionPrimitive& motion_primitive);

  static std::optional<NodeHybrid>
  calcNextNode(const NodeHybrid& node, double steer, int direction, double motion_res, double arc_len);

  static void setNeighbors(const NodeHybrid& current, std::vector<NodeHybrid>& neighbors, double motion_res);

  static double getDistance2goal(const NodeHybrid& node, const std::unordered_map<size_t, NodeDisc>& h_dp);

  static double getTurnCost(double delta_angle);

  static double getRAPathCosts(ReedsSheppStateSpace::ReedsSheppPath path);

  static double getRSPathCosts(ReedsSheppStateSpace::ReedsSheppPath path);

  static ReedsSheppStateSpace::ReedsSheppPath
  getReedSheppPath(const Pose<double>& start, const Pose<double>& goal, double step_size, double rho);

  static std::optional<NodeHybrid> getRearAxisPath(const NodeHybrid& current_node, const NodeHybrid& goal_node);

  static Path getFinalPath(const NodeHybrid& final_node, const std::unordered_map<size_t, NodeHybrid>& closet_set);

  static bool check4Expansions(const NodeHybrid& node, const std::unordered_map<size_t, NodeDisc>& h_dp);

  static std::optional<ReedsSheppStateSpace::ReedsSheppPath> getRSExpansionPath(const NodeHybrid& current,
                                                                                const NodeHybrid& goal);

  static std::optional<NodeHybrid> getRSExpansion(const NodeHybrid& current, const NodeHybrid& goal);

  static double getDrivenAngleDiff(double angle1, double angle2, char direction);

  static void recalculateLength(ReedsSheppStateSpace::ReedsSheppPath& path);

  static void calculateNonhnoobs();

  static std::vector<double> angleArange(double angle1, double angle2, char direction, double angle_res);

  static double det(const Point<double>& vec1, const Point<double>& vec2);

  static bool lineIntersection(const Line2D<double>& line1,
                               const Line2D<double>& line2,
                               const Pose<double>& start,
                               const Pose<double>& goal,
                               double length,
                               Point<double>& intersect_point,
                               Point<double>& s2intersect);

  static NodeHybrid getFinalNodeFromPath(const NodeHybrid& current,
                                         const ReedsSheppStateSpace::ReedsSheppPath& analytic_path,
                                         PATH_TYPE path_type,
                                         double res);

  static std::optional<NodeHybrid> hAstarCore(const NodeHybrid& ego_node,
                                              const NodeHybrid& start_node,
                                              const NodeHybrid& goal_node,
                                              bool to_final_pose,
                                              bool do_analytic);

  static void interpolatePathSegment(Path& path, const Segment& segment_info, double interp_res);

  static void equal_dists_interpolation(Path& path_segment,
                                        double cum_dist,
                                        fitpack_wrapper::BSpline1D& spline_x,
                                        fitpack_wrapper::BSpline1D& spline_y,
                                        int last_dir,
                                        const PATH_TYPE type,
                                        double interp_res);

  static void exact_dist_interpolation(Path& path_segment,
                                       double cum_dist,
                                       fitpack_wrapper::BSpline1D& spline_x,
                                       fitpack_wrapper::BSpline1D& spline_y,
                                       int last_dir,
                                       PATH_TYPE type,
                                       double interp_res);

  static void interpolatePath(Path& path, double interp_res);
};

#endif  // HYBRID_A_STAR_LIB_HPP
