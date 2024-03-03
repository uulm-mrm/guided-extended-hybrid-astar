//
// Created by schumann on 11.05.22.
//
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

#include "hybridastar_planning_lib/hybrid_a_star_lib.hpp"

namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(Path)
PYBIND11_MAKE_OPAQUE(NodeHybrid)
PYBIND11_MAKE_OPAQUE(NodeDisc)
PYBIND11_MAKE_OPAQUE(std::unordered_map<size_t, NodeHybrid>)
PYBIND11_MAKE_OPAQUE(std::unordered_map<size_t, NodeDisc>)

// The module name (example) is given as the first macro argument (it should not be in quotes).
// The second argument (m) defines a variable of type py::module_ which is the main interface for creating bindings
// The method module_::def() generates binding code that exposes the add() function to Python.
PYBIND11_MODULE(_hybridastar_planning_lib_api, m)
{
  py::class_<Vec3DFlat<int>>(m, "Vec3dFlatInt")
      .def("getVal", &Vec3DFlat<int>::getVal, "returns value of vector")
      .def("getDims", &Vec3DFlat<int>::getDims, "returns dims of vector");

  py::class_<Vec2DFlat<int>>(m, "Vec2DFlatInt")
      .def("getDims", &Vec2DFlat<int>::getDims, "returns dims of vector")
      .def("getNumpyArr", &Vec2DFlat<int>::getNumpyArr, "getNumpyArr");

  py::class_<Vec2DFlat<double>>(m, "Vec2DFlatDouble")
      .def("getDims", &Vec2DFlat<double>::getDims, "returns dims of vector")
      .def("getNumpyArr", &Vec2DFlat<double>::getNumpyArr, "getNumpyArr");

  py::class_<Vec2DFlat<float>>(m, "Vec2DFlatFloat")
      .def("getDims", &Vec2DFlat<float>::getDims, "returns dims of vector")
      .def("getNumpyArr", &Vec2DFlat<float>::getNumpyArr, "getNumpyArr");

  py::class_<Vec2DFlat<uint8_t>>(m, "Vec2DFlatUint8")
      .def("getDims", &Vec2DFlat<uint8_t>::getDims, "returns dims of vector")
      .def("getNumpyArr", &Vec2DFlat<uint8_t>::getNumpyArr, "getNumpyArr");

  py::class_<Vec2DFlat<int8_t>>(m, "Vec2DFlatInt8")
      .def("getDims", &Vec2DFlat<int8_t>::getDims, "returns dims of vector")
      .def("getNumpyArr", &Vec2DFlat<int8_t>::getNumpyArr, "getNumpyArr");

  py::class_<Pose<double>>(m, "PoseDouble")
      .def(py::init<double, double, double>())
      .def_readwrite("x", &Pose<double>::x)
      .def_readwrite("y", &Pose<double>::y)
      .def_readwrite("yaw", &Pose<double>::yaw)
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(-py::self)
      .def(float() * py::self)
      .def(py::self * float())
      .def(py::self / float())
      .def(py::self + float())
      .def(py::self - float())
      .def("__round__", &Pose<double>::round)
      .def("copy", &Pose<double>::copy)
      .def("equal", &Pose<double>::equal)
      .def("getPoint", &Pose<double>::getPoint);

  py::class_<Pose<int>>(m, "PoseInt")
      .def(py::init<int, int, int>())
      .def_readwrite("x", &Pose<int>::x)
      .def_readwrite("y", &Pose<int>::y)
      .def_readwrite("yaw", &Pose<int>::yaw)
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(-py::self)
      .def(float() * py::self)
      .def(py::self * float())
      .def(py::self / float())
      .def(py::self + float())
      .def(py::self - float())
      .def("copy", &Pose<int>::copy)
      .def("equal", &Pose<int>::equal)
      .def("getPoint", &Pose<int>::getPoint);

  py::class_<Point<double>>(m, "PointDouble")
      .def(py::init<double, double>())
      .def_readwrite("x", &Point<double>::x)
      .def_readwrite("y", &Point<double>::y)
      .def("__str__", &Point<double>::toString)
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(-py::self)
      .def(float() * py::self)
      .def(py::self * float())
      .def(py::self / float())
      .def(py::self + float())
      .def(py::self - float())
      .def("__round__", &Point<double>::round)
      .def("copy", &Point<double>::copy)
      .def("equal", &Point<double>::equal)
      .def("toInt", &Point<double>::toInt)
      .def("toDouble", &Point<double>::toDouble)
      .def("dist2", &Point<double>::dist2);

  py::class_<Point<int>>(m, "PointInt")
      .def(py::init<int, int>())
      .def_readwrite("x", &Point<int>::x)
      .def_readwrite("y", &Point<int>::y)
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(-py::self)
      //      .def(float() * py::self)
      .def(py::self * float())
      .def(py::self / float())
      .def(py::self + float())
      .def(py::self - float())
      .def("copy", &Point<int>::copy)
      .def("equal", &Point<int>::equal);

  py::class_<LaneGraph>(m, "LaneGraph")
      .def(py::init<Point<double>, double>())
      .def("addPoint", &LaneGraph::addPoint, "addPoint")
      .def_readwrite("edges_", &LaneGraph::edges_, "edges_")
      .def_readwrite("nodes_", &LaneGraph::nodes_, "nodes_");

  py::class_<LaneNode>(m, "LaneNode")
      .def(py::init<Point<double>>())
      .def_readwrite("point_utm", &LaneNode::point_utm, "point_utm")
      .def_readwrite("point_utm_patch", &LaneNode::point_utm_patch, "point_utm_patch")
      .def_readwrite("index", &LaneNode::index, "index")
      .def_readwrite("neighbor_1", &LaneNode::neighbor_1, "neighbor_1")
      .def_readwrite("neighbor_2", &LaneNode::neighbor_2, "neighbor_2");

  py::class_<Polygon>(m, "Polygon")
      .def_readwrite("vertices", &Polygon::vertices, "vertices")
      .def("addPoint", &Polygon::addPoint, "addPoint")
      .def("reset", &Polygon::reset, "reset");

  py::class_<Minipatch>(m, "Minipatch")
      .def(py::init<Vec2DFlat<uint8_t>,
                    Point<double>,
                    Point<double>,
                    int,
                    int,
                    Point<int>,
                    double,
                    Point<double>,
                    bool>())
      .def(py::init<const py::array_t<uint8_t>&,
                    Point<double>,
                    Point<double>,
                    int,
                    int,
                    Point<int>,
                    double,
                    Point<double>,
                    bool>())
      .def_readonly("center_", &Minipatch::center_)
      .def_readonly("origin_", &Minipatch::origin_)
      .def_readonly("patch_", &Minipatch::patch_)
      .def_readonly("width_", &Minipatch::width_)
      .def_readwrite("is_new_", &Minipatch::is_new_);

  py::class_<std::unordered_map<size_t, NodeHybrid>>(m, "unordered_map_hybrid")
      .def(py::init<>())
      .def("at",
           [](const std::unordered_map<size_t, NodeHybrid>& val, size_t node_index) { return val.at(node_index); })
      .def("__len__", [](const std::unordered_map<size_t, NodeHybrid>& val) { return val.size(); })
      .def("__iter__",
           [](const std::unordered_map<size_t, NodeHybrid>& val) { return py::make_iterator(val.begin(), val.end()); });

  py::class_<std::unordered_map<size_t, NodeDisc>>(m, "unordered_map_disc")
      .def(py::init<>())
      .def("at", [](const std::unordered_map<size_t, NodeDisc>& val, size_t node_index) { return val.at(node_index); })
      .def("__len__", [](const std::unordered_map<size_t, NodeDisc>& val) { return val.size(); })
      .def("__iter__",
           [](const std::unordered_map<size_t, NodeDisc>& val) { return py::make_iterator(val.begin(), val.end()); });

  py::class_<CollisionChecker>(m, "CollisionChecker")
      .def_readonly_static("MAX_PATCH_INS_DIST", &CollisionChecker::max_patch_ins_dist_)
      .def_readwrite_static("disk_r_c_", &CollisionChecker::disk_r_c_, "disk_r_c_")
      .def_readwrite_static("disk_r_", &CollisionChecker::disk_r_, "disk_r_")
      .def("initialize", &CollisionChecker::initialize, "Initializes arrays to the sizes in config")
      .def("calculateDisks", &CollisionChecker::calculateDisks, "calculateDisks")
      .def("insertMinipatches", &CollisionChecker::insertMinipatches, "insertMinipatches")
      .def("passLocalMap",
           py::overload_cast<const py::array_t<uint8_t>&, const Point<int>&, int>(&CollisionChecker::passLocalMap),
           "Pass local map")
      .def("passLocalMap",
           py::overload_cast<const Vec2DFlat<uint8_t>&, const Point<int>&, int>(&CollisionChecker::passLocalMap),
           "passLocalMap")
      .def_readonly_static("patch_arr_", &CollisionChecker::patch_arr_)
      .def_readonly_static("patch_safety_arr_", &CollisionChecker::patch_safety_arr_)
      .def("returnDiskPositions", &CollisionChecker::returnDiskPositions, "returnDiskPositions")
      .def("checkGrid", &CollisionChecker::checkGrid, "Check for collision in grid")
      .def("checkGridVariant", &CollisionChecker::checkGrid, "Check for collision in grid")
      .def("checkPose", &CollisionChecker::checkPose, "Check for collision of cont. pose in grid")
      .def("checkPoseVariant", &CollisionChecker::checkPoseVariant, "Check for collision of cont. pose in grid")
      .def("getPathCollisionIndex",
           &CollisionChecker::getPathCollisionIndex,
           "Check for collision of a path in grid, return index of collision")
      .def("processSafetyPatch", &CollisionChecker::processSafetyPatch, "dilate patch for collision checks");

  py::class_<HybridAStar>(m, "HybridAStar")
      .def_readonly_static("GM_RES", &GM_RES)
      .def_readonly_static("ASTAR_RES", &ASTAR_RES)
      .def_readonly_static("OUT_OF_HEURISTIC", &HybridAStar::OUT_OF_HEURISTIC)
      .def_readwrite_static("switch_cost_", &HybridAStar::switch_cost_)
      .def_readwrite_static("steer_cost_", &HybridAStar::steer_cost_)
      .def_readwrite_static("steer_change_cost_", &HybridAStar::steer_change_cost_)
      .def_readwrite_static("h_dist_cost_", &HybridAStar::h_dist_cost_)
      .def_readwrite_static("back_cost_", &HybridAStar::back_cost_)
      .def_readwrite_static("h_prox_cost_", &HybridAStar::h_prox_cost_)

      .def_readwrite_static("astar_dim_", &AStar::astar_dim_)
      .def_readwrite_static("lane_graph_", &HybridAStar::lane_graph_)
      .def_readwrite_static("connected_closed_nodes_", &HybridAStar::connected_closed_nodes_)

      .def("setSim", &HybridAStar::setSim)
      .def("initialize", &HybridAStar::initialize)
      .def("reinit", &HybridAStar::reinit)
      .def("hybridAStarPlanning",
           &HybridAStar::hybridAStarPlanning,
           "Hybrid A Star algorithm with heuristic creation and path extraction")
      .def("getEmergencyPath", &HybridAStar::getEmergencyPath, "getEmergencyPath")
      .def("getNonhnoobsVal", &HybridAStar::getNonhnoobsVal, "getNonhnoobsVal")
      .def("recalculateEnv", &HybridAStar::recalculateEnv, "recalculateEnv")
      .def("createNode", &HybridAStar::createNode, "createNode")
      .def("getDistance2GlobalGoal", &HybridAStar::getDistance2GlobalGoal, "getDistance2GlobalGoal")
      .def("projEgoOnPath", &HybridAStar::projEgoOnPath, "projEgoOnPath")
      .def("getValidClosePose", &HybridAStar::getValidClosePose, "getValidClosePose")
      .def("resetLaneGraph", &HybridAStar::resetLaneGraph, "resetLaneGraph")
      .def("updateLaneGraph", &HybridAStar::updateLaneGraph, "updateLaneGraph")
      .def("smoothLaneGraph", &HybridAStar::smoothLaneGraph, "smoothLaneGraph")
      .def("interpolateLaneGraph", &HybridAStar::interpolateLaneGraph, "smoothLaneGraph");

  py::class_<Smoother>(m, "Smoother")
      .def("smooth_path", &Smoother::smooth_path)

      .def_readwrite_static("max_iter_", &Smoother::max_iter_)
      .def_readwrite_static("wSmoothness_", &Smoother::wSmoothness_)
      .def_readwrite_static("wObstacle_", &Smoother::wObstacle_)
      .def_readwrite_static("wCurvature_", &Smoother::wCurvature_)
      .def_readwrite_static("alpha_", &Smoother::alpha_opt_);

  py::class_<NodeHybrid>(m, "NodeHybrid")
      .def(py::init<int,
                    int,
                    int,
                    int,
                    std::vector<int>,
                    std::vector<double>,
                    std::vector<double>,
                    std::vector<double>,
                    std::vector<double>,
                    std::vector<PATH_TYPE>,
                    int64,
                    double,
                    double>())
      // The name in " " is visible in python
      .def_readwrite("x_index", &NodeHybrid::x_index)
      .def_readwrite("y_index", &NodeHybrid::y_index)
      .def_readwrite("yaw_index", &NodeHybrid::yaw_index)
      .def_readwrite("discrete_direction", &NodeHybrid::discrete_direction)
      .def_readwrite("dir_list_cont", &NodeHybrid::dir_list_cont)
      .def_readwrite("x_list", &NodeHybrid::x_list)
      .def_readwrite("y_list", &NodeHybrid::y_list)
      .def_readwrite("yaw_list", &NodeHybrid::yaw_list)
      .def_readwrite("delta_list", &NodeHybrid::delta_list)
      .def_readwrite("types", &NodeHybrid::types)
      .def_readwrite("parent_index", &NodeHybrid::parent_index)
      .def_readwrite("cost", &NodeHybrid::cost)
      .def_readwrite("dist", &NodeHybrid::dist);

  py::class_<Vehicle>(m, "Vehicle")
      .def_readwrite_static("vis_vehicle_vertices_", &Vehicle::vis_vehicle_vertices_, "vis_vehicle_vertices_")
      .def("initialize", &Vehicle::initialize, "(re)Initializes the car params")
      .def("setPose", &Vehicle::setPose, "setPose");

  py::class_<Cartographing>(m, "Cartographing")
      .def("resetPatch", &Cartographing::resetPatch, "resetPatch")
      .def("cartograph",
           py::overload_cast<const py::array_t<uint8_t>&, const Point<int>&, int>(&Cartographing::cartograph),
           "cartograph")
      .def("cartograph",
           py::overload_cast<const Vec2DFlat<uint8_t>&, const Point<int>&, int>(&Cartographing::cartograph),
           "cartograph")
      .def("passLocalMap", &Cartographing::passLocalMap, "passLocalMap")
      .def("getMap", &Cartographing::getMap, "getMap")
      .def("loadPrevPatch", &Cartographing::loadPrevPatch, "loadPrevPatch");

  py::class_<Path>(m, "Path")
      .def("slice", &Path::slice, "slice")
      .def(py::init<std::vector<double>,
                    std::vector<double>,
                    std::vector<double>,
                    std::vector<double>,
                    std::vector<int>,
                    double,
                    int,
                    std::vector<PATH_TYPE>,
                    bool>())

      .def_readwrite("x_list", &Path::x_list)
      .def_readwrite("y_list", &Path::y_list)
      .def_readwrite("yaw_list", &Path::yaw_list)
      .def_readwrite("delta_list", &Path::delta_list)
      .def_readwrite("direction_list", &Path::direction_list)
      .def_readwrite("cost", &Path::cost)
      .def_readwrite("idx_analytic", &Path::idx_analytic)
      .def_readwrite("types", &Path::types)
      .def_readwrite("is_emergency", &Path::is_emergency);

  py::enum_<PATH_TYPE>(m, "PATH_TYPE", py::arithmetic())
      .value("HASTAR", PATH_TYPE::HASTAR)
      .value("REAR_AXIS", PATH_TYPE::REAR_AXIS)
      .value("REEDS_SHEPP", PATH_TYPE::REEDS_SHEPP)
      .value("EMERGENCY", PATH_TYPE::EMERGENCY)
      .value("UNKNOWN", PATH_TYPE::UNKNOWN)
      .export_values();

  py::class_<NodeDisc>(m, "NodeDisc")
      .def(py::init<Point<int>, double, double, int, bool>())
      .def_readwrite("pos", &NodeDisc::pos)
      .def_readwrite("cost_", &NodeDisc::cost_)
      .def_readwrite("cost_dist_", &NodeDisc::cost_dist_)
      .def_readwrite("parent_index_", &NodeDisc::parent_index_)
      .def_readwrite("is_unknown_", &NodeDisc::is_unknown_);

  py::class_<AStar>(m, "AStar")
      .def_readwrite_static("alpha_", &AStar::alpha_)
      .def_readwrite_static("do_max_", &AStar::do_max_)
      .def_readwrite_static("do_min_", &AStar::do_min_)
      .def_readwrite_static("astar_prox_cost_", &AStar::astar_prox_cost_)
      .def_readwrite_static("astar_movement_cost_", &AStar::astar_movement_cost_)
      .def_readwrite_static("astar_lane_movement_cost_", &AStar::astar_lane_movement_cost_)
      .def_readwrite_static("closed_set_path_", &AStar::closed_set_path_)
      .def_readwrite_static("closed_set_guidance_", &AStar::closed_set_guidance_)

      .def_readonly_static("voronoi_dist_", &AStar::voronoi_dist_)
      .def_readonly_static("obstacle_dist_", &AStar::obstacle_dist_)

      .def_readonly_static("movement_cost_map_", &AStar::movement_cost_map_)
      .def_readonly_static("astar_grid_", &AStar::astar_grid_)
      .def_readonly_static("h_prox_arr_", &AStar::h_prox_arr_)
      .def_readonly_static("motion_res_map_", &AStar::motion_res_map_)
      .def_readwrite_static("restr_geofence_", &AStar::restr_geofence_)
      .def("getAstarPath", &AStar::getAstarPath, "Extracts the discrete a star path")
      .def("processGeofence", &AStar::processGeofence, "processGeofence");

  auto util = m.def_submodule("UtilCpp");

  util.def("utm2grid", py::overload_cast<const Point<double>&>(&grid_tf::utm2grid<Point<double>>), "utm2grid");
  util.def("utm2grid", py::overload_cast<const Pose<double>&>(&grid_tf::utm2grid<Pose<double>>), "utm2grid");
  util.def(
      "utm2grid", py::overload_cast<const std::vector<double>&>(&grid_tf::utm2grid<std::vector<double>>), "utm2grid");
  util.def(
      "utm2grid", py::overload_cast<const pair_of_vec<double>&>(&grid_tf::utm2grid<pair_of_vec<double>>), "utm2grid");

  util.def("utm2grid_round", py::overload_cast<const Pose<double>&>(&grid_tf::utm2grid_round), "utm2grid_round");
  util.def("utm2grid_round", py::overload_cast<const Point<double>&>(&grid_tf::utm2grid_round), "utm2grid_round");
  util.def("utm2grid_round", py::overload_cast<double>(&grid_tf::utm2grid_round), "utm2grid_round");

  util.def("grid2utm", py::overload_cast<const Point<double>&>(&grid_tf::grid2utm<Point<double>>), "grid2utm");
  util.def("grid2utm", py::overload_cast<const Pose<double>&>(&grid_tf::grid2utm<Pose<double>>), "grid2utm");
  util.def(
      "grid2utm", py::overload_cast<const std::vector<double>&>(&grid_tf::grid2utm<std::vector<double>>), "grid2utm");
  util.def("grid2utm", py::overload_cast<const Path&>(&grid_tf::grid2utm<Path>), "grid2utm");
  util.def("grid2utm", py::overload_cast<const double&>(&grid_tf::grid2utm<double>), "grid2utm");

  util.def("astar2utm", py::overload_cast<const Point<double>&>(&grid_tf::astar2utm<Point<double>>), "astar2utm");
  util.def("astar2utm", py::overload_cast<const Pose<double>&>(&grid_tf::astar2utm<Pose<double>>), "astar2utm");
  util.def("astar2utm",
           py::overload_cast<const pair_of_vec<double>&>(&grid_tf::astar2utm<pair_of_vec<double>>),
           "astar2utm");

  util.def("utm2astar", py::overload_cast<const Point<double>&>(&grid_tf::utm2astar<Point<double>>), "utm2astar");
  util.def("utm2astar", py::overload_cast<const Pose<double>&>(&grid_tf::utm2astar<Pose<double>>), "utm2astar");
  util.def("utm2astar",
           py::overload_cast<const pair_of_vec<double>&>(&grid_tf::utm2astar<pair_of_vec<double>>),
           "utm2astar");

  util.def("utm2patch_utm", py::overload_cast<const Point<double>&>(&grid_tf::utm2patch_utm), "utm2patch_utm");
  util.def("utm2patch_utm", py::overload_cast<const Pose<double>&>(&grid_tf::utm2patch_utm), "utm2patch_utm");
  util.def("utm2patch_utm",
           py::overload_cast<const std::vector<double>&, const std::vector<double>&>(&grid_tf::utm2patch_utm),
           "utm2patch_utm");

  util.def("patch_utm2utm", py::overload_cast<const Point<double>&>(&grid_tf::patch_utm2utm), "patch_utm2utm");
  util.def("patch_utm2utm", py::overload_cast<const Pose<double>&>(&grid_tf::patch_utm2utm), "patch_utm2utm");
  util.def("patch_utm2utm",
           py::overload_cast<const std::vector<double>&, const std::vector<double>&>(&grid_tf::patch_utm2utm),
           "patch_utm2utm");
  util.def("patch_utm2utm", py::overload_cast<const Path&>(&grid_tf::patch_utm2utm), "patch_utm2utm");

  util.def("patch_utm2global_gm",
           py::overload_cast<const Point<double>&>(&grid_tf::patch_utm2global_gm),
           "patch_utm2global_gm");
  util.def("patch_utm2global_gm",
           py::overload_cast<const Pose<double>&>(&grid_tf::patch_utm2global_gm),
           "patch_utm2global_gm");
  util.def("patch_utm2global_gm",
           py::overload_cast<const std::vector<double>&, const std::vector<double>&>(&grid_tf::patch_utm2global_gm),
           "patch_utm2global_gm");

  util.def("utm2global_gm", &grid_tf::utm2global_gm, "utm2global_gm");

  util.def("getPathLength", &util::getPathLength);
}
