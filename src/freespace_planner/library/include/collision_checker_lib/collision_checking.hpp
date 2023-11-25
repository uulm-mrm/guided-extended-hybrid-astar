//
// Created by schumann on 11.05.22.
//
#ifndef PLANNING_BINDINGS_COLLISION_CHECKING_HPP
#define PLANNING_BINDINGS_COLLISION_CHECKING_HPP

#include <cmath>
#include <vector>
#include <iostream>
#include <execution>  // for parallel execution of std::transform...
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/cudafilters.hpp"
#include "opencv2/cudaimgproc.hpp"

#include "yaml-cpp/yaml.h"

#include "util_lib/util1.hpp"
#include "util_lib/data_structures1.hpp"
#include "util_lib/transforms.hpp"

#include "vehicle.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

/**
 * Collision Checking lib that checks for collisions on a grid map that is dilated by the diameter of the disks that
 * are used to represent the ego vehicle
 * Here, the sparse collision checking of the following paper of Ziegler et. al is used:
 * https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5547976
 */
class CollisionChecker
{
public:
  enum GRID_VAL
  {
    FREE,     // 2
    UNKNOWN,  // 1
    OCC,      // 0
    SENSOR_FREE = 255,
    SENSOR_UNKNOWN = 127,
    SENSOR_OCC = 0
  };

private:
  inline static size_t patch_dim_;
  inline static uint8_t min_thresh_;
  inline static uint8_t max_thresh_;
  inline static unsigned int yaw_res_coll_;
  inline static double yaw_res_coll_rad_;
  inline static unsigned int nb_disc_yaws_;
  inline static unsigned int nb_disks_;
  inline static bool double_disk_rows_;
  inline static double len_per_disk_;

  inline static int disk_diameter_c_;
  inline static double safety_distance_m_;
  inline static cv::Mat dil_kernel_;
  inline static cv::Ptr<cv::cuda::Filter> dilateFilter_;
  inline static double search_dist_;
  inline static double max_patch_ins_dist_;

  // array of disk centers
  inline static Vec2DFlat<Point<int>> disk_centers_;

public:
  inline static double gm_res_;
  inline static int disk_r_c_;

  inline static Vec2DFlat<uint8_t> patch_arr_;
  inline static Vec2DFlat<uint8_t> patch_safety_arr_;

  inline static size_t getPatchDim()
  {
    return patch_dim_;
  }

  static void initialize(size_t patch_dim, const std::string& path2config);

  static void calculateDisks();

  static double getDiskRadius(double length, double width, unsigned int nb_disks);

  static std::vector<double> getDiskPositions(double radius, unsigned int nb_disks, double width, double lb);

  static std::vector<Point<int>> returnDiskPositions(double yaw);

  static void resetPatch(size_t patch_dim);

  static void processSafetyPatch();

  static void insertMinipatches(const std::map<std::pair<int, int>, Minipatch>& minipatches,
                                const Point<double>& ego_utm,
                                bool only_nearest,
                                bool only_new);

  static void passLocalMap(const py::array_t<uint8_t>& local_map, const Point<int>& origin, int dim);

  static void passLocalMap(const Vec2DFlat<uint8_t>& local_map, const Point<int>& origin, int dim);

  static void passLocalMapData(const unsigned char* local_map, const Point<int>& origin, int dim);

  static int getYawIdx(double yaw);

  static bool checkGrid(const Pose<int>& pose);

  static bool checkGridVariant(const Pose<int>& pose);

  static bool checkPose(const Pose<double>& pose);

  static bool checkPoseVariant(const Pose<double>& pose);

  static int getPathCollisionIndex(const std::vector<double>& x_list,
                                   const std::vector<double>& y_list,
                                   const std::vector<double>& yaw_list);

  static bool checkPathCollision(const std::vector<double>& x_list,
                                 const std::vector<double>& y_list,
                                 const std::vector<double>& yaw_list);
};
#endif  // PLANNING_BINDINGS_COLLISION_CHECKING_HPP
