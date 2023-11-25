//
// Created by schumann on 12.09.22.
//
#ifndef GRIDMAP_SIM_LIB_HPP
#define GRIDMAP_SIM_LIB_HPP

#include <iostream>
#include <filesystem>
#include <vector>
#include <execution>
#include <opencv2/opencv.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

constexpr size_t GM_DIM = 801;
constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180;
constexpr double YAW_RES = 1 * DEG2RAD;
constexpr int NB_PRECASTS = 360;

namespace py = pybind11;

// Enums that describe the possible values of the given map to apply the raytracing on
enum
{
  FREE = 0,
  UNKNOWN = 127,
  OCCUPIED = 255,
  PLACEHOLDER = 3
};

class PrecastDB
{
public:
  PrecastDB(double d_in, int ix_in, int iy_in) : d_(d_in), ix_(ix_in), iy_(iy_in){};

  double d_;
  int ix_;
  int iy_;

  // To sort precasts after distance with std::sort
  bool operator<(const PrecastDB& other) const
  {
    return d_ < other.d_;
  }
};

class GridMapSim
{
private:
  inline static bool is_initialized_ = false;
  inline static const int middle_ = static_cast<int>(round(static_cast<double>(GM_DIM) / 2));
  inline static const cv::Mat kernel_ = cv::Mat();

  inline static std::array<std::vector<PrecastDB>, NB_PRECASTS> precast_list_;
  inline static std::array<std::array<uint8_t, GM_DIM>, GM_DIM> pmap_;

public:
  static void initialize();

  static void calculatePrecasts();

  static void raytracing(const py::array_t<uint8_t>& grid_map_gt);

  static double atan2Pi(double y_coord, double x_coord);

  static py::array_t<uint8_t> gmSimulation(const py::array_t<uint8_t>& grid_map_gt);
};

#endif  // GRIDMAP_SIM_LIB_HPP
