//
// Created by schumann on 12.09.22.
//
#ifndef GRIDMAP_SIM_LIB_HPP
#define GRIDMAP_SIM_LIB_HPP

#include <iostream>
#include <filesystem>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

constexpr size_t GM_DIM = 801;
constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180;
constexpr double YAW_RES = 2 * DEG2RAD;
constexpr int NB_PRECASTS = 180;

namespace py = pybind11;

// Enums that describe the possible values of the given map to apply the raytracing on
enum
{
  FREE = 0,
  UNKNOWN = 127,
  OCCUPIED = 255,
  PLACEHOLDER = 3
};

template <typename T>
class Arr2DFlat
{
private:
  std::array<T, GM_DIM * GM_DIM> arr_;

public:
  void setTo(int val)
  {
    std::fill(arr_.begin(), arr_.end(), val);
  }

  [[nodiscard]] std::pair<size_t, size_t> getDims() const
  {
    return { GM_DIM, GM_DIM };
  }

  [[nodiscard]] py::array_t<T> getNumpyArr() const
  {
    return py::array_t<T>({ GM_DIM, GM_DIM }, arr_.data());
  }

  [[nodiscard]] T operator()(int y_index, int x_index) const
  {
    return arr_[y_index * GM_DIM + x_index];
  }

  [[nodiscard]] T& operator()(int y_index, int x_index)
  {
    return arr_[y_index * GM_DIM + x_index];
  }

  T* getPtr()
  {
    return arr_.data();
  }

  [[nodiscard]] T* getPtr() const
  {
    return arr_.data();
  }
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
  inline static constexpr int middle_ = static_cast<int>(static_cast<double>(GM_DIM - 1) / 2);

  inline static std::array<std::vector<PrecastDB>, NB_PRECASTS> precast_list_;
  inline static Arr2DFlat<uint8_t> pmap_;

public:
  static void initialize();

  static void calculatePrecasts();

  static void raytracing(const py::array_t<uint8_t>& grid_map_gt);

  static double atan2Pi(double y_coord, double x_coord);

  static py::array_t<uint8_t> gmSimulation(const py::array_t<uint8_t>& grid_map_gt);
};

#endif  // GRIDMAP_SIM_LIB_HPP
