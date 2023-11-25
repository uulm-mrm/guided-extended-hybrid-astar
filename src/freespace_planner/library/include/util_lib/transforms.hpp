//
// Created by schumann on 8/29/23.
//

#ifndef TRANSFORMS_HPP
#define TRANSFORMS_HPP

#include <algorithm>
#include <execution>

#include "util2.hpp"

template <typename T>
using pair_of_vec = std::pair<std::vector<T>, std::vector<T>>;

// Allow multiplication of vectors
template <typename T>
inline std::vector<T> operator*(const std::vector<T>& vec, double val)
{
  std::vector<T> vec_new(vec.size());
  std::transform(std::execution::unseq, vec.begin(), vec.end(), vec_new.begin(), [&val](const double& vec_val) {
    return vec_val * val;
  });
  return vec_new;
}

// Allow addition of vectors
template <typename T>
inline std::vector<T> operator+(const std::vector<T>& vec, double val)
{
  std::vector<T> vec_new(vec.size());
  std::transform(std::execution::unseq, vec.begin(), vec.end(), vec_new.begin(), [&val](const double& vec_val) {
    return vec_val + val;
  });
  return vec_new;
}

// Allow subtraction of vectors
template <typename T>
inline std::vector<T> operator-(const std::vector<T>& vec, double val)
{
  std::vector<T> vec_new(vec.size());
  std::transform(std::execution::unseq, vec.begin(), vec.end(), vec_new.begin(), [&val](const double& vec_val) {
    return vec_val - val;
  });
  return vec_new;
}

// Allow multiplication of pair of vector
template <typename T>
inline pair_of_vec<T> operator*(const pair_of_vec<T>& pair_v, double val)
{
  return { pair_v.first * val, pair_v.second * val };
}

// Allow processing all values of a vector with a given func
template <typename T>
inline std::vector<T> transformVec(const std::vector<T>& vec, std::function<T(T)> func)
{
  std::vector<T> vec_new(vec.size());

  std::transform(
      std::execution::unseq, vec.begin(), vec.end(), vec_new.begin(), [&func](const T& val) { return func(val); });

  return vec_new;
}

class grid_tf
{
public:
  inline static double gm2con_;
  inline static double con2gm_;

  inline static double star2con_;
  inline static double con2star_;

  inline static double gm2star_;
  inline static double star2gm_;

private:
  inline static Point<double> patch_origin_utm_;

public:
  static void updateTransforms(double gm_res, double astar_res, const Point<double>& patch_origin_utm);

  static void updateGmRes(double gm_res);

  static void updateAstarRes(double astar_res);

  static void updatePatchOrigin(const Point<double>& patch_origin_utm);

  template <typename T>
  static inline T utm2grid(const T& val)
  {
    return val * con2gm_;
  }

  static inline int utm2grid_round(double val)
  {
    return static_cast<int>(std::round(val * con2gm_));
  }

  static inline Point<int> utm2grid_round(const Point<double>& val)
  {
    return (val * con2gm_).toInt();
  }

  static inline Pose<int> utm2grid_round(const Pose<double>& val)
  {
    return (val * con2gm_).toInt();
  }

  template <typename T>
  static inline T grid2utm(const T& val)
  {
    return val * gm2con_;
  }

  template <typename T>
  static inline T utm2astar(const T& val)
  {
    return val * con2star_;
  }

  template <typename T>
  static inline T astar2utm(const T& val)
  {
    return val * star2con_;
  }

  template <typename T>
  static inline T grid2astar(const T& val)
  {
    return val * gm2star_;
  }

  template <typename T>
  static inline T astar2grid(const T& val)
  {
    return val * star2gm_;
  }

  /// utm 2 patch utm
  template <typename T>
  static inline T utm2patch_utm_x(T val)
  {
    return val - patch_origin_utm_.x;
  }
  template <typename T>
  static inline T utm2patch_utm_y(T val)
  {
    return val - patch_origin_utm_.y;
  }
  static Pose<double> utm2patch_utm(const Pose<double>& pose);

  static Point<double> utm2patch_utm(const Point<double>& point);

  static std::pair<std::vector<double>, std::vector<double>> utm2patch_utm(const std::vector<double>& x_vec,
                                                                           const std::vector<double>& y_vec);

  /// Patch utm 2 utm
  template <typename T>
  static inline T patch_utm2utm_x(T val)
  {
    return val + patch_origin_utm_.x;
  }
  template <typename T>
  static inline T patch_utm2utm_y(T val)
  {
    return val + patch_origin_utm_.y;
  }
  static Pose<double> patch_utm2utm(const Pose<double>& pose);

  static Point<double> patch_utm2utm(const Point<double>& point);

  static std::pair<std::vector<double>, std::vector<double>> patch_utm2utm(const std::vector<double>& x_vec,
                                                                           const std::vector<double>& y_vec);

  static Path patch_utm2utm(const Path& path);

  /// Patch utm 2 global grid
  static Pose<double> patch_utm2global_gm(const Pose<double>& pose);

  static Point<double> patch_utm2global_gm(const Point<double>& point);

  static pair_of_vec<double> patch_utm2global_gm(const std::vector<double>& x_vec, const std::vector<double>& y_vec);

  /// Utm 2 grid
  static pair_of_vec<double> utm2global_gm(const std::vector<double>& x_vec, const std::vector<double>& y_vec);
};

#endif  // TRANSFORMS_HPP
