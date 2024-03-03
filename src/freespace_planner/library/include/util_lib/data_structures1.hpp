//
// Created by schumann on 2/1/23.
//

#ifndef FREESPACE_PLANNER_DATA_STRUCTURES1_HPP
#define FREESPACE_PLANNER_DATA_STRUCTURES1_HPP

#include <ostream>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <queue>
#include <algorithm>
#include <unordered_map>

#include <util_lib/util1.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#ifdef NDEBUG
// nondebug
#define RANGE_CHECKS false  // 1.95s + 8%
#define CLIP false          // 2s + 11%
#else
// debug code
#define RANGE_CHECKS true  // 1.95s + 8%
#define CLIP false         // 2s + 11%
#endif

namespace py = pybind11;

/**
 * A 2D Vector = Point
 * @tparam T
 */
template <typename T>
class Point
{
public:
  Point() = default;
  constexpr Point(const T x_in, const T y_in) : x(x_in), y(y_in){};

  [[nodiscard]] double dist2(const Point<T>& point) const
  {
    const double xdiff = x - point.x;
    const double ydiff = y - point.y;
    return std::abs(std::sqrt(xdiff * xdiff + ydiff * ydiff));
  }

  [[nodiscard]] Point<double> toDouble() const
  {
    return { static_cast<double>(x), static_cast<double>(y) };
  }

  [[nodiscard]] Point<int> toInt() const
  {
    return { static_cast<int>(std::round(x)), static_cast<int>(std::round(y)) };
  }

  [[nodiscard]] Point<T> rotate(const double yaw) const
  {
    return Point(x * cos(yaw) - y * sin(yaw), x * sin(yaw) + y * cos(yaw));
  }

  [[nodiscard]] Point<T> rotatePreCalc(double cos_yaw, double sin_yaw) const
  {
    return Point(x * cos_yaw - y * sin_yaw, x * sin_yaw + y * cos_yaw);
  }

  /// a method to multiply a vector by a scalar
  [[nodiscard]] Point<T> operator*(const double val) const
  {
    return Point<T>(x * val, y * val);
  }
  /// a method to divide a vector by a scalar
  [[nodiscard]] Point<T> operator/(const double val) const
  {
    return Point<T>(x / val, y / val);
  }
  /// a method to add a vector by a scalar
  [[nodiscard]] Point<T> operator+(const double val) const
  {
    return Point<T>(x + val, y + val);
  }
  /// a method to subtract a vector by a scalar
  [[nodiscard]] Point<T> operator-(const double val) const
  {
    return Point<T>(x - val, y - val);
  }
  /// a method to add a vector to a vector
  [[nodiscard]] Point<T> operator+(const Point<T>& point) const
  {
    return Point<T>(x + point.x, y + point.y);
  }
  /// a method to subtract a vector from a vector
  [[nodiscard]] Point<T> operator-(const Point<T>& point) const
  {
    return Point<T>(x - point.x, y - point.y);
  }
  /// a method to negate a vector
  [[nodiscard]] Point<T> operator-() const
  {
    return Point<T>(-x, -y);
  }
  /// a convenience method to print a point
  friend std::ostream& operator<<(std::ostream& out_stream, const Point<T>& point)
  {
    out_stream << "(" << point.x << "|" << point.y << ")";
    return out_stream;
  }
  /// a method to calculate the length of the vector
  [[nodiscard]] double length() const
  {
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
  }
  /// a method to calculate the length of the vector
  [[nodiscard]] double sqlength() const
  {
    return x * x + y * y;
  }
  /// a method to calculate the dot product of two vectors
  [[nodiscard]] double dot(Point<T> point) const
  {
    return x * point.x + y * point.y;
  }
  /// a method that returns the orthogonal complement of two vectors
  [[nodiscard]] Point<T> ort(Point<T> point) const
  {
    const Point<T> this_point(x, y);
    // multiply point by the dot product of this and point then divide it by point's length
    return this_point - point * this_point.dot(point) / point.sqlength();
    ;
  }
  [[nodiscard]] double getX() const
  {
    return x;
  }
  [[nodiscard]] double getY() const
  {
    return y;
  }
  inline void setX(double val)
  {
    x = val;
  }
  inline void setY(double val)
  {
    y = val;
  }
  [[nodiscard]] bool equal(const Point<T>& point) const
  {
    return (std::abs(x - point.x) < 0.001) && (std::abs(y - point.y) < 0.001);
  }

  [[nodiscard]] Point<T> round() const
  {
    return { std::round(x), std::round(y) };
  }
  // compatibility to functionality of python objects
  [[nodiscard]] Point<T> copy() const
  {
    return { x, y };
  }

  [[nodiscard]] std::string toString() const
  {
    return "(" + std::to_string(x) + "|" + std::to_string(y) + ")";
  }

  T x = 0;
  T y = 0;
};
template <typename T>
inline Point<T> operator*(double val, const Point<T>& point)
{
  return (point * val);
}

/**
 * A 3D vector = Pose
 * @tparam T
 */
template <typename T>
class Pose
{
public:
  Pose() = default;
  Pose(T x_in, T y_in, T yaw_in) : x(x_in), y(y_in), yaw(yaw_in){};

  /// a method to multiply a Pose by a scalar
  inline Pose<T> operator*(const double val) const
  {
    return Pose<T>(x * val, y * val, yaw);
  }
  /// a method to divide a Pose by a scalar
  inline Pose<T> operator/(const double val) const
  {
    return Pose<T>(x / val, y / val, yaw);
  }
  /// a method to add a scalar to a Pose
  inline Pose<T> operator+(const double val) const
  {
    return Pose<T>(x + val, y + val, yaw);
  }
  /// a method to subtract a scalar to a Pose
  inline Pose<T> operator-(const double val) const
  {
    return Pose<T>(x - val, y - val, yaw);
  }
  /// a method to add a Pose to a Pose
  inline Pose<T> operator+(const Pose<T>& point) const
  {
    return Pose<T>(x + point.x, y + point.y, yaw);
  }
  /// a method to subtract a Pose from a Pose
  inline Pose<T> operator-(const Pose<T>& point) const
  {
    return Pose<T>(x - point.x, y - point.y, yaw);
  }

  /// a method to add a Point to a Pose
  inline Pose<T> operator+(const Point<T>& point) const
  {
    return Pose<T>(x + point.x, y + point.y, yaw);
  }

  /// a method to subtract a Point from a Pose
  inline Pose<T> operator-(const Point<T>& point) const
  {
    return Pose<T>(x - point.x, y - point.y, yaw);
  }

  /// a method to negate a Pose
  inline Pose<T> operator-() const
  {
    return Pose<T>(-x, -y, yaw);
  }

  [[nodiscard]] double dist2(const Pose<T>& pose) const
  {
    const double xdiff = x - pose.x;
    const double ydiff = y - pose.y;
    return std::abs(std::sqrt(xdiff * xdiff + ydiff * ydiff));
  }

  [[nodiscard]] double dist2(const Point<T>& point) const
  {
    const double xdiff = x - point.x;
    const double ydiff = y - point.y;
    return std::abs(std::sqrt(xdiff * xdiff + ydiff * ydiff));
  }

  [[nodiscard]] Point<T> getPoint() const
  {
    return Point(x, y);
  }

  friend std::ostream& operator<<(std::ostream& out_stream, const Pose& point)
  {
    out_stream << "(" << point.x << "|" << point.y << "|" << point.yaw << ")";
    return out_stream;
  }

  [[nodiscard]] Pose<T> round()
  {
    return { std::round(x), std::round(y), yaw };
  }
  // compatibility to functionality of python objects
  [[nodiscard]] Pose<T> copy()
  {
    return { x, y, yaw };
  }

  [[nodiscard]] Pose<int> toInt() const
  {
    return Pose<int>(static_cast<int>(std::round(x)), static_cast<int>(std::round(y)), yaw);
    ;
  }

  [[nodiscard]] bool equal(const Pose<T>& pose)
  {
    return static_cast<bool>(x == pose.x and y == pose.y and yaw == pose.yaw);
  }

  [[nodiscard]] std::string toString() const
  {
    return "(" + std::to_string(x) + "|" + std::to_string(y) + "|" + std::to_string(yaw) + ")";
  }

  T x = 0;
  T y = 0;
  T yaw = 0;
};
template <typename T>
inline Pose<T> operator*(double val, const Pose<T>& pose)
{
  return (pose * val);
}

/**
 * Line consisting of two 2D vectors
 * @tparam T
 */
template <typename T>
class Line2D
{
public:
  Line2D(Point<T> p1_in, Point<T> p2_in) : p1(std::move(p1_in)), p2(std::move(p2_in)){};

  Point<T> p1;
  Point<T> p2;
};

class Polygon
{
public:
  Polygon() = default;

  inline void addPoint(const Point<double>& point)
  {
    vertices.push_back(point);
  }
  inline void reset()
  {
    vertices.clear();
  }

  std::vector<Point<double>> vertices;
};

/**
 * Flatten version of a 2D vector that can be indexed with 2D indices
 * @tparam T
 */
template <typename T>
class Vec2DFlat
{
private:
  std::string name_ = "not set";
  // Dimensions in each direction
  int xDim_{};
  int yDim_{};

public:
  std::vector<T> vec_;

  [[nodiscard]] std::pair<int, int> getDims() const
  {
    return { xDim_, yDim_ };
  }

  [[nodiscard]] bool is_empty() const
  {
    return vec_.empty();
  }

  void setName(std::string name)
  {
    name_ = name;
  }

  [[nodiscard]] py::array_t<T> getNumpyArr() const
  {
    return py::array_t<T>({ xDim_, yDim_ }, vec_.data());
  }

  void resize_and_reset(size_t x_dim, size_t y_dim, T val)
  {
    /*
     * Resize the inner 1d vector
     */
    xDim_ = x_dim;
    yDim_ = y_dim;
    vec_.assign(x_dim * y_dim, val);
    vec_.shrink_to_fit();
  }

  void resize(size_t x_dim, size_t y_dim)
  {
    /*
     * Resize the inner 1d vector
     */
    xDim_ = x_dim;
    yDim_ = y_dim;
    vec_.resize(x_dim * y_dim);
    vec_.shrink_to_fit();
  }

  [[nodiscard]] T operator()(int y_index, int x_index) const
  {
    if (RANGE_CHECKS)
    {
      if (y_index > yDim_ || y_index < 0)
      {
        throw std::out_of_range("y index out of bounds in " + name_ + "\nindex " + std::to_string(y_index) + " of " +
                                std::to_string(yDim_));
      }
      if (x_index > xDim_ || x_index < 0)
      {
        throw std::out_of_range("x index out of bounds in " + name_ + "\nindex " + std::to_string(x_index) + " of " +
                                std::to_string(xDim_));
      }
    }

    if (CLIP)
    {
      x_index = std::clamp(x_index, 0, xDim_);
      y_index = std::clamp(y_index, 0, yDim_);
    }

    return vec_[y_index * xDim_ + x_index];
  }

  [[nodiscard]] T operator()(const Point<int>& point) const
  {
    if (RANGE_CHECKS)
    {
      if (point.y > yDim_ - 1 || point.y < 0)
      {
        throw std::out_of_range("y index out of bounds in " + name_ + "\nindex " + std::to_string(point.y) + " of " +
                                std::to_string(yDim_));
      }
      if (point.x > xDim_ - 1 || point.x < 0)
      {
        throw std::out_of_range("x index out of bounds in " + name_ + "\nindex " + std::to_string(point.x) + " of " +
                                std::to_string(xDim_));
      }
    }

    if (CLIP)
    {
      const int x = std::clamp(point.x, 0, xDim_ - 1);
      const int y = std::clamp(point.y, 0, yDim_ - 1);
      return vec_[y * xDim_ + x];
    }

    return vec_[point.y * xDim_ + point.x];
  }

  [[nodiscard]] T& operator()(int y_index, int x_index)
  {
    if (RANGE_CHECKS)
    {
      if (y_index > yDim_ - 1 || y_index < 0)
      {
        throw std::out_of_range("y index out of bounds in " + name_ + "\nindex " + std::to_string(y_index) + " of " +
                                std::to_string(yDim_));
      }
      if (x_index > xDim_ - 1 || x_index < 0)
      {
        throw std::out_of_range("x index out of bounds in " + name_ + "\nindex " + std::to_string(x_index) + " of " +
                                std::to_string(xDim_));
      }
    }
    if (CLIP)
    {
      x_index = std::clamp(x_index, 0, xDim_ - 1);
      y_index = std::clamp(y_index, 0, yDim_ - 1);
    }

    return vec_[y_index * xDim_ + x_index];
  }

  [[nodiscard]] T& operator()(const Point<int>& point)
  {
    if (RANGE_CHECKS)
    {
      if (point.y > yDim_ - 1 || point.y < 0)
      {
        throw std::out_of_range("y index out of bounds in " + name_ + "\nindex " + std::to_string(point.y) + " of " +
                                std::to_string(yDim_));
      }
      if (point.x > xDim_ - 1 || point.x < 0)
      {
        throw std::out_of_range("x index out of bounds in " + name_ + "\nindex " + std::to_string(point.x) + " of " +
                                std::to_string(xDim_));
      }
    }

    if (CLIP)
    {
      const int x = std::clamp(point.x, 0, xDim_ - 1);
      const int y = std::clamp(point.y, 0, yDim_ - 1);
      return vec_[y * xDim_ + x];
    }

    return vec_[point.y * xDim_ + point.x];
  }

  [[nodiscard]] std::vector<T> data() const
  {
    return vec_;
  }

  [[nodiscard]] std::vector<T>& data_ref()
  {
    /**
     * This allows the modification of the inner vector
     */
    return vec_;
  }

  T* getPtr()
  {
    return vec_.data();
  }

  [[nodiscard]] T* getPtr() const
  {
    return vec_.data();
  }
};

/**
 * Flatten version of a 3D vector that can be indexed with 3D indices
 * @tparam T
 */
template <typename T>
class Vec3DFlat
{
private:
  std::vector<T> vec_;
  std::string name_ = "not set";
  // Dimensions in each direction
  int xDim_{};
  int yDim_{};
  int zDim_{};

public:
  [[nodiscard]] std::tuple<int, int, int> getDims() const
  {
    return { xDim_, yDim_, zDim_ };
  }

  [[nodiscard]] bool is_empty() const
  {
    return vec_.empty();
  }

  void setName(std::string name)
  {
    name_ = name;
  }

  void resize_and_reset(size_t x_dim, size_t y_dim, size_t yaw_dim, T val)
  {
    /*
     * Resize the inner 1d vector
     */
    xDim_ = x_dim;
    yDim_ = y_dim;
    zDim_ = yaw_dim;
    vec_.assign(x_dim * y_dim * yaw_dim, val);
    vec_.shrink_to_fit();
  }

  [[nodiscard]] T operator()(int x_index, int y_index, int yaw_index) const
  {
    if (RANGE_CHECKS)
    {
      if (y_index > yDim_ - 1 || y_index < 0)
      {
        throw std::out_of_range("y index out of bounds in " + name_ + "\nindex " + std::to_string(y_index) + " of " +
                                std::to_string(yDim_));
      }
      if (x_index > xDim_ - 1 || x_index < 0)
      {
        throw std::out_of_range("x index out of bounds in " + name_ + "\nindex " + std::to_string(x_index) + " of " +
                                std::to_string(xDim_));
      }
      if (yaw_index > zDim_ - 1 - 1 || yaw_index < 0)
      {
        throw std::out_of_range("z index out of bounds in " + name_ + "\nindex " + std::to_string(yaw_index) + " of " +
                                std::to_string(zDim_));
      }
    }

    if (CLIP)
    {
      x_index = std::clamp(x_index, 0, xDim_ - 1);
      y_index = std::clamp(y_index, 0, yDim_ - 1);
      yaw_index = std::clamp(y_index, 0, zDim_ - 1);
    }

    return vec_[yaw_index * yDim_ * xDim_ + y_index * xDim_ + x_index];
  }

  [[nodiscard]] T getVal(int x_index, int y_index, int yaw_index) const
  {
    return vec_[yaw_index * yDim_ * xDim_ + y_index * xDim_ + x_index];
  }

  [[nodiscard]] T& operator()(int x_index, int y_index, int yaw_index)
  {
    if (RANGE_CHECKS)
    {
      if (y_index > yDim_ - 1 || y_index < 0)
      {
        throw std::out_of_range("y index out of bounds in " + name_ + "\nindex " + std::to_string(y_index) + " of " +
                                std::to_string(yDim_));
      }
      if (x_index > xDim_ - 1 || x_index < 0)
      {
        throw std::out_of_range("x index out of bounds in " + name_ + "\nindex " + std::to_string(x_index) + " of " +
                                std::to_string(xDim_));
      }
      if (yaw_index > zDim_ - 1 || yaw_index < 0)
      {
        throw std::out_of_range("yaw index out of bounds in " + name_ + "\nindex " + std::to_string(yaw_index) +
                                " of " + std::to_string(zDim_));
      }
    }

    if (CLIP)
    {
      x_index = std::clamp(x_index, 0, xDim_ - 1);
      y_index = std::clamp(y_index, 0, yDim_ - 1);
      yaw_index = std::clamp(y_index, 0, zDim_ - 1);
    }

    return vec_[yaw_index * yDim_ * xDim_ + y_index * xDim_ + x_index];
  }

  [[nodiscard]] std::vector<T> data() const
  {
    /**
     * This returns the actual vector
     */
    return vec_;
  }

  [[nodiscard]] std::vector<T>& data_ref()
  {
    /**
     * This allows the modification of the inner vector
     */
    return vec_;
  }

  [[nodiscard]] T* getPtr()
  {
    return vec_.data();
  }

  [[nodiscard]] T* getPtr() const
  {
    return vec_.data();
  }
};

#endif  // FREESPACE_PLANNER_DATA_STRUCTURES1_HPP
