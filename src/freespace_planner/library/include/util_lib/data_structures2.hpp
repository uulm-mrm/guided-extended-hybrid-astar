//
// Created by schumann on 8/15/23.
//

#ifndef FREESPACE_PLANNER_DATA_STRUCTURES2_HPP
#define FREESPACE_PLANNER_DATA_STRUCTURES2_HPP

#include <vector>

#include "data_structures1.hpp"

class MotionPrimitive
{
public:
  MotionPrimitive() = default;

  explicit MotionPrimitive(size_t nb_elements)
  {
    x_list_.reserve(nb_elements);
    y_list_.reserve(nb_elements);
    yaw_list_.reserve(nb_elements);
    dir_list_.reserve(nb_elements);
    nb_elements_ = nb_elements;
  }

  std::vector<double> x_list_;
  std::vector<double> y_list_;
  std::vector<double> yaw_list_;
  std::vector<int> dir_list_;
  size_t nb_elements_;

  void reserve(size_t nb_elements)
  {
    x_list_.reserve(nb_elements);
    y_list_.reserve(nb_elements);
    yaw_list_.reserve(nb_elements);
    dir_list_.reserve(nb_elements);
  }
};

/**
 * Object to execute the A* search on
 */
class NodeDisc
{
public:
  // Copy constructor
  NodeDisc() = default;
  NodeDisc(const NodeDisc&) = default;
  NodeDisc& operator=(const NodeDisc&) = default;

  NodeDisc(const Point<int>& pos_in, double cost_in, double cost_dist_in, int parent_index_in, bool is_unknown_in)
    : pos(std::move(pos_in))
    , cost_(cost_in)
    , cost_dist_(cost_dist_in)
    , parent_index_(parent_index_in)
    , is_unknown_(is_unknown_in){};

  Point<int> pos;
  double cost_;
  double cost_dist_;
  int parent_index_;
  bool is_unknown_;
};

enum PATH_TYPE
{
  HASTAR,       // 0
  REAR_AXIS,    // 1
  REEDS_SHEPP,  // 2
  EMERGENCY,
  UNKNOWN
};

/**
 * Node of the Hybrid A* Algorithm that is used during exploration
 */
class NodeHybrid
{
public:
  NodeHybrid() = default;
  NodeHybrid(const NodeHybrid&) = default;
  NodeHybrid& operator=(const NodeHybrid&) = default;

  NodeHybrid(int x_index_in,
             int y_index_in,
             int yaw_index_in,
             int discrete_direction_in,
             std::vector<int> dir_list_cont_in,
             std::vector<double> x_list_in,
             std::vector<double> y_list_in,
             std::vector<double> yaw_list_in,
             std::vector<PATH_TYPE> types_in,
             double steer_in,
             int64_t parent_index_in,
             double cost_in,
             double dist_in)
    : x_index(x_index_in)
    , y_index(y_index_in)
    , yaw_index(yaw_index_in)
    , discrete_direction(discrete_direction_in)
    , dir_list_cont(std::move(dir_list_cont_in))
    , x_list(std::move(x_list_in))
    , y_list(std::move(y_list_in))
    , yaw_list(std::move(yaw_list_in))
    , types(std::move(types_in))
    , steer(steer_in)
    , parent_index(parent_index_in)
    , cost(cost_in)
    , dist(dist_in){};

  void set_analytic()
  {
    is_analytic = true;
  }

  // To sort the nodes after costs
  [[nodiscard]] bool operator<(const NodeHybrid& other) const
  {
    return cost < other.cost;
  }

  [[nodiscard]] NodeDisc getNodeDisc() const
  {
    return { { x_index, y_index }, 0, 0, -1, false };
  }

  int x_index{};
  int y_index{};
  int yaw_index{};
  int discrete_direction{};
  std::vector<int> dir_list_cont;
  std::vector<double> x_list;
  std::vector<double> y_list;
  std::vector<double> yaw_list;
  std::vector<PATH_TYPE> types;
  double steer{};
  int64_t parent_index{};
  double cost = -1;
  double dist = 0;
  bool is_analytic = false;
};

class Segment
{
public:
  PATH_TYPE path_type;
  size_t start_idx;
  size_t end_idx;
};

/**
 * Structures the information of a path to be followed by the trajectory planning
 */
class Path
{
public:
  Path() = default;
  Path(const Path&) = default;
  Path(std::vector<double> x_list_in,
       std::vector<double> y_list_in,
       std::vector<double> yaw_list_in,
       std::vector<int> direction_list_in,
       double cost_in,
       int idx_analytic_in,
       std::vector<PATH_TYPE> types_in,
       bool is_emergency_in = false)
    : x_list(std::move(x_list_in))
    , y_list(std::move(y_list_in))
    , yaw_list(std::move(yaw_list_in))
    , direction_list(std::move(direction_list_in))
    , cost(cost_in)
    , idx_analytic(idx_analytic_in)
    , types(std::move(types_in))
    , is_emergency(is_emergency_in){};

  std::vector<double> x_list;
  std::vector<double> y_list;
  std::vector<double> yaw_list;
  std::vector<int> direction_list;
  double cost = 0;
  int idx_analytic = -1;
  std::vector<PATH_TYPE> types;
  bool is_emergency = false;

  // to sort paths
  [[nodiscard]] bool operator<(const Path& other) const
  {
    return cost < other.cost;
  }
};

/**
 * Priority queue that drives the A* search, nodes are sorted by their estimated cost to the goal
 * This class sorts also the Hybrid Nodes
 * @tparam T
 * @tparam priority_t
 */
template <typename T, typename priority_t>
struct PriorityQueue
{
  using PqElement = std::pair<priority_t, T>;
  std::priority_queue<PqElement, std::vector<PqElement>, std::greater<PqElement>> elements;

  [[nodiscard]] inline bool empty() const
  {
    return elements.empty();
  }

  inline void put(T item, priority_t priority)
  {
    elements.emplace(priority, item);
  }

  T get()
  {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }
};

class LaneNode
{
public:
  explicit LaneNode(Point<double> point) : point_utm(point){};

  Point<double> point_utm;
  Point<double> point_utm_patch;
  int index = -1;
  int neighbor_1 = -1;
  int neighbor_2 = -1;
};

class LaneGraph
{
public:
  using edge_t = std::tuple<std::optional<LaneNode>, LaneNode, std::optional<LaneNode>>;
  using edges_t = std::vector<edge_t>;

  LaneGraph() = default;
  LaneGraph(const Point<double>& origin_utm, double patch_dim)
  {
    init(origin_utm, patch_dim);
  }

  /**
   * sets the patch origin and dim
   * @param origin_utm
   * @param patch_dim
   */
  void init(const Point<double>& origin_utm, double patch_dim)
  {
    patch_origin_utm_ = origin_utm;
    patch_dim_ = patch_dim;

    recalculateNodeCoords();
    create();
  }

  void reinit()
  {
    recalculateNodeCoords();
    create();
  }

  /**
   * sets the patch origin and dim
   * @param origin_utm
   * @param patch_dim
   */
  void reset()
  {
    nodes_.clear();
    edges_.clear();
  }

  void create()
  {
    setNeighbors();
    setEdges();
  }

  /**
   * recalculates the patch_utm coords of all nodes
   */
  void recalculateNodeCoords()
  {
    for (auto& node : nodes_)
    {
      setLaneNodeCoords(node);
      setLaneNodeIdx(node);
    }
  }

  /**
   * Adds a point as a node to the lane graph
   * @param point
   */
  void addPoint(const Point<double>& point)
  {
    auto node = LaneNode(point);
    setLaneNodeCoords(node);
    setLaneNodeIdx(node);

    addNode(node);
  }

  /**
   * Calculates the two existing neighbors of a node based on proximity and angle to others
   * @param node
   * @return
   */
  void setNeighbors()
  {
    const double max_dist = 10;
    //    std::cout << "" << std::endl;
    //    std::cout << "Nb nodes " << nodes_.size() << std::endl;
    for (auto& node : nodes_)
    {
      //      std::cout << "Checking node " << node.index << std::endl;

      double closest_dist = std::numeric_limits<double>::max();
      double second_closest_dist = std::numeric_limits<double>::max();

      double angle2first = 0;
      double angle2second = 0;

      node.neighbor_1 = -1;
      node.neighbor_2 = -1;

      // find closest node
      for (const auto& next_node : nodes_)
      {
        // ignore yourself
        if (node.index == next_node.index)
        {
          continue;
        }

        // distance to next node
        const double dist = node.point_utm.dist2(next_node.point_utm);

        // ignore nodes far away
        if (dist > max_dist)
        {
          continue;
        }

        if (dist < closest_dist)
        {
          node.neighbor_1 = next_node.index;
          closest_dist = dist;
          const auto diff = next_node.point_utm - node.point_utm;
          angle2first = atan2(diff.y, diff.x);
        }
      }

      // find second-closest node
      for (const auto& next_node : nodes_)
      {
        // ignore yourself and closest node
        if (node.index == next_node.index or next_node.index == node.neighbor_1)
        {
          continue;
        }

        // distance to next node
        const double dist = node.point_utm.dist2(next_node.point_utm);

        // ignore nodes far away
        if (dist > max_dist)
        {
          continue;
        }

        if (dist < second_closest_dist)
        {
          const auto diff = next_node.point_utm - node.point_utm;
          angle2second = atan2(diff.y, diff.x);

          // points are not in the same direction
          if (util::getAngleDiff(angle2first, angle2second) > util::PI / 2)
          {
            node.neighbor_2 = next_node.index;
            second_closest_dist = dist;
          }
        }
      }
    }
  }

  [[nodiscard]] std::optional<LaneNode> findNode(int index2find) const
  {
    for (const auto& node : nodes_)
    {
      if (node.index == index2find)
      {
        return node;
      }
    }
    return {};
  }

  /**
   * Returns the edge consisting of 3 neighboring nodes found in @setNeighbors
   * @param node
   * @return
   */
  [[nodiscard]] edge_t getEdge(const LaneNode& node) const
  {
    std::optional<LaneNode> neighbor1 = {};
    std::optional<LaneNode> neighbor2 = {};

    int idx1 = node.neighbor_1;
    if (idx1 != -1)
    {
      if (auto found_node = findNode(idx1))
      {
        neighbor1 = found_node;
      }
      else
      {
        std::cout << "Error: Did not find node in Lane graph with index " << idx1 << std::endl;
      }
    }
    const int idx2 = node.neighbor_2;
    if (idx2 != -1)
    {
      if (auto found_node = findNode(idx2))
      {
        neighbor2 = found_node;
      }
      else
      {
        std::cout << "Error: Did not find node in Lane graph with index " << idx2 << std::endl;
      }
    }

    return { neighbor1, node, neighbor2 };
  }

  void setEdges()
  {
    edges_.clear();
    edges_.reserve(nodes_.size());

    for (auto& node : nodes_)
    {
      const auto edge = getEdge(node);

      edges_.push_back(std::move(edge));
    }
  }

private:
  /**
   * adds node to graph
   * @param node
   */
  void addNode(const LaneNode& node)
  {
    nodes_.push_back(node);
  }

  /**
   * Sets the index of a node on an existing patch
   * @param node
   */
  void setLaneNodeIdx(LaneNode& node) const
  {
    Point<double> disc_point = node.point_utm_patch / res;
    node.index = static_cast<int>(disc_point.x * patch_dim_ + disc_point.y);
  }

  /**
   * sets the utm coords on an existing patch
   * @param node
   */
  void setLaneNodeCoords(LaneNode& node) const
  {
    node.point_utm_patch = node.point_utm - patch_origin_utm_;
  }

public:
  edges_t edges_;
  std::vector<LaneNode> nodes_;

private:
  double res = 0.1;  // res for lane nodes
  Point<double> patch_origin_utm_ = { 0, 0 };
  double patch_dim_ = -1;
};

class Minipatch
{
public:
  Minipatch() = default;
  Minipatch(Vec2DFlat<uint8_t> patch,
            Point<double> origin,
            Point<double> center,
            int width,
            int repr_step,
            Point<int> patch_idx,
            double edge_length,
            Point<double> grid_reference,
            bool is_new)
    : patch_(std::move(patch))
    , origin_(origin)
    , center_(center)
    , width_(width)
    , repr_step_(repr_step)
    , patch_idx_(patch_idx)
    , edge_length_(edge_length)
    , grid_reference_(grid_reference)
    , is_new_(is_new){};

  Minipatch(const py::array_t<uint8_t>& local_map,
            Point<double> origin,
            Point<double> center,
            int width,
            int repr_step,
            Point<int> patch_idx,
            double edge_length,
            Point<double> grid_reference,
            bool is_new)
    : origin_(origin)
    , center_(center)
    , width_(width)
    , repr_step_(repr_step)
    , patch_idx_(patch_idx)
    , edge_length_(edge_length)
    , grid_reference_(grid_reference)
    , is_new_(is_new)
  {
    const auto* data = local_map.data();
    patch_.resize(width, width);
    std::memcpy(patch_.getPtr(), data, width * width * sizeof(uint8_t));
  };

  Vec2DFlat<uint8_t> patch_;
  Point<double> origin_;
  Point<double> center_;
  int width_;
  int repr_step_;
  Point<int> patch_idx_;
  double edge_length_;
  Point<double> grid_reference_;
  bool is_new_;
};

#endif  // FREESPACE_PLANNER_DATA_STRUCTURES2_HPP
