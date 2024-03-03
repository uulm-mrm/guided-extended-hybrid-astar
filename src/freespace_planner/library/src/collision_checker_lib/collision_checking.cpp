//
// Created by schumann on 11.05.22.
//
#include "collision_checker_lib/collision_checking.hpp"

#define USE_GPU_MASK_DILATE false

/**
 * Initialize the collision checking lib from the config file
 * @param path2config
 */
void CollisionChecker::initialize(size_t patch_dim, const std::string& path2config)
{
  YAML::Node config = YAML::LoadFile(path2config);

  yaw_res_coll_ = config["YAW_RES_COLL"].as<int>();
  yaw_res_coll_rad_ = yaw_res_coll_ * util::TO_RAD;
  nb_disc_yaws_ = 360 / yaw_res_coll_;
  safety_distance_m_ = config["SAFETY_DISTANCE_M"].as<double>();
  search_dist_ = config["SEARCH_DIST"].as<double>();
  search_dist_cells_ = search_dist_ * grid_tf::con2gm_;  // in cells
  min_thresh_ = config["MIN_THRESH"].as<int>();
  max_thresh_ = config["MAX_THRESH"].as<int>();
  len_per_disk_ = config["LEN_PER_DISK"].as<double>();
  double_disk_rows_ = config["DOUBLE_DISK_ROWS"].as<bool>();
  max_patch_ins_dist_ = config["MAX_PATCH_INS_DIST"].as<double>();

  resetPatch(patch_dim);

  calculateDisks();
}

/**
 * set patch_info dim and resize the patches
 * @param patch_dim
 */
void CollisionChecker::resetPatch(size_t patch_dim)
{
  patch_dim_ = patch_dim;
  patch_safety_arr_.resize_and_reset(patch_dim_, patch_dim_, UNKNOWN);
  patch_arr_.resize_and_reset(patch_dim_, patch_dim_, SENSOR_UNKNOWN);
}

double CollisionChecker::getDiskRadius(double length, double width, unsigned int nb_disks)
{
  return sqrt(((length * length) / (nb_disks * nb_disks) + (width * width)) / 4);
}

/**
 * Calculate disk positions to store them in a lookup table
 * @param radius
 * @param nb_disks
 * @param width
 * @param lb
 * @return
 */
std::vector<double> CollisionChecker::getDiskPositions(double radius, unsigned int nb_disks, double width, double lb)
{
  std::vector<double> disk_positions;
  disk_positions.reserve(nb_disks);
  const double dist = 2 * sqrt((radius * radius) - (width * width) / 4);  // half of this is the distance to lb

  const double first_pos = dist / 2 - lb;
  // circle positions beginning from rear axis
  for (int i = 0; i < nb_disks; i++)
  {
    disk_positions.push_back(first_pos + i * dist);
  }
  return disk_positions;
}

/**
 * Calculates the disk positions for the number of disks necessary for the lb stored in Vehicle
 */
void CollisionChecker::calculateDisks()
{
  // set number and width of vehicle
  const unsigned int eff_nb_disks = static_cast<int>(std::ceil(Vehicle::length_ / CollisionChecker::len_per_disk_));

  //  int disk_mult = 1;
  //  if (double_disk_rows_)
  //  {
  //    disk_mult = 2;
  //  }
  const int disk_mult = (double_disk_rows_) ? 2 : 1;

  nb_disks_ = disk_mult * eff_nb_disks;

  // set effective width
  const double eff_width = Vehicle::width_ / disk_mult;

  // Resize vectors to actual size
  disk_centers_.resize_and_reset(nb_disks_, nb_disc_yaws_, Point<int>(0, 0));
  disk_centers_.setName("disk_centers");

  // Calculate disk radius and position
  const double disk_radius = getDiskRadius(Vehicle::length_, eff_width, eff_nb_disks);
  const std::vector<double> disk_positions = getDiskPositions(disk_radius, eff_nb_disks, eff_width, Vehicle::lb_);

  // Prepare dilation filter
  const double safety_disk_radius = disk_radius + safety_distance_m_;
  disk_r_ = safety_disk_radius;
  disk_r_c_ = std::ceil(safety_disk_radius * grid_tf::con2gm_);
  disk_diameter_c_ = 2 * disk_r_c_ + 1;

  dil_kernel_ = getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(disk_diameter_c_, disk_diameter_c_), cv::Point(disk_r_c_, disk_r_c_));
  dilateFilter_ = cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, CV_8UC1, dil_kernel_);

  // Precalculate disk positions
  const unsigned int max_yaw_idx = (360 / yaw_res_coll_);
  double yaw = 0;
  for (int yaw_idx = 0; yaw_idx < max_yaw_idx; yaw_idx++)
  {
    yaw = yaw_idx * yaw_res_coll_rad_;

    int disk_idx = 0;
    for (const auto disk_pos : disk_positions)
    {
      double y_offset = 0;

      if (double_disk_rows_)
      {
        y_offset = eff_width / 2.0;
        const Point<double> disk_point1(disk_pos, y_offset);
        const Point<int> disk_point_rot1 = (disk_point1.rotate(yaw) * grid_tf::con2gm_).toInt();
        disk_centers_(yaw_idx, disk_idx) = disk_point_rot1;

        y_offset = -eff_width / 2.0;
        const Point<double> disk_point2 = { disk_pos, y_offset };
        const Point<int> disk_point_rot2 = (disk_point2.rotate(yaw) * grid_tf::con2gm_).toInt();
        disk_centers_(yaw_idx, disk_idx + 1) = disk_point_rot2;
        disk_idx += 2;
      }
      else
      {
        y_offset = 0;
        const Point<double> disk_point1(disk_pos, y_offset);
        const Point<int> disk_point_rot1 = (disk_point1.rotate(yaw) * grid_tf::con2gm_).toInt();
        disk_centers_(yaw_idx, disk_idx) = disk_point_rot1;
        disk_idx += 1;
      }
    }
  }
}

void CollisionChecker::insertMinipatches(const std::map<std::pair<int, int>, Minipatch>& minipatches,
                                         const Point<double>& ego_utm,
                                         bool only_nearest,
                                         bool only_new)
{
  for (const auto& [patch_idx, minipatch] : minipatches)
  {
    if (only_new and !minipatch.is_new_)
    {
      continue;
    }

    // TODO (Schumann) do this already on node to spare ressources
    if (only_nearest)
    {
      double dist2patch = minipatch.center_.dist2(ego_utm);
      if (dist2patch > max_patch_ins_dist_)
      {
        continue;
      }
    }

    // calculate origin on grm
    const auto origin_grid = grid_tf::utm2grid_round(grid_tf::utm2patch_utm(minipatch.origin_));

    CollisionChecker::passLocalMap(minipatch.patch_, origin_grid, minipatch.width_);
  }
}

void CollisionChecker::passLocalMap(const py::array_t<uint8_t>& local_map, const Point<int>& origin, int dim)
{
  passLocalMapData(local_map.data(), origin, dim);
}

/**
 * Passes a local map to the collision checker
 * @param local_map
 * @param origin
 * @param o_y
 * @param dim
 */
void CollisionChecker::passLocalMap(const Vec2DFlat<uint8_t>& local_map, const Point<int>& origin, int dim)
{
  passLocalMapData(local_map.data().data(), origin, dim);
}

void CollisionChecker::processSafetyPatch()
{
  /// CPU
  // input mat
  cv::Mat matImg(static_cast<int>(patch_dim_), static_cast<int>(patch_dim_), CV_8UC1, patch_arr_.getPtr());

  // output mat
  cv::Mat out_map(static_cast<int>(patch_dim_), static_cast<int>(patch_dim_), CV_8UC1, patch_safety_arr_.getPtr());

  if (not USE_GPU_MASK_DILATE)
  {
    /// CPU
    // data structs
    cv::Mat free_mask_gpu;
    cv::Mat inv_free_mask_gpu;
    cv::Mat occ_mask_gpu;
    cv::Mat inv_occ_mask_gpu;
    cv::Mat unknown_mask_gpu;

    // mask creation
    cv::compare(matImg, min_thresh_, free_mask_gpu, cv::CMP_LT);
    cv::compare(matImg, max_thresh_, occ_mask_gpu, cv::CMP_GT);
    cv::subtract(1, free_mask_gpu, inv_free_mask_gpu);
    cv::subtract(1, occ_mask_gpu, inv_occ_mask_gpu);
    cv::bitwise_and(inv_free_mask_gpu, inv_occ_mask_gpu, unknown_mask_gpu);

    out_map.setTo(FREE, free_mask_gpu);
    out_map.setTo(OCC, occ_mask_gpu);
    out_map.setTo(UNKNOWN, unknown_mask_gpu);

    cv::dilate(out_map, out_map, dil_kernel_);
  }
  else
  {
    try
    {
      /// GPU
      // data structs
      cv::cuda::GpuMat imgGpu;
      cv::cuda::GpuMat free_mask_gpu;
      cv::cuda::GpuMat inv_free_mask_gpu;
      cv::cuda::GpuMat occ_mask_gpu;
      cv::cuda::GpuMat inv_occ_mask_gpu;
      cv::cuda::GpuMat unknown_mask_gpu;
      imgGpu.upload(matImg);

      // mask creation
      cv::cuda::compare(imgGpu, min_thresh_, free_mask_gpu, cv::CMP_LT);
      cv::cuda::compare(imgGpu, max_thresh_, occ_mask_gpu, cv::CMP_GT);
      cv::cuda::subtract(1, free_mask_gpu, inv_free_mask_gpu);
      cv::cuda::subtract(1, occ_mask_gpu, inv_occ_mask_gpu);
      cv::cuda::bitwise_and(inv_free_mask_gpu, inv_occ_mask_gpu, unknown_mask_gpu);

      imgGpu.setTo(FREE, free_mask_gpu);
      imgGpu.setTo(OCC, occ_mask_gpu);
      imgGpu.setTo(UNKNOWN, unknown_mask_gpu);

      dilateFilter_->apply(imgGpu, imgGpu);
      imgGpu.download(out_map);
    }
    catch (...)
    {
      throw std::runtime_error(std::string("OpenCV inside the docker was not compiled for your graphics card! Activate "
                                           "the CPU-version of this code snippet by setting USE_GPU_MASK_DILATE to "
                                           "false in collision_checking.cpp:6"));
    }
  }
}

/**
 * Passes a local map to the collision checker
 * @param local_map
 * @param origin
 * @param o_y
 * @param dim
 */
void CollisionChecker::passLocalMapData(const unsigned char* local_map_data, const Point<int>& origin, int dim)
{
  // Boundary checking for x: rows must fit completely into patch
  // TODO insert partial, too!
  const int max_col_idx = origin.x + dim;
  const int x_start = 0;
  const int dim_eff = dim;
  //  if (origin.x < 0)
  //  {
  //    x_start = -origin.x;
  //    dim_eff -= x_start;
  //  }
  //  if (max_col_idx > patch_dim_)
  //  {
  //    dim_eff -= max_col_idx - patch_dim_;
  //    return;
  //  }

  if (origin.x < 0 or max_col_idx > patch_dim_)
  {
    return;
  }

  // Copy values of local map to internal patch
  const int size_data_patch = sizeof(uint8_t);
  for (size_t i = 0; i < dim; ++i)
  {
    // Boundary checking for y: only rows inside patch are copied completely
    const int row_index = origin.y + i;
    if (row_index > patch_dim_ - 1 or row_index < 0)
    {
      continue;
    }

    size_t dest_start_index = patch_dim_ * origin.y + i * patch_dim_ + origin.x + x_start;
    size_t src_start_index = i * dim_eff + x_start;
    //    std::memcpy(patch_safety_arr_.getPtr() + dest_start_index * size_data_safety,
    //                matImg.data + src_start_index * size_data_safety,
    //                dim_eff * size_data_safety);

    std::memcpy(patch_arr_.getPtr() + dest_start_index * size_data_patch,
                local_map_data + src_start_index * size_data_patch,
                dim_eff * size_data_patch);
  }
}

/**
 * Return center points of disk positions given the specified yaw pos in meters for visualization
 * @param yaw
 * @return
 */
std::vector<Point<double>> CollisionChecker::returnDiskPositions(double yaw)
{
  std::vector<Point<double>> points;
  points.reserve(nb_disks_);
  const int yaw_idx = getYawIdx(yaw);

  for (int disk_idx = 0; disk_idx < nb_disks_; ++disk_idx)
  {
    points.push_back(grid_tf::grid2utm(disk_centers_(yaw_idx, disk_idx).toDouble()));
  }
  return points;
}

int CollisionChecker::getYawIdx(double yaw)
{
  const int yaw_idx = static_cast<int>(util::constrainAngleZero2Pi(yaw) / yaw_res_coll_rad_);
  return std::min(yaw_idx, static_cast<int>(nb_disc_yaws_ - 1));
}

/**
 * Check a single position for collision
 * @param pose
 * @return
 */
bool CollisionChecker::checkGrid(const Pose<int>& pose)
{
  for (int disk_idx = 0; disk_idx < nb_disks_; ++disk_idx)
  {
    const Point<int> point = disk_centers_(pose.yaw, disk_idx) + Point<int>(pose.x, pose.y);

    // out of map
    if (point.x >= patch_dim_ || point.y >= patch_dim_ || point.x < 0 || point.y < 0)
    {
      return false;  // not valid
    }
    // collides with grid
    if (patch_safety_arr_(point) == OCC)
    {
      return false;  // collision
    }
  }
  return true;  // no collision
}

/**
 * Check a single position for collision, out of map coords are valid, not colliding as in the other one
 * @param pose
 * @return
 */
bool CollisionChecker::checkGridVariant(const Pose<int>& pose)
{
  for (int disk_idx = 0; disk_idx < nb_disks_; ++disk_idx)
  {
    const Point<int> point = disk_centers_(pose.yaw, disk_idx) + Point<int>(pose.x, pose.y);

    // out of map
    if (point.x >= patch_dim_ || point.y >= patch_dim_ || point.x < 0 || point.y < 0)
    {
      continue;  // Ignore
    }
    // collides with grid
    if (patch_safety_arr_(point) == OCC)
    {
      return false;  // collision
    }
  }
  return true;  // no collision
}

bool CollisionChecker::checkPose(const Pose<double>& pose)
{
  // Position on grid
  const Pose<int> pose2check(
      static_cast<int>(pose.x * grid_tf::con2gm_), static_cast<int>(pose.y * grid_tf::con2gm_), getYawIdx(pose.yaw));

  return checkGrid(pose2check);
}
/**
 * Variant that does return True on out of patch
 * @param pose
 * @return
 */
bool CollisionChecker::checkPoseVariant(const Pose<double>& pose)
{
  // Position on grid
  const Pose<int> pose2check(
      static_cast<int>(pose.x * grid_tf::con2gm_), static_cast<int>(pose.y * grid_tf::con2gm_), getYawIdx(pose.yaw));

  return checkGridVariant(pose2check);
}

/**
 * Check for collisions of path
 * @param x_list
 * @param y_list
 * @param yaw_list
 * @return
 */
bool CollisionChecker::checkPathCollision(const std::vector<double>& x_list,
                                          const std::vector<double>& y_list,
                                          const std::vector<double>& yaw_list)
{
  //  for (int i = static_cast<int>(x_list.size()) - 1; i >= 0; --i)  // start from the beginning
  for (size_t i = 0, max = x_list.size(); i < max; ++i)
  {
    if (!checkPose({ x_list[i], y_list[i], yaw_list[i] }))
    {
      return false;  // subpath collides
    }
  }
  return true;  // subpath is valid
}

/**
 * Check path for collisions and return the index of collision
 * @param x_list
 * @param y_list
 * @param yaw_list
 * @return
 */
int CollisionChecker::getPathCollisionIndex(const std::vector<double>& x_list,
                                            const std::vector<double>& y_list,
                                            const std::vector<double>& yaw_list)
{
  for (size_t i = 0, max = x_list.size(); i < max; ++i)
  {
    if (!checkPose({ x_list[i], y_list[i], yaw_list[i] }))
    {
      return static_cast<int>(i);  // subpath collides
    }
  }
  return -1;  // subpath is valid
}