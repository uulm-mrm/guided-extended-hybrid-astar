//
// Created by Schumann on 12/9/22.
//

#include "hybridastar_planning_lib/smoother.hpp"

void Smoother::init()
{
  // Load config
  YAML::Node config = YAML::LoadFile(AStar::path2config_);

  is_initialized_ = true;
  kappaMax_ = Vehicle::max_curvature_;
}

/**
 * Simple gradient descent for x iterations
 * @param newPath
 * @param collision_indices
 */
void Smoother::optimize_gd(Path& newPath, const std::vector<int>& collision_indices)
{
  const size_t path_length = newPath.x_list.size();

  if (path_length < 2 * kignore_indices)
  {
    return;
  }

  // Gradient descent begin
  unsigned int iterations = 0;
  const double totalWeight = wSmoothness_ + wObstacle_ + wCurvature_;
  while (iterations < max_iter_)
  {
    // Ignore the first nodes of the path, and some points around
    const size_t start_idx = kignore_indices;
    const size_t max_idx = path_length - kignore_indices;
    for (size_t i = start_idx; i < max_idx; ++i)
    {
      if (newPath.types[i - 3] == PATH_TYPE::REAR_AXIS || newPath.types[i - 2] == PATH_TYPE::REAR_AXIS ||
          newPath.types[i - 1] == PATH_TYPE::REAR_AXIS || newPath.types[i] == PATH_TYPE::REAR_AXIS ||
          newPath.types[i + 1] == PATH_TYPE::REAR_AXIS || newPath.types[i + 2] == PATH_TYPE::REAR_AXIS ||
          newPath.types[i + 3] == PATH_TYPE::REAR_AXIS)
      {
        continue;
      }

      if (std::find(collision_indices.begin(), collision_indices.end(), i) != collision_indices.end())
      {
        // Point collided previously
        continue;
      }

      const Point<double> xim2(newPath.x_list[i - 2], newPath.y_list[i - 2]);
      const Point<double> xim1(newPath.x_list[i - 1], newPath.y_list[i - 1]);
      Point<double> xi0(newPath.x_list[i], newPath.y_list[i]);
      const Point<double> xip1(newPath.x_list[i + 1], newPath.y_list[i + 1]);
      const Point<double> xip2(newPath.x_list[i + 2], newPath.y_list[i + 2]);

      // don't smooth over direction changes
      if (newPath.direction_list[i + 3] != newPath.direction_list[i + 2] ||
          newPath.direction_list[i + 2] != newPath.direction_list[i + 1] ||
          newPath.direction_list[i + 1] != newPath.direction_list[i] ||
          newPath.direction_list[i] != newPath.direction_list[i - 1] ||
          newPath.direction_list[i - 1] != newPath.direction_list[i - 2] ||
          newPath.direction_list[i - 2] != newPath.direction_list[i - 3])
      {
        continue;
      }

      // don't smooth over equal points and some points around. Is only a catch if RA points were not discovered
      if (xip2.equal(xip1) || xip1.equal(xi0) || xi0.equal(xim1) || xim1.equal(xim2))
      {
        //        LOG_INF("Rear axis point detected, ignoring");
        //        LOG_INF("Point is " << xi0);
        continue;
      }

      // check if point is out of bounds
      const auto [dim_x, dim_y] = AStar::obs_x_grad_.getDims();
      if (xi0.x < kborder_lim || xi0.y < kborder_lim || xi0.x > dim_x + kborder_lim || xi0.y > dim_y + kborder_lim)
      {
        continue;
      }

      Point<double> gradient_vector = { 0, 0 };

      //      gradient_vector = gradient_vector - obsTerm(xi0, xip1);

      gradient_vector = gradient_vector - smoothnessTerm(xim2, xim1, xi0, xip1, xip2);

      gradient_vector = gradient_vector - curvatureTerm(xim2, xim1, xi0, xip1, xip2);

      // Update coordinate
      Point<double> step = alpha_opt_ * gradient_vector / totalWeight;
      // clamp gradients
      step = Point<double>(std::clamp(step.x, -1.0, 1.0), std::clamp(step.y, -1.0, 1.0));
      xi0 = xi0 + step;

      // recalculate yaw
      newPath.x_list[i] = xi0.getX();
      newPath.y_list[i] = xi0.getY();
      const Point<double> diff_point = xi0 - xim1;
      double yaw = std::atan2(diff_point.getY(), diff_point.getX());
      if (newPath.direction_list[i] != 1)
      {
        yaw += util::PI;
      }
      newPath.yaw_list[i] = yaw;
    }
    iterations++;
  }
}

/**
 * Start function for path smoothing.
 * Here, the anchoring of the original paper is implemented
 * @param path
 * @return
 */
void Smoother::smooth_path(Path& path)
{
  if (!is_initialized_)
  {
    init();
  }

  // copy new path to enable it to switch back to new one after smoothing
  Path prevPath = Path(path);

  int coll_idx = -1;
  std::vector<int> collision_indices;
  int coll_try_idx = 0;
  // Loop to anchor colliding points to original points
  do
  {
    Smoother::optimize_gd(path, collision_indices);
    coll_idx = CollisionChecker::getPathCollisionIndex(path.x_list, path.y_list, path.yaw_list);
    if (coll_idx != -1)
    {
      //      LOG_INF("Resetting coordinate, found collision at " << coll_idx);
      collision_indices.push_back(coll_idx);
      // reset path with previous one
      //      path = Path(prevPath);
      path.x_list[coll_idx] = prevPath.x_list[coll_idx];
      path.y_list[coll_idx] = prevPath.y_list[coll_idx];
      path.yaw_list[coll_idx] = prevPath.yaw_list[coll_idx];
      coll_try_idx++;
      if (coll_try_idx > kmax_coll_tries)
      {
        //        LOG_WARN("Path still collides, anchoring failed. Resetting to original one");
        path = Path(prevPath);
        coll_idx = -1;
      }
    }
  } while (coll_idx != -1);
}

/**
 * Penalizes the proximity to obstacles
 * @param xi0
 * @param xip
 * @return
 */
// Point<double> Smoother::obsTerm(const Point<double>& xi0, const Point<double>& xip)
//{
//   // Take value at geometric center
//   //  LOG_INF("xi0 " << xi0);
//   const Point<double> diff = xip - xi0;
//   const double yaw = std::atan2(diff.getY(), diff.getX());
//   const double rot_geo_center_x = Vehicle::geo_center_ * cos(yaw);
//   const double rot_geo_center_y = Vehicle::geo_center_ * sin(yaw);
//   // Do bilinear interpolation of gradients
//   const double x_val = (xi0.getX() + rot_geo_center_x) * grid_tf::con2star_;
//   const double y_val = (xi0.getY() + rot_geo_center_y) * grid_tf::con2star_;
//   const double x_grad = util::getBilinInterp(x_val, y_val, AStar::obs_x_grad_);
//   const double y_grad = util::getBilinInterp(x_val, y_val, AStar::obs_y_grad_);
//   const Point<double> grad(x_grad, y_grad);
//
//   // If there is no gradient
//   if (grad.length() == 0)
//   {
//     return { 0, 0 };
//   }
//
//   // Project grad orthogonal onto point
//   const double rad90 = 90.0 / 180 * util::PI;
//   const double delta_phi = rad90 + std::acos(std::clamp(diff.dot(grad) / (diff.length() * grad.length()),
//   -1.0, 1.0)); const Point<double> rot(-sin(delta_phi), cos(delta_phi)); const Point<double> grad_orth =
//   (rot.dot(grad)) * rot;
//
//   return -wObstacle_ * grad_orth;
// }

/**
 * Penalizes if two consecutive vectors differ
 * @param xim2
 * @param xim1
 * @param xi0
 * @param xip1
 * @param xip2
 * @return
 */
Point<double> Smoother::smoothnessTerm(const Point<double>& xim2,
                                       const Point<double>& xim1,
                                       const Point<double>& xi0,
                                       const Point<double>& xip1,
                                       const Point<double>& xip2)
{
  return wSmoothness_ * (xim2 - 4 * xim1 + 6 * xi0 - 4 * xip1 + xip2);
}

/**
 * Penalizes points if curvature exceeds maximum curvature
 * @param x_im2
 * @param x_im1
 * @param x_i
 * @param x_ip1
 * @param x_ip2
 * @return
 */
Point<double> Smoother::curvatureTerm(const Point<double>& x_im2,
                                      const Point<double>& x_im1,
                                      const Point<double>& x_i,
                                      const Point<double>& x_ip1,
                                      const Point<double>& x_ip2)
{
  // the vectors between the nodes
  const Point<double>& delta_x_im1 = x_im1 - x_im2;
  const Point<double>& delta_x_i = x_i - x_im1;
  const Point<double>& delta_x_ip1 = x_ip1 - x_i;
  const Point<double>& delta_x_ip2 = x_ip2 - x_ip1;

  // ensure that the absolute values are not null
  if (delta_x_im1.length() == 0 || delta_x_i.length() == 0 || delta_x_ip1.length() == 0 || delta_x_ip2.length() == 0)
  {
    // return gradient of 0
    //    std::cout << "abs values not larger than 0" << std::endl;
    return { 0, 0 };
  }
  // the angular change at the node
  auto compute_kappa =
      [](const Point<double>& delta_x_0, const Point<double>& delta_x_1, double& delta_phi, double& kappa) {
        delta_phi =
            std::acos(std::clamp(delta_x_0.dot(delta_x_1) / (delta_x_0.length() * delta_x_1.length()), -1.0, 1.0));
        kappa = delta_phi / delta_x_0.length();
      };  // Equation (2)

  double delta_phi_im1;
  double kappa_im1;
  compute_kappa(delta_x_im1, delta_x_i, delta_phi_im1, kappa_im1);
  double delta_phi_i;
  double kappa_i;
  compute_kappa(delta_x_i, delta_x_ip1, delta_phi_i, kappa_i);
  double delta_phi_ip1;
  double kappa_ip1;
  compute_kappa(delta_x_ip1, delta_x_ip2, delta_phi_ip1, kappa_ip1);

  // if the curvature is smaller than the maximum do nothing
  if (kappa_i <= kappaMax_)
  {
    return { 0, 0 };
  }
  auto compute_d_delta_phi = [](const double delta_phi) {
    return -1. / std::sqrt(1. - std::pow(std::cos(delta_phi), 2));
  };  // where...

  // Equation between (2) and (3)
  //    std::cout << "Kappa is " << kappa_i << " max is " << kappaMax_ << std::endl;
  const double& d_delta_phi_im1 = compute_d_delta_phi(delta_phi_im1);
  const Point<double>& d_cos_delta_phi_im1 = delta_x_im1.ort(delta_x_i) / (delta_x_im1.length() * delta_x_i.length());
  const Point<double>& d_kappa_im1 = 1. / delta_x_im1.length() * d_delta_phi_im1 * d_cos_delta_phi_im1;
  const Point<double>& kim1 = 2. * (kappa_im1 - kappaMax_) * d_kappa_im1;

  const double& d_delta_phi_i = compute_d_delta_phi(delta_phi_i);
  const Point<double>& d_cos_delta_phi_i = delta_x_ip1.ort(delta_x_i) / (delta_x_ip1.length() * delta_x_i.length()) -
                                           delta_x_i.ort(delta_x_ip1) / (delta_x_i.length() * delta_x_ip1.length());
  const Point<double>& d_kappa_i = 1. / delta_x_i.length() * d_delta_phi_i * d_cos_delta_phi_i -
                                   delta_phi_i / std::pow(delta_x_i.length(), 3) * delta_x_i;
  const Point<double>& ki0 = 2. * (kappa_i - kappaMax_) * d_kappa_i;

  const double& d_delta_phi_ip1 = compute_d_delta_phi(delta_phi_ip1);
  const Point<double>& d_cos_delta_phi_ip1 =
      -delta_x_ip2.ort(delta_x_ip1) / (delta_x_ip2.length() * delta_x_ip1.length());
  const Point<double>& d_kappa_ip1 = 1. / delta_x_ip1.length() * d_delta_phi_ip1 * d_cos_delta_phi_ip1 +
                                     delta_phi_ip1 / std::pow(delta_x_ip1.length(), 3) * delta_x_ip1;
  const Point<double>& kip1 = 2. * (kappa_ip1 - kappaMax_) * d_kappa_ip1;

  const Point<double> gradient = wCurvature_ * (0.25 * kim1 + 0.5 * ki0 + 0.25 * kip1);

  if (std::isnan(gradient.getX()) || std::isnan(gradient.getY()))
  {
    //    std::cout << "nan values in curvature term" << std::endl;
    return { 0, 0 };
  }
  return gradient;
}