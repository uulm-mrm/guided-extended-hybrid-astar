//
// Created by schumann on 3/14/23.
//

#include "gtest/gtest.h"
#include "collision_checker_lib/collision_checking.hpp"
#include "collision_checker_lib/vehicle.hpp"

TEST(CollisionChecker, checkGrid)
{
  Vehicle::initialize(0.6, 2.7, 3.0, 1.5, 2.0, false);

  constexpr size_t patch_dim = 1000;
  const std::string path2config = "/home/schumann/mrm/projects/sandboxes/aduulm_sandbox/colcon_build/install/"
                                  "freespace_planner_lib/share/freespace_planner_lib/config/config.yml";
  CollisionChecker::initialize(patch_dim, path2config);
  bool result;

  Pose<int> test_pose = { 500, 500, 0 };
  result = CollisionChecker::checkGrid(test_pose);
  EXPECT_TRUE(result);

  // center is inside but disks are not -> must fail
  test_pose = { 0, 0, 0 };
  result = CollisionChecker::checkGrid(test_pose);
  EXPECT_FALSE(result);

  test_pose = { patch_dim - 1, patch_dim - 1, 0 };
  result = CollisionChecker::checkGrid(test_pose);
  EXPECT_FALSE(result);
}