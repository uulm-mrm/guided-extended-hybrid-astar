#!/usr/bin/env python3
import os
import sys
import numpy as np
from collections import deque
import threading
import time
from typing import Optional
from numpy.typing import NDArray

from path_planner_lib.planning import AStar, HybridAStar, CollisionChecker, Smoother
from path_planner_lib import util
from path_planner_lib.planning import PathPlanning, UtilCpp
from path_planner_lib.data_structures import GoalMsg, VehConfig, OccEnum, Path, PoseDouble, PointInt, PointDouble, \
    Minipatch
from gridmap_sim_lib import GridMapSim
import sim_util as sutil
from ament_index_python.packages import get_package_share_directory

CI_RUN = False
if os.getenv("CI_PROJECT_NAME") is not None:
    CI_RUN = True


class Sim:
    def __init__(self, ushift_override: bool = False):
        self.speed: float = 0.0
        node_name = "freespace_planner"
        self.share_dir = get_package_share_directory(node_name)

        string2find = "colcon_build/install"
        string_len = len(string2find)
        install_idx = self.share_dir.find(string2find) + string_len
        install_dir = self.share_dir[:install_idx]
        rel_share_path = self.share_dir[install_idx:]
        rel_lib_share_path = rel_share_path.replace(
            node_name, node_name + "_lib")
        lib_share_dir = install_dir + rel_lib_share_path

        # get library config and params
        lib_config: dict = util.load_lib_config(lib_share_dir)
        lib_params: dict = util.load_lib_params(lib_share_dir)

        # simulator structs
        self.vis: Optional = None
        self.ego_utm_global: PoseDouble = PoseDouble(0, 0, 0)
        self.goal_msg_queue: deque = deque(maxlen=1)
        self.pause: bool = False
        sim_config: dict = sutil.load_sim_config(self.share_dir)
        self.FILE_NAME = sim_config["SCENARIO_NAME"]

        # gm sim
        self.GM_DIM: int = 801
        self.ALL_VISIBLE = sim_config["ALL_VISIBLE"]
        # Simulate that only the border is visible
        pad_dim = 800
        self.global_map_gt = sutil.load_png(self.FILE_NAME)
        self.global_map_gt = sutil.pad_map(self.global_map_gt, pad_dim)
        if self.ALL_VISIBLE:
            self.global_map_gt = sutil.transform_binary_to_gm(
                self.global_map_gt)

        # planning params
        self.IS_USHIFT = lib_config["IS_USHIFT"] or ushift_override
        self.T: float = 0.1

        # Create planners
        veh_config = VehConfig(is_ushift=self.IS_USHIFT)
        self.plan = PathPlanning(veh_config, lib_share_dir)
        self.plan.set_sim(True)

        # Set params, usually set by ros params
        AStar.alpha_ = lib_params["alpha_"]
        AStar.do_max_ = lib_params["do_max_"]
        AStar.do_min_ = lib_params["do_min_"]
        AStar.astar_prox_cost_ = lib_params["astar_prox_cost_"]
        AStar.astar_movement_cost_ = lib_params["astar_movement_cost_"]
        AStar.astar_lane_movement_cost_ = lib_params["astar_lane_movement_cost_"]

        HybridAStar.steer_change_cost_ = lib_params["steer_change_cost_"]
        HybridAStar.steer_cost_ = lib_params["steer_cost_"]
        HybridAStar.back_cost_ = lib_params["back_cost_"]
        HybridAStar.h_prox_cost_ = lib_params["h_prox_cost_"]
        HybridAStar.switch_cost_ = lib_params["switch_cost_"]
        HybridAStar.h_dist_cost_ = lib_params["h_dist_cost_"]

        Smoother.max_iter_ = lib_params["max_iter_"]
        Smoother.wSmoothness_ = lib_params["wSmoothness_"]
        Smoother.wObstacle_ = lib_params["wObstacle_"]
        Smoother.wCurvature_ = lib_params["wCurvature_"]
        Smoother.alpha_opt_ = lib_params["alpha_opt_"]

    def get_minipatch(self, local_map: NDArray):
        ego_point: PointDouble = self.ego_utm_global.getPoint()
        dim: int = local_map.shape[0]
        origin = ego_point - (dim / 2) * HybridAStar.GM_RES
        patch_center: PointInt = origin + dim / 2
        return Minipatch(local_map, origin, patch_center, dim, 0, PointInt(0, 0), 0.0, PointDouble(0, 0), True)

    def run_simulation(self) -> None:

        # Start simulation
        # Allow vis to start
        time.sleep(1)

        # don't import earlier to ensure that imviz is imported in actual thread that is using it
        if not CI_RUN:
            import imviz as viz

        ci_timeout = 120
        ci_start_time = time.perf_counter()

        while True:

            if not CI_RUN:
                # Allow automatic reloading of python code - this is spooky ;)
                viz.update_autoreload()

            if self.pause:
                continue

            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # Lidar data simulation
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            local_map = self.simulate_gm_data(self.ego_utm_global)

            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # Planning
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            goal_message = None
            if self.goal_msg_queue:
                goal_message = self.goal_msg_queue.popleft()

            time_seconds = time.time()

            # Transform local map to iterable of minipatches for compatability with ros version
            minipatch = self.get_minipatch(local_map)

            path_utm, new_path = self.plan.do_planning(self.ego_utm_global, self.speed, goal_message,
                                                       minipatch, time_seconds)

            self.move_ego_vehicle_on_path(path_utm)

            self.plan.sim_idx += 1

            if CI_RUN:
                current_time = time.perf_counter()
                if (current_time - ci_start_time) > ci_timeout:
                    raise Exception(
                        "Did not reach goal on timeout. Raising Exception!")

                if self.plan.was_goal_just_reached():
                    # Exit and save png
                    print("Goal reached in sim, creating result plot")
                    self.save_final_output()
                    return

    def save_final_output(self):
        """
        Save driven path in grid map
        :return:
        """
        # ct stores current time
        import matplotlib.pyplot as plt

        # get data
        path_x, path_y = UtilCpp.utm2global_gm(
            self.plan.driven_path.x_list, self.plan.driven_path.y_list)
        arr = (OccEnum.OCC - CollisionChecker.patch_arr_.getNumpyArr())
        offset = UtilCpp.utm2grid(self.plan.patch_info.origin_utm)
        dim = self.plan.patch_info.dim

        # calculate window boundaries
        margin = 10
        lower_x = min(path_x) - margin
        upper_x = max(path_x) + margin
        lower_y = min(path_y) - margin
        upper_y = max(path_y) + margin
        x_dim = upper_x - lower_x
        y_dim = upper_y - lower_y
        max_dim = max(x_dim, y_dim)
        x_center = lower_x + x_dim/2
        y_center = lower_y + y_dim/2
        lower_x = x_center - max_dim/2
        upper_x = x_center + max_dim/2
        lower_y = y_center - max_dim/2
        upper_y = y_center + max_dim/2

        # create figure
        plt.figure(1)
        plt.xlim([lower_x, upper_x])
        plt.ylim([lower_y, upper_y])
        if self.IS_USHIFT:
            plt.title("Path with U-Shift")
        else:
            plt.title("Path with standard vehicle")
        plt.xlabel("x in m")
        plt.ylabel("y in m")
        plt.imshow(arr, origin="lower", cmap="gray", extent=[
            offset.x, offset.x + dim, offset.y, offset.y + dim])
        plt.plot(path_x, path_y)

        # save
        script_dir = os.path.dirname(os.path.realpath(__file__))
        filename_latest = script_dir + \
            "/sim_data/ci_artefacts/ci_test_latest_ushift_" + \
            str(self.IS_USHIFT) + ".png"
        plt.savefig(filename_latest)

    def move_ego_vehicle_on_path(self, path_utm: Path) -> None:
        """
        Simulate exact movement of ego vehicle on path
        :return:
        """
        if path_utm is not None:
            path_x_in_front = path_utm.x_list[self.plan.ego_idx:]
            path_y_in_front = path_utm.y_list[self.plan.ego_idx:]
            path_yaw_in_front = path_utm.yaw_list[self.plan.ego_idx:]

            if len(path_x_in_front) == 1:
                self.speed = 0.0
                return

            if len(path_x_in_front) > 1:
                # Simulate movement of vehicle on path
                step_width: int = 1
                max_idx: int = len(path_x_in_front) - 1
                next_idx = min(step_width, max_idx)

                if path_utm.direction_list[self.plan.ego_idx] < 0:
                    self.speed = -5.0
                else:
                    self.speed = 5.0

                next_pose: PoseDouble = PoseDouble(path_x_in_front[next_idx],
                                                   path_y_in_front[next_idx],
                                                   path_yaw_in_front[next_idx])

                self.ego_utm_global = next_pose.copy()

    def get_preset_goal(self) -> tuple[PoseDouble, PoseDouble]:
        if self.FILE_NAME in ["Valet_parking_015", "Free_015"]:
            # Driving from bottom to top
            start_pose = PoseDouble(165.734262, 128.0, 0.0)
            goal_pose = PoseDouble(161.325933, 161.124533, 90.0)

        elif self.FILE_NAME == "neu-ulm_gm":
            # measurements
            start_pose = PoseDouble(513.508328, 579.258298, 100)
            goal_pose = PoseDouble(501.480847, 625.956814, 133)

        elif self.FILE_NAME == "Maze":
            start_pose = PoseDouble(130, 130, 0)
            goal_pose = PoseDouble(181, 226, 0)
        elif self.FILE_NAME == "depot":
            start_pose = PoseDouble(200, 200, 0)
            goal_pose = start_pose
        else:
            raise Exception("Unknown filename, abort")

        # Set angle to yaw
        start_pose.yaw = np.deg2rad(start_pose.yaw)
        goal_pose.yaw = np.deg2rad(goal_pose.yaw)

        return start_pose, goal_pose

    def simulate_gm_data(self, ego_utm_global: PoseDouble) -> NDArray:
        """
        Given a grid map, this function simulates the results of an occupancy grid map with its corresponding LIDAR
        in the middle of the map
        """
        # Turn continuous start coords into global grid map coords
        ego_gm_global = UtilCpp.utm2grid_round(ego_utm_global)

        # Get grid map around ego position
        grid_map_local_gt = util.get_local_map(
            ego_gm_global, self.global_map_gt, self.GM_DIM)

        if self.ALL_VISIBLE:
            return grid_map_local_gt

        return GridMapSim.gmSimulation(grid_map_local_gt)

    def viz_loop(self) -> None:
        from path_planner_lib.visualization import Vis

        self.vis = Vis()

        while True:
            if self.plan is not None:
                goal_message, self.pause, action_obj = self.vis.render_data(
                    self.plan)

                if goal_message is not None:
                    self.goal_msg_queue.append(goal_message)

    def simulate(self) -> None:
        if not CI_RUN:
            vis_thread = threading.Thread(
                target=self.viz_loop,
                args=(),
                daemon=True)
            vis_thread.start()

        # Get start and goal
        start_pose, goal_pose = self.get_preset_goal()

        # Add to goal queue
        goal_msg = GoalMsg(goal_pose, remove=False)
        self.goal_msg_queue.append(goal_msg)

        # Set ego position
        self.ego_utm_global = start_pose.copy()

        # execute sim
        self.run_simulation()


if __name__ == "__main__":

    ushift_override: bool = False
    if sys.argv:
        if len(sys.argv) == 2:
            if sys.argv[1]:
                param = sys.argv[1]
                if str(param) == "IS_USHIFT":
                    ushift_override = True

    sim = Sim(ushift_override)
    sim.simulate()
