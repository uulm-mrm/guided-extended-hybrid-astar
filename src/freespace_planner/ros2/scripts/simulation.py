#!/usr/bin/env python3
import os
import math
import numpy as np
from collections import deque
import threading
import time
from typing import Optional
from numpy.typing import NDArray

from path_planner_lib.planning import AStar, HybridAStar, CollisionChecker
from path_planner_lib import util
from path_planner_lib.planning import PathPlanning, UtilCpp
from path_planner_lib.data_structures import GoalMsg, VehConfig, OccEnum, Path, PoseDouble, PointInt, PointDouble, Minipatch
from gridmap_sim_lib import GridMapSim
import sim_util as sutil


CI_RUN = False
if os.getenv("CI_PROJECT_NAME") is not None:
    CI_RUN = True


class Sim:
    def __init__(self):
        share_dir = "/opt/guided-extended-hybrid-astar/src/freespace_planner/ros2/scripts"
        lib_share_dir = "/opt/guided-extended-hybrid-astar/src/freespace_planner/library"
        lib_config = util.load_lib_config(lib_share_dir)

        # simulator structs
        self.vis: Optional = None
        self.ego_utm_global: PoseDouble = PoseDouble(0, 0, 0)
        self.goal_msg_queue: deque = deque(maxlen=1)
        self.pause: bool = False
        sim_config: dict = sutil.load_sim_config(share_dir)
        self.FILE_NAME = sim_config["SCENARIO_NAME"]

        # gm sim
        self.GM_DIM = lib_config["GM_DIM"]
        self.GM_RES = lib_config["GM_RES"]
        self.ALL_VISIBLE = sim_config["ALL_VISIBLE"]

        # Get simulation map
        pad_dim = 800
        self.global_map_gt = sutil.load_png(self.FILE_NAME, share_dir)
        self.global_map_gt = sutil.pad_map(self.global_map_gt, pad_dim)
        if self.ALL_VISIBLE:
            self.global_map_gt = sutil.transform_binary_to_gm(
                self.global_map_gt)

        # planning params
        self.IS_USHIFT = lib_config["IS_USHIFT"]
        self.T: float = 0.1

        # Create planner framework
        veh_config = VehConfig(is_ushift=self.IS_USHIFT)
        self.plan = PathPlanning(veh_config, lib_share_dir)
        self.plan.set_sim(True)

        # Set params, usually set by ros params
        AStar.alpha_ = lib_config["alpha_"]
        AStar.do_max_ = lib_config["do_max_"]
        AStar.do_min_ = lib_config["do_min_"]
        AStar.astar_prox_cost_ = lib_config["astar_prox_cost_"]
        AStar.astar_movement_cost_ = lib_config["astar_movement_cost_"]
        AStar.astar_lane_movement_cost_ = lib_config["astar_lane_movement_cost_"]

        HybridAStar.steer_change_cost_ = lib_config["steer_change_cost_"]
        HybridAStar.steer_cost_ = lib_config["steer_cost_"]
        HybridAStar.back_cost_ = lib_config["back_cost_"]
        HybridAStar.h_prox_cost_ = lib_config["h_prox_cost_"]
        HybridAStar.switch_cost_ = lib_config["switch_cost_"]
        HybridAStar.h_dist_cost_ = lib_config["h_dist_cost_"]

    def get_speed(self, ego_utm_global: PoseDouble) -> float:
        # Calculate speed
        dx = ego_utm_global.x - self.ego_utm_global.x
        dy = ego_utm_global.y - self.ego_utm_global.y
        speed = abs(np.linalg.norm([dx, dy]) / self.T)

        speed_angle = np.arctan2(dy, dx)
        diff = util.angle_diff(speed_angle, ego_utm_global.yaw)
        if abs(diff) > math.pi / 2:
            speed *= -1
        return speed

    def get_minipatch(self, local_map: NDArray):
        ego_point: PointDouble = self.ego_utm_global.getPoint()
        origin = ego_point - (self.GM_DIM / 2) * self.GM_RES
        edge_length = local_map.shape[0]
        patch_center: PointInt = origin + edge_length / 2
        return Minipatch(local_map, origin, patch_center, self.GM_DIM, 0, PointInt(0, 0), 0.0, PointDouble(0, 0), True)

    def run_simulation(self) -> None:

        # Start simulation
        toggle_container = False

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

            speed = self.get_speed(self.ego_utm_global)

            # Transform local map to iterable of minipatches for compatability with ros version
            minipatch = self.get_minipatch(local_map)

            path_utm, path_id = self.plan.do_planning(self.ego_utm_global, speed, goal_message, toggle_container,
                                                      minipatch, time_seconds)

            self.move_ego_vehicle_on_path(path_utm)
            self.plan.sim_idx += 1

            if CI_RUN:
                current_time = time.perf_counter()
                if (current_time-ci_start_time) > ci_timeout:
                    raise Exception(
                        "Did not reach goal on timeout. Raising Exception!")

                if self.plan.was_goal_just_reached():
                    # Exit and save png
                    print("Goal reached in sim")
                    self.save_final_output()
                    return

    def save_final_output(self):
        # ct stores current time
        import matplotlib.pyplot as plt
        import datetime

        ct = datetime.datetime.now()
        ct = ct.strftime("%d-%m-%y_%h-%m-%s")
        path_x, path_y = UtilCpp.utm2global_gm(
            self.plan.driven_path.x_list, self.plan.driven_path.y_list)
        arr = (OccEnum.OCC - CollisionChecker.patch_arr_.getNumpyArr())
        offset = UtilCpp.utm2grid(self.plan.patch_info.origin_utm)
        dim = self.plan.patch_info.dim

        plt.figure(1)
        plt.imshow(arr, origin="lower", cmap="gray", extent=[
                   offset.x, offset.x + dim, offset.y, offset.y + dim])
        plt.plot(path_x, path_y)

        script_dir = os.path.dirname(os.path.realpath(__file__))
        filename_latest = script_dir + "/sim_data/ci_artefacts/ci_test_latest" + ".png"
        filename = script_dir + \
            "/sim_data/ci_artefacts/ci_test_" + str(ct) + ".png"
        plt.savefig(filename_latest)
        print("Saved to", filename_latest)
        plt.savefig(filename)

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
                return

            if len(path_x_in_front) > 1:
                # Simulate movement of vehicle on path
                step_width = 1
                max_idx: int = len(path_x_in_front) - 1
                next_idx = min(step_width, max_idx)

                next_pose: PoseDouble = self.ego_utm_global.copy()

                while self.ego_utm_global.equal(next_pose):
                    # Relative movements to make movement independent of coordinate system origin --> more robust
                    dx = path_x_in_front[next_idx] - path_x_in_front[0]
                    dy = path_y_in_front[next_idx] - path_y_in_front[0]
                    next_pose: PoseDouble = PoseDouble(self.ego_utm_global.x + dx,
                                                       self.ego_utm_global.y + dy,
                                                       path_yaw_in_front[next_idx])

                    next_idx = min(next_idx + 1, max_idx)

                self.ego_utm_global = next_pose.copy()

    def get_preset_goal(self) -> tuple[PoseDouble, PoseDouble]:
        if self.FILE_NAME in ["Valet_parking_015", "Free_015"]:

            # Driving from bottom to top
            start_pose = PoseDouble(173.734262, 128.411466, 180.0)
            goal_pose = PoseDouble(161.325933, 161.124533, 90.0)

            # driving from lower left to lower right
            # start_pose = PoseDouble(136.010339, 129.572797, 90.0)
            # goal_pose = PoseDouble(170.347060, 129.358193, 0.0)

            # nice RA (hybrid)
            # start_pose = PoseDouble(165.976219, 128.519317, 0.0)
            # goal_pose = PoseDouble(159.960801, 138.577726, 0.0)

            # nice RA (extension)
            # start_pose = PoseDouble(167.976219, 128.519317, 180.0)
            # goal_pose = PoseDouble(136.455628, 128.971720, 90.0)

            # div path problem
            # start_pose = PoseDouble(173.5, 160, -90.0)
            # goal_pose = PoseDouble(140, 162, 90)

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

        return start_pose, goal_pose

    def get_start_and_goal_in_sim(self) -> tuple[PoseDouble, PoseDouble]:
        """
        Returns goal in utm coordinates

        Returns:

        """

        start_pose, goal_pose = self.get_preset_goal()

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
                    None, self.plan)
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
        start_pose, goal_pose = self.get_start_and_goal_in_sim()

        # Add to goal queue
        goal_msg = GoalMsg(goal_pose, remove=False)
        self.goal_msg_queue.append(goal_msg)

        # Set ego position
        self.ego_utm_global = start_pose.copy()

        # execute sim
        self.run_simulation()


if __name__ == "__main__":
    sim = Sim()
    sim.simulate()
