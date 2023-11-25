import os
import math
import time
from datetime import datetime
from copy import deepcopy
from typing import Optional
from numpy.typing import ArrayLike

import imviz as viz
import numpy as np
import matplotlib
import subprocess

from path_planner_lib.state_machine import *
from path_planner_lib import util
from path_planner_lib.planning import Vehicle, AStar, HybridAStar, CollisionChecker, Cartographing, UtilCpp
from path_planner_lib.planning import PoseDouble, PointDouble, PointInt
from path_planner_lib.logger_setup import Logger
from path_planner_lib.data_structures import GoalMsg, ActionObj, OccEnum, WayType


class Vis:
    def __init__(self):

        self.initialized: bool = False
        self.ini_path = None

        self.step_vals: ArrayLike = None
        self.idx: int = 0
        self.prev_plan_idx: int = -1
        self.pause: bool = False
        self.record: bool = False
        self.t_press: float = 0

        self.action_obj = None

        # Changed on the run for plotting
        self.car_driven_outlines_x: list = []
        self.car_driven_outlines_y: list = []

        self.add_driven_outlines_x: list = []
        self.add_driven_outlines_y: list = []

        # coordinates of search tree (only start and end to visualize faster)
        self.closed_x: list = []
        self.closed_y: list = []

        self.follow_zoom: float = 3  # the higher, this value the further away is the view
        self.follow_vehicle: bool = True

        self.map_plot_size: tuple = (500, 500)

        self.show_patch: bool = True
        self.show_internal_heuristics: bool = True
        self.show_collision_checks: bool = False
        self.show_internal_states: bool = False
        self.show_path_details: bool = False
        self.show_dilated_map: bool = False
        self.show_tree: bool = True
        self.show_circles: bool = True

        # Default visualization values. Can be changed from gui
        self.auto_fit: bool = False
        self.stop: bool = False

        self.show_file_dialog: bool = False
        self.add_driven_paths: list = []
        self.driven_x: list = []
        self.driven_y: list = []
        self.ego_utm_global: Optional[PointDouble] = None
        self.car_outline_x: list = []
        self.car_outline_y: list = []

        self.allow_scrolling: bool = False

        self.state_history: list = []
        self.mouse_pos: tuple = (0, 0)

        self.goal_pose: PoseDouble = PoseDouble(0, 0, 0)
        self.goal_pose_utm: PoseDouble = PoseDouble(0, 0, 0)
        self.goal_set: bool = False
        self.goal_message: Optional = None

        self.map_context_menu_open: bool = False
        self.logger: Logger = None

        self.rec_process_started: bool = False
        self.frame_width: int = -1
        self.frame_height: int = 1
        self.proc = None
        self.goal_reached = False
        self.last_iterations = math.inf

        self.create_lane_graph: bool = False

    def initialize(self):
        if not self.initialized:
            self.logger = Logger(self)

            self.ini_path = os.path.dirname(os.path.realpath(__file__)) + "/.freespace_planner.ini"
            viz.load_ini(self.ini_path)

            self.initialized = True

    @staticmethod
    def ffmpeg_cmd(width, height, path):
        return [
            '/usr/bin/ffmpeg',
            '-y',
            '-f', 'rawvideo',
            '-s', f'{int(width)}x{int(height)}',
            '-pix_fmt', 'rgb24',
            '-r', '24',
            '-i', '-',
            '-vcodec', 'mpeg4',
            '-b:v', '10M',
            f'{path}'
        ]

    @staticmethod
    def get_vehicle_outline(x: float, y: float, yaw: float, transf: str = "utm", keep_pos: bool = False) \
            -> tuple[list, list]:

        vertices = Vehicle.getVehicleVertices()

        vrx, vry = [point.x for point in vertices], [point.y for point in vertices]
        vrx.append(vrx[0])
        vry.append(vry[0])

        point: PointDouble = PointDouble(x, y)
        if transf == "gm":
            if not keep_pos:
                point = UtilCpp.utm2grid(point)

            vrx, vry = UtilCpp.utm2grid([vrx, vry])

        yaw_rot: float = -yaw
        rot = np.array([[math.cos(yaw_rot), -math.sin(yaw_rot)],
                        [math.sin(yaw_rot), math.cos(yaw_rot)]])
        car_outline_x: list = []
        car_outline_y: list = []
        for rx, ry in zip(vrx, vry):
            converted_xy = np.stack([rx, ry]).T @ rot
            car_outline_x.append(converted_xy[0] + point.x)
            car_outline_y.append(converted_xy[1] + point.y)

        return car_outline_x, car_outline_y

    def set_xaxis_settings4states(self, plan):
        viz.setup_axis(viz.Axis.X1, "steps")
        if self.auto_fit:
            if not self.stop:
                viz.setup_axis(viz.Axis.X1, "steps", flags=viz.PlotAxisFlags.AUTO_FIT)
        else:
            if not self.stop:
                viz.setup_axis_limits(viz.Axis.X1, -plan.history_len, 0,
                                      flags=viz.PlotCond.ALWAYS)

    def enable_goal_change(self):
        viz.push_override_id(viz.get_plot_id())
        if viz.begin_popup("##PlotContext"):

            self.map_context_menu_open = True

            if viz.menu_item("Set Goal"):
                self.goal_pose = PoseDouble(self.mouse_pos[0], self.mouse_pos[1],
                                            self.goal_pose.yaw)
                self.goal_pose_utm = UtilCpp.grid2utm(self.goal_pose)
                self.goal_set = True
                self.map_context_menu_open = False

            viz.end_menu()

        viz.pop_id()

    def enable_follow_veh(self):
        viz.push_override_id(viz.get_plot_id())
        if viz.begin_popup("##PlotContext"):

            # if not self.map_context_menu_open:
            self.map_context_menu_open = True

            if viz.menu_item("Follow Vehicle"):
                self.follow_vehicle = True
                self.map_context_menu_open = False

            viz.end_menu()
        viz.pop_id()

    def plot_selected_and_current_goal(self, plan):
        # Plot selected goal
        goal_outline_x, goal_outline_y = self.get_vehicle_outline(self.goal_pose.x, self.goal_pose.y,
                                                                  self.goal_pose.yaw,
                                                                  transf="gm", keep_pos=True)
        viz.plot(np.array(goal_outline_x), np.array(goal_outline_y), line_weight=3, fmt="-", color="orange")

        # Plot selected goal
        if plan.goal_utm is not None:
            goal_pose = UtilCpp.utm2grid(plan.goal_utm)
            goal_outline_x, goal_outline_y = self.get_vehicle_outline(goal_pose.x, goal_pose.y, plan.goal_utm.yaw,
                                                                      transf="gm", keep_pos=True)
            viz.plot(np.array(goal_outline_x), np.array(goal_outline_y), line_weight=3, fmt="-", color="red",
                     label="goal")

    def plot_circles(self, plan):

        ego_pose = UtilCpp.patch_utm2global_gm(plan.ego_utm_patch)
        points = CollisionChecker.returnDiskPositions(plan.ego_utm_patch.yaw)
        viz.plot([p.x + ego_pose.x for p in points], [p.y + ego_pose.y for p in points], marker_size=2.0,
                 fmt=".o", color="red")
        for p in points:
            viz.plot_circle([p.x + ego_pose.x, p.y + ego_pose.y], CollisionChecker.disk_r_c_, color="red")

        if self.goal_pose is not None:
            points = CollisionChecker.returnDiskPositions(self.goal_pose.yaw)
            viz.plot([p.x + self.goal_pose.x for p in points], [p.y + self.goal_pose.y for p in points],
                     marker_size=2.0, fmt=".o", color="red")
            for p in points:
                viz.plot_circle([p.x + self.goal_pose.x, p.y + self.goal_pose.y], CollisionChecker.disk_r_c_,
                                color="red")

    def plot_search_tree(self):
        closed_coordinates = HybridAStar.getConnectedClosedNodes()

        if len(closed_coordinates[0]) != 0:
            x_coords, y_coords = closed_coordinates
            self.closed_x, self.closed_y = UtilCpp.patch_utm2global_gm(x_coords, y_coords)

        if self.closed_x:
            viz.plot(self.closed_x, self.closed_y, marker_size=1, fmt="-", color="red", label="closed nodes",
                     flags=viz.PlotLineFlags.SEGMENTS)

    def enable_and_follow_vehicle(self, plan):
        self.enable_follow_veh()
        if self.follow_vehicle:
            ego_gm_global = UtilCpp.utm2grid(self.ego_utm_global)

            if viz.is_window_hovered():
                for se in viz.get_scroll_events():
                    if se.yoffset < 0:
                        self.follow_zoom *= abs(se.yoffset) * 1.1
                    elif se.yoffset > 0:
                        self.follow_zoom /= abs(se.yoffset) * 1.1

                if not (viz.get_mouse_drag_delta() == 0.0).all():
                    self.follow_vehicle = False

            d = 100 * self.follow_zoom

            aspect_ratio = self.map_plot_size[1] / self.map_plot_size[0]
            plot_x_min = ego_gm_global.x - d
            plot_x_max = ego_gm_global.x + d
            plot_y_min = ego_gm_global.y - d * aspect_ratio
            plot_y_max = ego_gm_global.y + d * aspect_ratio

            viz.setup_axis_limits(viz.Axis.X1, plot_x_min, plot_x_max, flags=viz.PlotCond.ALWAYS)
            viz.setup_axis_limits(viz.Axis.Y1, plot_y_min, plot_y_max, flags=viz.PlotCond.ALWAYS)

            self.map_plot_size = viz.get_plot_size()

        else:
            origin_gm = UtilCpp.utm2grid(plan.patch_info.origin_utm)
            viz.setup_axis_limits(viz.Axis.X1, origin_gm.x,
                                  origin_gm.x + plan.patch_info.dim,
                                  flags=viz.PlotCond.ONCE)
            viz.setup_axis_limits(viz.Axis.Y1, origin_gm.y,
                                  origin_gm.y + plan.patch_info.dim,
                                  flags=viz.PlotCond.ONCE)

    def vis_dilated_collision_map(self, plan):
        if self.show_dilated_map:
            window_title = "Dilated collision map"
            if viz.begin_window(
                    window_title,
                    # size=(700, 700),
                    # position=(0, 0),
                    resize=True,
            ):
                self.show_dilated_map = viz.get_window_open()

                if viz.begin_plot(window_title, flags=viz.PlotFlags.EQUAL):

                    viz.setup_axis(viz.Axis.X1, "x")
                    viz.setup_axis(viz.Axis.Y1, "y")

                    self.enable_and_follow_vehicle(plan)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    if not self.map_context_menu_open:
                        self.mouse_pos = viz.get_plot_mouse_pos()

                    patch_safety = CollisionChecker.patch_safety_arr_.getNumpyArr()

                    offset = UtilCpp.utm2grid(plan.patch_info.origin_utm)
                    viz.plot_image("Grid Map", 1 - (patch_safety / 2),
                                   x=offset.x,
                                   y=offset.y,
                                   uv0=(0, 1), uv1=(1, 0), interpolate=False)
                    self.show_pixel_value(patch_safety, offset)

                    ego_pose = UtilCpp.patch_utm2global_gm(plan.ego_utm_patch)
                    points = CollisionChecker.returnDiskPositions(plan.ego_utm_patch.yaw)
                    viz.plot([p.x + ego_pose.x for p in points], [p.y + ego_pose.y for p in points], marker_size=2.0,
                             fmt=".o", color="red")

                    if self.goal_pose is not None:
                        points = CollisionChecker.returnDiskPositions(self.goal_pose.yaw)
                        viz.plot([p.x + self.goal_pose.x for p in points], [p.y + self.goal_pose.y for p in points],
                                 marker_size=2.0, fmt=".o", color="red")

                    if plan.path is not None:
                        if plan.path.x_list:
                            # Cast to global gm coordinates
                            path_x, path_y = UtilCpp.patch_utm2global_gm(plan.path.x_list, plan.path.y_list)

                            viz.plot(path_x, path_y, line_weight=2, color="blue", fmt="-o", label="planned path")

                    viz.plot(self.driven_x, self.driven_y, line_weight=2, marker_size=1.0, color="black",
                             fmt="-",
                             label="driven path")
                    viz.plot(self.car_outline_x, self.car_outline_y, line_weight=2, color="blue", fmt="-",
                             label="vehicle")

                    self.plot_selected_and_current_goal(plan)
                    self.enable_goal_change()

                    viz.end_plot()

            viz.end_window()

    def vis_collision_checks(self, minipatches, plan):
        if self.show_collision_checks:
            window_title = "Collision Checks"
            if viz.begin_window(
                    window_title,
                    # size=(400, 400),
                    # position=(700, 0),
                    resize=True,
            ):
                self.show_collision_checks = viz.get_window_open()

                if viz.begin_plot(window_title, flags=viz.PlotFlags.EQUAL):  # | viz.PlotFlags.NO_LEGEND
                    viz.setup_axis(viz.Axis.X1, "x")
                    viz.setup_axis(viz.Axis.Y1, "y")

                    self.enable_and_follow_vehicle(plan)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    patch = CollisionChecker.patch_arr_.getNumpyArr()
                    offset = UtilCpp.utm2grid(plan.patch_info.origin_utm)
                    viz.plot_image("Patch", (OccEnum.OCC - patch),
                                   x=offset.x,
                                   y=offset.y,
                                   uv0=(0, 1), uv1=(1, 0), interpolate=False)
                    self.show_pixel_value(patch, offset)

                    # for path in self.generated_paths:
                    #     if path is not None:
                    #         path_x, path_y = UtilCpp.utm2global_gm(path.x_list, path.y_list)
                    #         viz.plot(path_x, path_y, line_weight=2, color="black", fmt="--")

                    # add additional paths
                    colors = ["red", "green", "yellow"]
                    if self.idx % 100 == 0:
                        for path_idx, (add_path, filename) in enumerate(self.add_driven_paths):
                            if add_path is not None:
                                self.add_driven_outlines_x = []
                                self.add_driven_outlines_y = []

                                x_list = []
                                y_list = []
                                for i, (x, y, yaw) in enumerate(zip(add_path.x_list, add_path.y_list,
                                                                    add_path.yaw_list)):
                                    if i % 4 != 0:
                                        continue
                                    car_outline_x, car_outline_y = self.get_vehicle_outline(x, y, yaw, transf="gm")
                                    x_list.append(car_outline_x)
                                    y_list.append(car_outline_y)

                                self.add_driven_outlines_x.append(x_list)
                                self.add_driven_outlines_y.append(y_list)

                    # plot additional paths
                    if self.add_driven_paths:
                        for i, (x_list, y_list) in enumerate(
                                zip(self.add_driven_outlines_x, self.add_driven_outlines_y)):
                            for outline_x, outline_y in zip(x_list, y_list):
                                viz.plot(outline_x, outline_y, line_weight=2, color=colors[i % len(colors)], fmt="-",
                                         label="")

                    # on len change
                    if len(plan.driven_path.x_list) > len(self.car_driven_outlines_x) * 6:
                        self.car_driven_outlines_x = []
                        self.car_driven_outlines_y = []
                        for i, (x, y, yaw) in enumerate(zip(plan.driven_path.x_list, plan.driven_path.y_list,
                                                            plan.driven_path.yaw_list)):
                            if i % 4 != 0:
                                continue
                            car_outline_x, car_outline_y = self.get_vehicle_outline(x, y, yaw, transf="gm")
                            self.car_driven_outlines_x.append(car_outline_x)
                            self.car_driven_outlines_y.append(car_outline_y)

                    for i, (outline_x, outline_y) in enumerate(
                            zip(self.car_driven_outlines_x, self.car_driven_outlines_y)):
                        viz.plot(outline_x, outline_y, line_weight=2, color="blue", fmt="-", label="vehicle")

                    # viz.plot(self.driven_x, self.driven_y, line_weight=3, marker_size=1, color="black",
                    #          fmt="-",
                    #          label="driven path")

                    self.plot_selected_and_current_goal(plan)
                    self.enable_goal_change()

                    viz.end_plot()

            viz.end_window()

    def vis_patch(self, plan):
        if self.show_patch:

            window_title = "Patch"
            if viz.begin_window(
                    window_title,
                    # size=(700, 700),
                    # position=(0, 0),
                    resize=True,
            ):
                self.show_patch = viz.get_window_open()

                if viz.begin_plot(window_title, flags=viz.PlotFlags.EQUAL):

                    viz.setup_axis(viz.Axis.X1, "x in grid cells")
                    viz.setup_axis(viz.Axis.Y1, "y in grid cells")

                    self.enable_and_follow_vehicle(plan)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    if not self.map_context_menu_open:
                        self.mouse_pos = viz.get_plot_mouse_pos()

                    patch = CollisionChecker.patch_arr_.getNumpyArr()
                    offset = UtilCpp.utm2grid(plan.patch_info.origin_utm)
                    viz.plot_image("Patch", (OccEnum.OCC - patch),
                                   x=offset.x,
                                   y=offset.y,
                                   uv0=(0, 1), uv1=(1, 0), interpolate=False)
                    self.show_pixel_value(patch, offset)

                    if self.create_lane_graph:
                        if self.mouse_pos is not None:
                            pos = PointDouble(self.mouse_pos[0], self.mouse_pos[1])
                            pos = UtilCpp.grid2utm(pos)

                            if viz.is_window_hovered():
                                if viz.is_mouse_clicked():
                                    HybridAStar.lane_graph_.addPoint(pos)

                    # show edges of lane graph
                    for i_edge, edge in enumerate(HybridAStar.lane_graph_.edges_):
                        points = [UtilCpp.utm2grid(node_opt.point_utm) for node_opt in edge if node_opt]

                        xs = [point.x for point in points]
                        ys = [point.y for point in points]

                        color = "black"
                        line_weight = 1
                        viz.plot(xs, ys, fmt="-o", color=color, line_weight=line_weight)

                    self.plot_selected_and_current_goal(plan)

                    ego_pose = UtilCpp.patch_utm2global_gm(plan.ego_utm_patch)
                    viz.plot([ego_pose.x], [ego_pose.y], fmt="o", color="blue")

                    if self.show_circles:
                        self.plot_circles(plan)

                    if self.show_tree:
                        self.plot_search_tree()

                    # Get astar path
                    if plan.WAYPOINT_TYPE != WayType.NONE:
                        if plan.astar_path:
                            way_x, way_y = UtilCpp.astar2utm([plan.astar_path[0], plan.astar_path[1]])
                            # way_y = TF.astar2con(plan.astar_path[1])
                            # Cast to global utm coordinates
                            way_x, way_y = UtilCpp.patch_utm2global_gm(way_x, way_y)

                            viz.plot(way_x, way_y, marker_size=2, color="black", fmt="-o", label="A* path")

                    if plan.path is not None:
                        if plan.path.x_list:
                            # Cast to global utm coordinates
                            path_x, path_y = UtilCpp.patch_utm2global_gm(plan.path.x_list, plan.path.y_list)

                            if len(path_x) == 0:
                                return

                            viz.plot(path_x, path_y, line_weight=2, color="blue", marker_size=2, fmt="-o", label="path")

                    if plan.coll_point is not None:
                        coll_point = UtilCpp.patch_utm2global_gm(plan.coll_point)
                        viz.plot([coll_point.x], [coll_point.y], marker_size=5, color="red", fmt=".x")
                        # self.pause = True

                    viz.plot(self.driven_x, self.driven_y, line_weight=2, color="black", marker_size=2, fmt="-o",
                             label="driven path")

                    viz.plot(self.car_outline_x, self.car_outline_y, line_weight=3, color="blue",
                             fmt="-", label="vehicle")

                    # Create video
                    if self.record:
                        self.do_record(plan)

                    self.enable_goal_change()

                    viz.end_plot()
                viz.end_window()

    def do_record(self, plan):
        if plan.sim_idx > self.prev_plan_idx and plan.sim_idx > 5:

            if self.last_iterations != math.inf:
                self.last_iterations -= 1

            if self.last_iterations > 0:
                self.prev_plan_idx = plan.sim_idx
                x, y = viz.get_plot_pos()
                self.frame_width, self.frame_height = viz.get_plot_size()
                if self.frame_height % 2 == 1:
                    self.frame_height -= 1
                    self.frame_height = int(self.frame_height)
                frame = viz.get_pixels(x, y, self.frame_width, self.frame_height)[:, :, :3]

                if plan.WAYPOINT_TYPE == WayType.NONE:
                    pad_dim = 10
                    pad = np.zeros((self.frame_height, pad_dim, 3), dtype=np.uint8)
                    frame = np.hstack((frame, pad))
                    self.frame_width += pad_dim

                if not self.rec_process_started:
                    self.rec_process_started = True
                    stamp = datetime.now().strftime("%d%m%Y_%H%M%S")
                    scenario_name = "scenario_" + str(stamp)
                    self.proc = subprocess.Popen(
                        self.ffmpeg_cmd(self.frame_width,
                                        self.frame_height,
                                        f"/opt/guided-extended-hybrid-astar/recorded_videos/{scenario_name}.mp4"),
                        stdin=subprocess.PIPE,
                        stdout=None,
                        stderr=None)
                self.proc.stdin.write(frame.tobytes())

    @staticmethod
    def show_pixel_value(arr, offset=PointDouble(0, 0)):
        mouse_pos = viz.get_plot_mouse_pos()
        # correct mouse pos
        mouse_pos[0] -= offset.x
        mouse_pos[1] -= offset.y

        val = None
        if mouse_pos is not None:
            if 0 <= mouse_pos[0] < arr.shape[0] and 0 <= mouse_pos[1] < arr.shape[1]:
                y_ind = max(int(np.floor(mouse_pos[1])), 0)
                x_ind = max(int(np.floor(mouse_pos[0])), 0)
                val = arr[y_ind, x_ind]
        viz.text("Pixel value: " + str(val))

    def vis_internal_heuristics(self, plan):
        if self.show_internal_heuristics:

            # heuristic Map
            window_title = "Heuristic map"
            if viz.begin_window(
                    window_title,
                    # size=(400, 400),
                    # position=(700, 400),
                    resize=True,
            ):
                self.show_internal_heuristics = viz.get_window_open()

                if viz.begin_plot(window_title, flags=viz.PlotFlags.EQUAL):
                    viz.setup_axis(viz.Axis.X1, "x in planning cells")
                    viz.setup_axis(viz.Axis.Y1, "y in planning cells")

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    # visualize map
                    cmap = matplotlib.cm.get_cmap('jet_r')
                    heur_map_gray = -np.ones((HybridAStar.astar_dim_, HybridAStar.astar_dim_), dtype=np.float64)

                    # TODO (Schumann) do all this in cpp
                    h_dp_global = AStar.getDistanceHeuristic(False)
                    # self.logger.log_debug("Number of visited nodes", len(h_dp_global))
                    if h_dp_global is not None:
                        for obj in h_dp_global:
                            el = obj[1]
                            heur_map_gray[el.pos.y, el.pos.x] = el.cost_

                    # normalise
                    max_val = np.max(heur_map_gray)
                    heur_map_gray[heur_map_gray == -1] = max_val
                    heur_map_gray_norm = heur_map_gray / max_val
                    heur_map_color = cmap(heur_map_gray_norm)[:, :, 0:3]

                    self.show_pixel_value(heur_map_gray)
                    viz.plot_image(window_title, heur_map_color, uv0=(0, 1), uv1=(1, 0), interpolate=False)
                    if plan.astar_path:
                        viz.plot(plan.astar_path[0], plan.astar_path[1], marker_size=2, color="black", fmt="-o")

                    viz.end_plot()

            viz.end_window()

            # Motion res Map
            window_title = "Motion res map"
            if viz.begin_window(
                    window_title,
                    # size=(400, 400),
                    # position=(1100, 0),
                    resize=True,
            ):
                self.show_internal_heuristics = viz.get_window_open()

                if viz.begin_plot(window_title, flags=viz.PlotFlags.EQUAL):
                    viz.setup_axis(viz.Axis.X1, "x in planning cells")
                    viz.setup_axis(viz.Axis.Y1, "y in planning cells")
                    viz.setup_axis_limits(viz.Axis.X1, 0, HybridAStar.astar_dim_, flags=viz.PlotCond.ONCE)
                    viz.setup_axis_limits(viz.Axis.Y1, 0, HybridAStar.astar_dim_, flags=viz.PlotCond.ONCE)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    motion_res_map = AStar.motion_res_map_.getNumpyArr()

                    if motion_res_map is not None:
                        self.show_pixel_value(motion_res_map)
                        viz.plot_image(window_title, motion_res_map, uv0=(0, 1), uv1=(1, 0), interpolate=False)

                    if plan.astar_path:
                        viz.plot(plan.astar_path[0], plan.astar_path[1], marker_size=2, color="blue", fmt="-o")

                    viz.end_plot()

            viz.end_window()

            # Motion res Map
            window_title = "movement cost map"
            if viz.begin_window(
                    window_title,
                    # size=(400, 400),
                    # position=(1100, 0),
                    resize=True,
            ):
                self.show_internal_heuristics = viz.get_window_open()

                if viz.begin_plot(window_title, flags=viz.PlotFlags.EQUAL):
                    viz.setup_axis(viz.Axis.X1, "x in planning cells")
                    viz.setup_axis(viz.Axis.Y1, "y in planning cells")
                    viz.setup_axis_limits(viz.Axis.X1, 0, HybridAStar.astar_dim_, flags=viz.PlotCond.ONCE)
                    viz.setup_axis_limits(viz.Axis.Y1, 0, HybridAStar.astar_dim_, flags=viz.PlotCond.ONCE)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    movement_cost_map = AStar.movement_cost_map_.getNumpyArr()

                    if movement_cost_map is not None:
                        self.show_pixel_value(movement_cost_map)
                        viz.plot_image(window_title, movement_cost_map / 5, uv0=(0, 1), uv1=(1, 0), interpolate=False)

                    if plan.astar_path:
                        viz.plot(plan.astar_path[0], plan.astar_path[1], marker_size=2, color="blue", fmt="-o")

                    viz.end_plot()

            viz.end_window()

            # Proximity Map
            window_title = "Proximity map"
            if viz.begin_window(
                    window_title,
                    # size=(400, 400),
                    # position=(1100, 400),
                    resize=True,
            ):
                self.show_internal_heuristics = viz.get_window_open()

                if viz.begin_plot(window_title, flags=viz.PlotFlags.EQUAL):
                    viz.setup_axis(viz.Axis.X1, "x in planning cells")
                    viz.setup_axis(viz.Axis.Y1, "y in planning cells")
                    viz.setup_axis_limits(viz.Axis.X1, 0, HybridAStar.astar_dim_, flags=viz.PlotCond.ONCE)
                    viz.setup_axis_limits(viz.Axis.Y1, 0, HybridAStar.astar_dim_, flags=viz.PlotCond.ONCE)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    h_prox = AStar.h_prox_arr_.getNumpyArr()
                    if h_prox is not None:
                        viz.plot_image(window_title, 1 - h_prox, uv0=(0, 1), uv1=(1, 0), interpolate=False)
                        self.show_pixel_value(h_prox)

                    if plan.astar_path:
                        viz.plot(plan.astar_path[0], plan.astar_path[1], line_weight=3, color="blue", fmt="-o")

                    viz.end_plot()

            viz.end_window()

            # Planning Map
            window_title = "Planning map"
            if viz.begin_window(
                    window_title,
                    # size=(400, 400),
                    # position=(700, 400),
                    resize=True,
            ):
                self.show_internal_heuristics = viz.get_window_open()

                if viz.begin_plot(window_title, flags=viz.PlotFlags.EQUAL):
                    viz.setup_axis(viz.Axis.X1, "x in planning cells")
                    viz.setup_axis(viz.Axis.Y1, "y in planning cells")
                    viz.setup_axis_limits(viz.Axis.X1, 0, HybridAStar.astar_dim_, flags=viz.PlotCond.ONCE)
                    viz.setup_axis_limits(viz.Axis.Y1, 0, HybridAStar.astar_dim_, flags=viz.PlotCond.ONCE)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    astar_grid = AStar.astar_grid_.getNumpyArr()
                    viz.plot_image(window_title, 1 - astar_grid / 2, uv0=(0, 1), uv1=(1, 0), interpolate=False)
                    self.show_pixel_value(astar_grid)

                    if plan.astar_path:
                        viz.plot(plan.astar_path[0], plan.astar_path[1], marker_size=2, color="blue", fmt="-o",
                                 label="waypoints")

                    viz.plot([plan.ego_node.x_index], [plan.ego_node.y_index], color="blue", fmt="o")
                    viz.plot([plan.global_goal_node.x_index], [plan.global_goal_node.y_index], color="red", fmt="o")

                    viz.end_plot()

            viz.end_window()

    def vis_state_details(self, plan):
        window_title = "State output"

        if viz.begin_window(
                window_title,
                # size=(400, 400),
                # position=(1100, 400),
                resize=True,
        ):
            if viz.tree_node("State", viz.TreeNodeFlags.DEFAULT_OPEN):
                viz.text(plan.overall_state.repl_s)
                viz.text(plan.overall_state.ego_s)
                viz.text(plan.overall_state.goal_s)
                viz.text("plan.new_goal: " + str(plan.new_goal))
                viz.text("plan.to_final_pose: " + str(plan.to_final_pose))
                viz.text("Rec. goal collides: " + str(plan.rec_goal_collides))
                if plan.coll_idx != -1:
                    text = str(plan.overall_state.path_s) + str(" at ") + str(plan.coll_idx)
                else:
                    text = plan.overall_state.path_s
                viz.text(text)
                viz.tree_pop()

        viz.end_window()

        if self.show_internal_states:

            path_states = [el.path_s.value for el in self.state_history]
            goal_states = [el.goal_s.value for el in self.state_history]
            ego_states = [el.ego_s.value for el in self.state_history]
            repl_states = [el.repl_s.value for el in self.state_history]

            step_vals = np.arange(-len(path_states), 0)

            window_title = "REPL_STATE"
            if viz.begin_window(
                    window_title,
                    resize=True,
            ):
                if viz.begin_plot(window_title):

                    self.set_xaxis_settings4states(plan)

                    viz.setup_axis(viz.Axis.Y1, "REPL_STATE")
                    viz.setup_axis_limits(viz.Axis.Y1, -0.1, 3.1, flags=viz.PlotCond.ALWAYS)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    viz.plot(step_vals, repl_states, color="blue")
                    viz.end_plot()

                viz.end_window()
            window_title = "PATH_STATE"
            if viz.begin_window(
                    window_title,
                    resize=True,
            ):
                if viz.begin_plot(window_title):

                    self.set_xaxis_settings4states(plan)

                    viz.setup_axis(viz.Axis.Y1, "PATH_STATE")
                    viz.setup_axis_limits(viz.Axis.Y1, -0.1, 1.1, flags=viz.PlotCond.ALWAYS)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()
                    viz.plot(step_vals, path_states, color="red")
                    viz.end_plot()

                viz.end_window()
            window_title = "GOAL_STATE"
            if viz.begin_window(
                    window_title,
            ):
                if viz.begin_plot(window_title):

                    self.set_xaxis_settings4states(plan)

                    viz.setup_axis(viz.Axis.Y1, "GOAL_STATE")
                    viz.setup_axis_limits(viz.Axis.Y1, -0.1, 1.1, flags=viz.PlotCond.ALWAYS)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()
                    viz.plot(step_vals, goal_states, color="green")
                    viz.end_plot()

                viz.end_window()
            window_title = "EGO_STATE"
            if viz.begin_window(
                    window_title,
            ):
                if viz.begin_plot(window_title):

                    self.set_xaxis_settings4states(plan)

                    viz.setup_axis(viz.Axis.Y1, "EGO_STATE")
                    viz.setup_axis_limits(viz.Axis.Y1, -0.1, 2.1, flags=viz.PlotCond.ALWAYS)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()
                    viz.plot(step_vals, ego_states, color="blue")
                    viz.end_plot()
                viz.end_window()

    def vis_vis_parameters(self, plan, minipatches: dict):
        window_title = "Vis"
        if viz.begin_window(
                window_title,
        ):
            max_patch_dist = viz.drag("max_patch_dist", plan.max_patch_dist)
            if max_patch_dist != plan.max_patch_dist:
                minipatches.clear()
                plan.max_patch_dist = max_patch_dist

            plan.history_len = viz.input("history_len", plan.history_len)
            self.auto_fit = viz.checkbox("auto_fit", self.auto_fit)
            viz.same_line()
            self.stop = viz.checkbox("stop", self.stop)
            self.show_tree = viz.checkbox("Show Tree", self.show_tree)
            self.show_circles = viz.checkbox("Show Circles", self.show_circles)
            if viz.button("EraseHistory"):
                self.car_driven_outlines_x.clear()
                self.car_driven_outlines_y.clear()
                self.closed_x.clear()
                self.closed_y.clear()
                plan.driven_path.x_list.clear()
                plan.driven_path.y_list.clear()
                plan.driven_path.yaw_list.clear()
                plan.driven_path.direction_list.clear()
                plan.driven_path.cost_list.clear()
                plan.driven_path.cost = 0
                plan.driven_path.idx_analytic = -1

            viz.end_window()

    def vis_path_details(self, plan):
        if self.show_path_details:
            driven_yaw = plan.driven_path.yaw_list
            driven_dirs = plan.driven_path.direction_list

            if driven_yaw:
                # Plot states
                window_title = "Yaw (driven)"
                if viz.begin_window(
                        window_title,
                ):
                    if viz.begin_plot(window_title):

                        self.set_xaxis_settings4states(plan)
                        viz.setup_axis(viz.Axis.Y1, "Y")
                        viz.setup_axis_limits(viz.Axis.Y1, min(-math.pi, min(driven_yaw)),
                                              max(math.pi, max(driven_yaw)),
                                              flags=viz.PlotCond.ALWAYS)
                        if viz.plot_selection_ended():
                            viz.hard_cancel_plot_selection()

                        viz.plot(self.step_vals, driven_yaw, color="red")

                        viz.end_plot()
                    viz.end_window()

            window_title = "Yaw(planned)"
            if viz.begin_window(
                    window_title,
            ):
                if viz.begin_plot(window_title):

                    viz.setup_axis(viz.Axis.X1, "X")
                    if plan.path is not None:
                        viz.setup_axis_limits(viz.Axis.X1, 0, len(plan.path.yaw_list),
                                              flags=viz.PlotCond.ALWAYS)
                    viz.setup_axis(viz.Axis.Y1, "Y")
                    viz.setup_axis_limits(viz.Axis.Y1, -math.pi, math.pi, flags=viz.PlotCond.ALWAYS)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    if plan.path is not None:
                        viz.plot(plan.path.yaw_list, color="red")

                    viz.end_plot()
            viz.end_window()

            if driven_yaw:
                window_title = "Curvature(driven)"
                if viz.begin_window(
                        window_title,
                ):
                    if viz.begin_plot(window_title):

                        self.set_xaxis_settings4states(plan)

                        curvatures = util.get_curvatures(driven_yaw, ds=0.1)

                        viz.setup_axis(viz.Axis.Y1, "Y")

                        if viz.plot_selection_ended():
                            viz.hard_cancel_plot_selection()


                        viz.plot(self.step_vals, curvatures, color="blue")
                        viz.end_plot()
                viz.end_window()

            window_title = "Curvature(planned)"
            if viz.begin_window(
                    window_title,
            ):
                if viz.begin_plot(window_title):

                    viz.setup_axis(viz.Axis.X1, "X")

                    if plan.path is not None:
                        viz.setup_axis_limits(viz.Axis.X1, 0, len(plan.path.yaw_list),
                                              flags=viz.PlotCond.ALWAYS)

                    viz.setup_axis(viz.Axis.Y1, "Y")

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    if plan.path is not None:
                        curvatures = util.get_curvatures(plan.path.yaw_list, ds=0.1)
                        viz.plot(curvatures, color="blue")

                    viz.end_plot()
            viz.end_window()

            if driven_dirs:
                window_title = "Direction(driven)"
                if viz.begin_window(
                        window_title,
                ):
                    if viz.begin_plot(window_title):

                        self.set_xaxis_settings4states(plan)

                        viz.setup_axis(viz.Axis.Y1, "Y")
                        viz.setup_axis_limits(viz.Axis.Y1, -1.1, 1.1, flags=viz.PlotCond.ALWAYS)

                        if viz.plot_selection_ended():
                            viz.hard_cancel_plot_selection()
                        viz.plot(self.step_vals, driven_dirs, color="green")
                        viz.end_plot()
                viz.end_window()

            window_title = "Direction(planned)"
            if viz.begin_window(
                    window_title,
            ):
                if viz.begin_plot(window_title):
                    # nb_data = len(plan.path.direction_list)

                    viz.setup_axis(viz.Axis.X1, "X")
                    if plan.path is not None:
                        viz.setup_axis_limits(viz.Axis.X1, 0, len(plan.path.direction_list),
                                              flags=viz.PlotCond.ALWAYS)

                    viz.setup_axis(viz.Axis.Y1, "Y")
                    viz.setup_axis_limits(viz.Axis.Y1, -1.1, 1.1, flags=viz.PlotCond.ALWAYS)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    if plan.path is not None:
                        viz.plot(plan.path.direction_list, color="green")

                    viz.end_plot()
            viz.end_window()

    def vis_cycle_times(self, plan):
        window_title = "Cycle times"
        if viz.begin_window(window_title):
            if viz.begin_plot(window_title):

                self.set_xaxis_settings4states(plan)

                if self.stop:
                    viz.setup_axis(viz.Axis.Y1, "Y", flags=viz.PlotAxisFlags.AUTO_FIT)
                else:
                    max_val = 0
                    if len(plan.planning_cycle_times) > 0:
                        max_val = np.max(plan.planning_cycle_times)
                    viz.setup_axis_limits(viz.Axis.Y1, 0,
                                          max(max_val, 0.05),
                                          flags=viz.PlotCond.ALWAYS)

                if viz.plot_selection_ended():
                    viz.hard_cancel_plot_selection()

                step_vals = np.arange(-len(plan.planning_cycle_times), 0)
                viz.plot(step_vals, plan.planning_cycle_times, color="cyan", label="planning")
                viz.plot(step_vals, plan.hastar_cycle_times, color="red", label="H-A* planning")

                viz.end_plot()

        viz.end_window()

    def vis_poses_and_control(self, plan):
        window_title = "Goal pose"
        if viz.begin_window(
                window_title,
                resize=True,
        ):

            if viz.tree_node("Ego PoseDouble (utm) (read-only)", viz.TreeNodeFlags.DEFAULT_OPEN):
                plan.ego_utm.x = viz.drag("x", plan.ego_utm.x)
                plan.ego_utm.y = viz.drag("y", plan.ego_utm.y)
                plan.ego_utm.yaw = viz.drag("yaw", plan.ego_utm.yaw)
                viz.tree_pop()

            if viz.tree_node("Goal Pose (utm)", viz.TreeNodeFlags.DEFAULT_OPEN):
                self.goal_pose_utm.x = viz.drag("x", self.goal_pose_utm.x)
                self.goal_pose_utm.y = viz.drag("y", self.goal_pose_utm.y)
                self.goal_pose_utm.yaw = viz.drag("yaw_rad", self.goal_pose_utm.yaw)
                self.goal_pose_utm.yaw = np.deg2rad(viz.drag("yaw_deg", int(np.rad2deg(self.goal_pose_utm.yaw))))
                viz.tree_pop()

            if viz.tree_node("Goal Pose (patch utm)"):
                if plan.goal_utm_patch is not None:
                    viz.drag("x", plan.goal_utm_patch.x)
                    viz.drag("y", plan.goal_utm_patch.y)
                    viz.drag("yaw_rad", plan.goal_utm_patch.yaw)
                    np.deg2rad(viz.drag("yaw_deg", int(np.rad2deg(plan.goal_utm_patch.yaw))))
                viz.tree_pop()

            if viz.button("+45"):
                self.goal_pose_utm.yaw += np.deg2rad(45)
            viz.same_line()
            if viz.button("-45"):
                self.goal_pose_utm.yaw -= np.deg2rad(45)
            viz.same_line()
            if viz.button("+90"):
                self.goal_pose_utm.yaw += np.deg2rad(90)
            viz.same_line()
            if viz.button("-90"):
                self.goal_pose_utm.yaw -= np.deg2rad(90)
            viz.same_line()
            if viz.button("+180"):
                self.goal_pose_utm.yaw += np.deg2rad(180)

            self.goal_pose = UtilCpp.utm2grid(self.goal_pose_utm)

            # Return new goal
            key_pressed = viz.get_key_events()
            space_pressed = key_pressed and key_pressed[0].key == 32
            plan_pressed = viz.button("Plan") and self.goal_set
            if (space_pressed or plan_pressed) and self.is_press_valid():
                # convert to rad
                self.logger.log_info("New goal set in visualization!")
                self.goal_message = GoalMsg(self.goal_pose_utm, remove=False)

            viz.same_line()
            if viz.button("RemoveGoal"):
                self.goal_message = GoalMsg(None, remove=True)
            viz.same_line()
            if viz.button("ResetPatch"):
                plan.reset_patch = True

            if viz.tree_node("Capsule control"):
                if viz.button("Take Capsule"):
                    plan.reinit_vehicle(has_capsule=True)
                    CollisionChecker.calculateDisks()
                viz.same_line()
                if viz.button("Drop Capsule"):
                    plan.reinit_vehicle(has_capsule=False)
                    CollisionChecker.calculateDisks()
                viz.tree_pop()

            if viz.tree_node("Graph control"):
                if viz.button("reset LaneGraph"):
                    HybridAStar.resetLaneGraph()
                    HybridAStar.updateLaneGraph(plan.patch_info.origin_utm, plan.patch_info.dim_utm)

                viz.same_line()
                if not self.create_lane_graph:
                    if viz.button("create LaneGraph"):
                        self.create_lane_graph = True
                else:
                    if viz.button("stop creation LaneGraph"):
                        self.create_lane_graph = False
                        HybridAStar.updateLaneGraph(plan.patch_info.origin_utm, plan.patch_info.dim_utm)

                if viz.button("save graph"):
                    self.save_graph(plan)
                viz.same_line()
                if viz.button("load graph"):
                    plan.load_graph()

                viz.same_line()
                if viz.button("smooth graph"):
                    HybridAStar.smoothLaneGraph(HybridAStar.lane_graph_)
                    HybridAStar.updateLaneGraph(plan.patch_info.origin_utm, plan.patch_info.dim_utm)

                viz.same_line()
                if viz.button("interpolate graph"):
                    HybridAStar.interpolateLaneGraph(HybridAStar.lane_graph_)
                    HybridAStar.updateLaneGraph(plan.patch_info.origin_utm, plan.patch_info.dim_utm)
                viz.tree_pop()

            viz.text("AStar config")
            AStar.alpha_ = viz.drag("alpha_", AStar.alpha_)
            AStar.do_max_ = viz.drag("do_max_", AStar.do_max_)
            AStar.do_min_ = viz.drag("do_min_", AStar.do_min_)
            AStar.astar_prox_cost_ = viz.drag("astar_prox_cost_", AStar.astar_prox_cost_)
            AStar.astar_movement_cost_ = viz.drag("astar_movement_cost_", AStar.astar_movement_cost_)
            AStar.astar_lane_movement_cost_ = viz.drag("astar_lane_movement_cost_", AStar.astar_lane_movement_cost_)

            viz.text("Hybrid AStar config")
            HybridAStar.switch_cost_ = viz.drag("switch_cost_", HybridAStar.switch_cost_)
            HybridAStar.steer_cost_ = viz.drag("steer_cost_", HybridAStar.steer_cost_)
            HybridAStar.steer_change_cost_ = viz.drag("steer_change_cost_", HybridAStar.steer_change_cost_)
            HybridAStar.h_dist_cost_ = viz.drag("h_dist_cost_", HybridAStar.h_dist_cost_)
            HybridAStar.back_cost_ = viz.drag("back_cost_", HybridAStar.back_cost_)
            HybridAStar.h_prox_cost_ = viz.drag("h_prox_cost_", HybridAStar.h_prox_cost_)

        viz.end_window()

    def save_graph(self, plan):
        x_list = [node.point_utm.x for node in HybridAStar.lane_graph_.nodes_]
        y_list = [node.point_utm.y for node in HybridAStar.lane_graph_.nodes_]
        data = np.vstack((x_list, y_list))
        np.save(plan.lane_filename, data)
        print("Saved LaneGraph")

    def is_press_valid(self):
        t_press = time.perf_counter()
        if t_press - self.t_press > 0.5:
            self.t_press = t_press
            return True
        return False

    def gather_data(self, plan):
        # always needed
        self.driven_x, self.driven_y = UtilCpp.utm2grid([plan.driven_path.x_list, plan.driven_path.y_list])

        self.ego_utm_global = UtilCpp.patch_utm2utm(plan.ego_utm_patch)

        self.car_outline_x, self.car_outline_y = self.get_vehicle_outline(self.ego_utm_global.x,
                                                                          self.ego_utm_global.y,
                                                                          plan.ego_utm_patch.yaw,
                                                                          transf="gm")

        self.state_history.append(deepcopy(plan.overall_state))

    def map_handling(self, save):
        self.action_obj = ActionObj()
        if save:
            self.action_obj.save_map()
            self.logger.log_info("Save map triggered from vis")
        else:
            self.action_obj.load_map()
            self.logger.log_info("Load map triggered from vis")

    def render_data(self, minipatches: dict, plan, show_vis: bool = True):
        self.initialize()

        main_title = "freespace planner"
        if show_vis:
            viz.set_main_window_title(main_title)

            if self.idx == 0:
                viz.set_main_window_size((1000, 1000))
                viz.set_main_window_pos((0, 0))
        else:
            title = main_title + "(deactivated)"
            viz.set_main_window_title(title)
            viz.set_main_window_size((10, 10))
            viz.set_main_window_pos((0, 0))

        # Only continue if patch_info is set
        if not viz.wait(vsync=False) or not show_vis or plan.patch_info is None or plan.ego_utm_patch is None:
            time.sleep(0.1)
            return None, self.pause, self.action_obj

        nb_history_els = min(plan.history_len, len(self.driven_x))
        self.step_vals = np.arange(-nb_history_els, 0)
        self.goal_message = None
        self.action_obj = None

        viz.style_colors_light()
        if viz.begin_main_menu_bar():
            if viz.begin_menu("Actions"):
                if viz.menu_item("Save Map"):
                    self.map_handling(save=True)
                if viz.menu_item("Load Map"):
                    self.map_handling(save=False)
                viz.end_menu()

            if viz.begin_menu("Data comparison"):
                if viz.menu_item("save current path"):
                    import pickle
                    from datetime import datetime
                    now = datetime.now()
                    dt_string = now.strftime("%H_%M_%S_%d_%m_%Y")

                    data = plan.driven_path
                    waypoint_type: WayType = plan.WAYPOINT_TYPE
                    waypoint_str = "NONE"
                    if waypoint_type == 1:
                        waypoint_str = "EARLY"

                    path_id: str = waypoint_str + "_" + dt_string
                    with open(plan.path2data + "/path_" + path_id + ".path", "wb") as fp:
                        pickle.dump(data, fp)

                if viz.menu_item("load driven path"):
                    self.show_file_dialog = True
                viz.end_menu()

            if viz.begin_menu("Show"):
                if viz.menu_item("Show patch map",
                                 selected=self.show_patch):
                    self.show_patch = not self.show_patch
                if viz.menu_item("Show internal heuristics",
                                 selected=self.show_internal_heuristics):
                    self.show_internal_heuristics = not self.show_internal_heuristics
                if viz.menu_item("Show dilated patch",
                                 selected=self.show_dilated_map):
                    self.show_dilated_map = not self.show_dilated_map
                if viz.menu_item("Show collision checks",
                                 selected=self.show_collision_checks):
                    self.show_collision_checks = not self.show_collision_checks
                if viz.menu_item("Show internal states",
                                 selected=self.show_internal_states):
                    self.show_internal_states = not self.show_internal_states
                if viz.menu_item("Show path details",
                                 selected=self.show_path_details):
                    self.show_path_details = not self.show_path_details

                viz.end_menu()

            if not self.pause:
                if viz.button("Pause"):
                    self.pause = not self.pause
            else:
                if viz.button("Continue"):
                    self.pause = not self.pause

            if not self.record:
                if viz.button("Record"):
                    self.record = not self.record
            else:
                if viz.button("Stop"):
                    self.record = not self.record
                    self.proc.terminate()

            viz.end_main_menu_bar()

        # Set data fields for driven path etc. with current data
        self.gather_data(plan)

        if self.show_file_dialog:
            viz.open_popup("fileDialog")

        file_path = viz.file_dialog_popup("fileDialog", plan.path2data)

        if ".path" in file_path:
            self.show_file_dialog = False

            import pickle
            with open(file_path, "rb") as fp:
                path = pickle.load(fp)
                self.add_driven_paths.append((path, file_path.split("/")[-1]))

        self.vis_dilated_collision_map(plan)

        self.vis_collision_checks(minipatches, plan)

        self.vis_patch(plan)

        self.vis_internal_heuristics(plan)

        self.vis_state_details(plan)

        self.vis_vis_parameters(plan, minipatches)

        self.vis_path_details(plan)

        self.vis_cycle_times(plan)

        self.vis_poses_and_control(plan)

        self.idx = self.idx + 1

        viz.save_ini(self.ini_path)

        return self.goal_message, self.pause, self.action_obj
