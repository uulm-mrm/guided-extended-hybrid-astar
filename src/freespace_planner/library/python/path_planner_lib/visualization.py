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
from path_planner_lib.planning import Vehicle, AStar, HybridAStar, CollisionChecker, Cartographing, UtilCpp, Smoother
from path_planner_lib.planning import PoseDouble, PointDouble, PointInt
from path_planner_lib.logger_setup import Logger
from path_planner_lib.data_structures import GoalMsg, ActionObj, OccEnum, WayType

cmap = matplotlib.cm.get_cmap('turbo')


class Vis:
    def __init__(self):

        # vis handling
        self.t_last_vis: float = 0.0
        self.delta_t_vis: float = 0.0
        fps: float = 30.0
        self.vis_period: float = 1 / fps

        self.initialized: bool = False
        self.ini_path = None

        self.step_vals: Optional[ArrayLike] = None
        self.idx: int = 0
        self.prev_plan_idx: int = -1
        self.pause: bool = False
        self.record: bool = False
        self.t_press: float = 0

        self.action_obj = None

        # Changed on the run for plotting
        self.car_driven_outlines_x: list = []
        self.car_driven_outlines_y: list = []

        self.way_x: list = []
        self.way_y: list = []

        # coordinates of search tree (only start and end to visualize faster)
        self.closed_x: list = []
        self.closed_y: list = []

        self.follow_zoom: float = 4  # the higher, this value the further away is the view
        self.follow_vehicle: bool = True

        self.map_plot_size: tuple = (500, 500)

        self.show_patch: bool = True
        self.show_internal_heuristics: bool = False
        self.show_collision_checks: bool = False
        self.show_internal_states: bool = False
        self.show_path_details: bool = False
        self.show_dilated_map: bool = False
        self.show_tree: bool = True
        self.show_circles: bool = True

        self.show_file_dialog: bool = False
        self.add_driven_paths: list = []

        self.allow_scrolling: bool = False

        self.state_history: list = []
        self.mouse_pos: PointDouble = PointDouble(0, 0)

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
        self.create_geofence: bool = False

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

    def show_vehicle(self, plan):

        veh_x, veh_y = self.get_vehicle_outline(plan.ego_utm)

        track_width: float = plan.veh_config.width - 0.2

        if plan.delta_ego_state is not None:
            delta: float = plan.delta_ego_state
        else:
            delta: float = plan.delta_path

        wheel_bounds = np.array([
            (-0.4, -0.1, 1),
            (0.4, -0.1, 1),
            (0.4, 0.1, 1),
            (-0.4, 0.1, 1),
            (-0.4, -0.1, 1)
        ])

        trans = np.array([
            [np.cos(plan.ego_utm.yaw), -np.sin(plan.ego_utm.yaw), plan.ego_utm.x],
            [np.sin(plan.ego_utm.yaw), np.cos(plan.ego_utm.yaw), plan.ego_utm.y],
            [0.0, 0.0, 1.0]
        ])

        rl_wheel_trans = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, track_width/2.0],
            [0.0, 0.0, 1.0]
        ])

        rr_wheel_trans = rl_wheel_trans.copy()
        rr_wheel_trans[1, 2] -= track_width
        fl_wheel_trans = np.array([
            [np.cos(delta), -np.sin(delta), plan.veh_config.wb],
            [np.sin(delta), np.cos(delta), track_width/2.0],
            [0.0, 0.0, 1.0]
        ])
        fr_wheel_trans = fl_wheel_trans.copy()
        fr_wheel_trans[1, 2] -= track_width

        fl_wheel_trans = trans @ fl_wheel_trans
        fr_wheel_trans = trans @ fr_wheel_trans
        rl_wheel_trans = trans @ rl_wheel_trans
        rr_wheel_trans = trans @ rr_wheel_trans

        rl_wheel_bounds = wheel_bounds @ rl_wheel_trans.T
        rr_wheel_bounds = wheel_bounds @ rr_wheel_trans.T
        fl_wheel_bounds = wheel_bounds @ fl_wheel_trans.T
        fr_wheel_bounds = wheel_bounds @ fr_wheel_trans.T

        viz.plot(veh_x, veh_y,
                 label="vehicle",
                 line_weight=2.0,
                 color="blue")

        viz.plot(rr_wheel_bounds[:, 0], rr_wheel_bounds[:, 1],
                 label="rr",
                 line_weight=2.0,
                 color="blue")

        viz.plot(rl_wheel_bounds[:, 0], rl_wheel_bounds[:, 1],
                 label="rl",
                 line_weight=2.0,
                 color="blue")

        viz.plot(fr_wheel_bounds[:, 0], fr_wheel_bounds[:, 1],
                 label="fr",
                 line_weight=2.0,
                 color="blue")

        viz.plot(fl_wheel_bounds[:, 0], fl_wheel_bounds[:, 1],
                 label="fl",
                 line_weight=2.0,
                 color="blue")

    @staticmethod
    def get_vehicle_outline(pose: PointDouble) -> tuple[list, list]:

        vrx, vry = [point.x for point in Vehicle.vis_vehicle_vertices_], [point.y for point in
                                                                          Vehicle.vis_vehicle_vertices_]
        yaw_rot: float = -pose.yaw
        rot = np.array([[math.cos(yaw_rot), -math.sin(yaw_rot)],
                        [math.sin(yaw_rot), math.cos(yaw_rot)]])

        converted_xy = np.stack([vrx, vry]).T @ rot

        return converted_xy[:, 0] + pose.x, converted_xy[:, 1] + pose.y

    def set_xaxis_settings4states(self, plan):
        viz.setup_axis(viz.Axis.X1, "steps")
        viz.setup_axis_limits(viz.Axis.X1, -plan.history_len, 0,
                              flags=viz.PlotCond.ALWAYS)

    def enable_goal_change(self):
        viz.push_override_id(viz.get_plot_id())
        if viz.begin_popup("##PlotContext"):

            self.map_context_menu_open = True

            if viz.menu_item("Set Goal"):
                self.goal_pose_utm = PoseDouble(self.mouse_pos.x, self.mouse_pos.y, self.goal_pose_utm.yaw)
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
        goal_outline_x, goal_outline_y = self.get_vehicle_outline(self.goal_pose_utm)
        viz.plot(goal_outline_x, goal_outline_y, line_weight=3, fmt="-", color="orange")

        # Plot selected goal
        if plan.goal_utm is not None:
            goal_outline_x, goal_outline_y = self.get_vehicle_outline(plan.goal_utm)
            viz.plot(goal_outline_x, goal_outline_y, line_weight=3, fmt="-", color="red",
                     label="goal")

    def plot_circles(self, plan, only_disks: bool = False):
        """
        Plot circles with radi of the dilation disk
        :param plan:
        :return:
        """
        points = CollisionChecker.returnDiskPositions(plan.ego_utm.yaw)

        viz.plot([p.x + plan.ego_utm.x for p in points], [p.y + plan.ego_utm.y for p in points], marker_size=2.0,
                 fmt=".o", color="red")

        if not only_disks:
            for p in points:
                viz.plot_circle([p.x + plan.ego_utm.x, p.y + plan.ego_utm.y], CollisionChecker.disk_r_, color="red")

        if self.goal_pose_utm is not None:
            points = CollisionChecker.returnDiskPositions(self.goal_pose_utm.yaw)

            viz.plot([p.x + self.goal_pose_utm.x for p in points], [p.y + self.goal_pose_utm.y for p in points],
                     marker_size=2.0, fmt=".o", color="red")

            if not only_disks:
                for p in points:
                    viz.plot_circle([p.x + self.goal_pose_utm.x, p.y + self.goal_pose_utm.y], CollisionChecker.disk_r_,
                                    color="red")

    def plot_search_tree(self):
        """
        Plot visited/closed nodes of the hybrid a star search
        :return:
        """

        if len(HybridAStar.connected_closed_nodes_[0]) != 0:
            self.closed_x, self.closed_y = HybridAStar.connected_closed_nodes_

        if self.closed_x:
            viz.plot(self.closed_x, self.closed_y, marker_size=1, fmt="-", color="red", label="closed nodes",
                     flags=viz.PlotLineFlags.SEGMENTS)

    def enable_and_follow_vehicle(self, plan):
        self.enable_follow_veh()
        if self.follow_vehicle:

            if viz.is_window_hovered():
                for se in viz.get_scroll_events():
                    if se.yoffset < 0:
                        self.follow_zoom *= abs(se.yoffset) * 1.1
                    elif se.yoffset > 0:
                        self.follow_zoom /= abs(se.yoffset) * 1.1

                if not (viz.get_mouse_drag_delta() == 0.0).all():
                    self.follow_vehicle = False

            d = 10 * self.follow_zoom

            aspect_ratio = self.map_plot_size[1] / self.map_plot_size[0]
            plot_x_min = plan.ego_utm.x - d
            plot_x_max = plan.ego_utm.x + d
            plot_y_min = plan.ego_utm.y - d * aspect_ratio
            plot_y_max = plan.ego_utm.y + d * aspect_ratio

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

                    viz.setup_axis(viz.Axis.X1, "x in m")
                    viz.setup_axis(viz.Axis.Y1, "y in m")

                    self.enable_and_follow_vehicle(plan)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    if not self.map_context_menu_open:
                        pos = viz.get_plot_mouse_pos()
                        self.mouse_pos = PointDouble(pos[0], pos[1])

                    patch_safety = CollisionChecker.patch_safety_arr_.getNumpyArr()
                    offset = plan.patch_info.origin_utm
                    viz.plot_image("Dilated patch",  1 - (patch_safety / 2),
                                   x=offset.x,
                                   y=offset.y,
                                   width=plan.patch_info.dim_utm,
                                   height=plan.patch_info.dim_utm,
                                   uv0=(0, 1), uv1=(1, 0), interpolate=False)

                    self.show_pixel_value(patch_safety, offset)
                    self.plot_circles(plan, only_disks=True)

                    if plan.path is not None:
                        if plan.path.x_list:
                            viz.plot(plan.path_utm.x_list, plan.path_utm.y_list, line_weight=2, color="blue", fmt="-o", label="planned path")

                    viz.plot(plan.driven_path.x_list, plan.driven_path.y_list, line_weight=2, marker_size=1.0,
                             color="black",
                             fmt="-",
                             label="driven path")

                    self.show_vehicle(plan)

                    self.plot_selected_and_current_goal(plan)
                    self.enable_goal_change()

                    viz.end_plot()
            viz.end_window()

    def vis_collision_checks(self, plan):
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
                    viz.setup_axis(viz.Axis.X1, "x in m")
                    viz.setup_axis(viz.Axis.Y1, "y in m")

                    self.enable_and_follow_vehicle(plan)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    patch = CollisionChecker.patch_arr_.getNumpyArr()
                    offset = plan.patch_info.origin_utm
                    viz.plot_image("Patch", (OccEnum.OCC - patch),
                                   x=offset.x,
                                   y=offset.y,
                                   width=plan.patch_info.dim_utm,
                                   height=plan.patch_info.dim_utm,
                                   uv0=(0, 1), uv1=(1, 0), interpolate=False)
                    self.show_pixel_value(patch, offset)

                    # on len change
                    if len(plan.driven_path.x_list) > len(self.car_driven_outlines_x) * 6:
                        self.car_driven_outlines_x = []
                        self.car_driven_outlines_y = []
                        for i, (x, y, yaw) in enumerate(zip(plan.driven_path.x_list, plan.driven_path.y_list,
                                                            plan.driven_path.yaw_list)):
                            if i % 4 != 0:
                                continue
                            car_outline_x, car_outline_y = self.get_vehicle_outline(PoseDouble(x, y, yaw))
                            self.car_driven_outlines_x.append(car_outline_x)
                            self.car_driven_outlines_y.append(car_outline_y)

                    for i, (outline_x, outline_y) in enumerate(
                            zip(self.car_driven_outlines_x, self.car_driven_outlines_y)):
                        viz.plot(outline_x, outline_y, line_weight=2, color="blue", fmt="-", label="vehicle")

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

                    viz.setup_axis(viz.Axis.X1, "x in m")
                    viz.setup_axis(viz.Axis.Y1, "y in m")

                    self.enable_and_follow_vehicle(plan)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    if not self.map_context_menu_open:
                        pos = viz.get_plot_mouse_pos()
                        self.mouse_pos = PointDouble(pos[0], pos[1])

                    patch = CollisionChecker.patch_arr_.getNumpyArr()
                    offset = plan.patch_info.origin_utm
                    viz.plot_image("Patch", OccEnum.OCC - patch,
                                   x=offset.x,
                                   y=offset.y,
                                   width=plan.patch_info.dim_utm,
                                   height=plan.patch_info.dim_utm,
                                   uv0=(0, 1), uv1=(1, 0), interpolate=False)
                    self.show_pixel_value(patch, offset)

                    if self.show_tree:
                        self.plot_search_tree()

                    if self.create_lane_graph:
                        if self.mouse_pos is not None:
                            if viz.is_window_hovered():
                                if viz.is_mouse_clicked():
                                    HybridAStar.lane_graph_.addPoint(self.mouse_pos)

                    if self.create_geofence:
                        if viz.is_window_hovered():
                            if viz.is_mouse_clicked():
                                AStar.restr_geofence_.addPoint(self.mouse_pos)

                    # show edges of lane graph
                    for i_edge, edge in enumerate(HybridAStar.lane_graph_.edges_):
                        points = [node_opt.point_utm for node_opt in edge if node_opt]
                        xs = [point.x for point in points]
                        ys = [point.y for point in points]

                        color = "black"
                        line_weight = 1
                        viz.plot(xs, ys, fmt="-o", color=color, line_weight=line_weight)

                    self.plot_selected_and_current_goal(plan)

                    viz.plot([plan.ego_utm.x], [plan.ego_utm.y], fmt="o", color="blue")

                    if self.show_circles:
                        self.plot_circles(plan)

                    # Get astar path
                    if plan.config.WAYPOINT_TYPE != WayType.NONE:
                        if self.way_x:
                            viz.plot(self.way_x, self.way_y, marker_size=2, color="blue", fmt="-o", label="A* path")

                    if plan.path_utm is not None:
                        if plan.path_utm.x_list:
                            viz.plot(plan.path_utm.x_list, plan.path_utm.y_list, line_weight=2, color="blue",
                                     marker_size=2, fmt="-o", label="path")

                    if plan.coll_point is not None:
                        coll_point = UtilCpp.patch_utm2utm(plan.coll_point)
                        viz.plot([coll_point.x], [coll_point.y], marker_size=10, color="red", fmt=".x")
                        # self.pause = True

                    viz.plot(plan.driven_path.x_list, plan.driven_path.y_list, line_weight=2, color="gray",
                             marker_size=2, fmt="-o",
                             label="driven path")

                    self.show_vehicle(plan)

                    x = [vertice.x for vertice in AStar.restr_geofence_.vertices]
                    if x:
                        x.append(x[0])
                    y = [vertice.y for vertice in AStar.restr_geofence_.vertices]
                    if y:
                        y.append(y[0])
                    viz.plot(x, y, line_weight=3, color="red", fmt="-", label="restr_geofence_")

                    if plan.plan_start_node is not None:
                        x, y = UtilCpp.patch_utm2utm([plan.plan_start_node.x_list[0]], [plan.plan_start_node.y_list[0]])
                        viz.plot(x, y, line_weight=3, color="red", fmt="o", label="plan start node")

                    # Create video
                    if self.record:
                        self.do_record(plan)

                    self.enable_goal_change()

                    viz.end_plot()
                viz.end_window()

    def do_record(self, plan):
        if plan.sim_idx > self.prev_plan_idx and plan.sim_idx > 5:

            # if not self.goal_reached and plan.overall_state.ego_s == EgoOnPathState.GOAL:
            #     self.goal_reached = True
            #     self.last_iterations = 50

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

                if plan.config.WAYPOINT_TYPE == WayType.NONE:
                    pad_dim = 10
                    pad = np.zeros((self.frame_height, pad_dim, 3), dtype=np.uint8)
                    frame = np.hstack((frame, pad))
                    self.frame_width += pad_dim

                if not self.rec_process_started:
                    self.rec_process_started = True
                    stamp = datetime.now().strftime("%d%m%Y_%H%M%S")
                    if plan.config.WAYPOINT_TYPE == 1:
                        scenario_name = "neu-ulm-standard-unknown_" + str(stamp)
                    else:
                        scenario_name = "neu-ulm-guided-unknown_" + str(stamp)
                    self.proc = subprocess.Popen(
                        self.ffmpeg_cmd(self.frame_width,
                                        self.frame_height,
                                        f"/home/schumann/mrm/projects/sandboxes/aduulm_sandbox/src/freespace_planner/videos/{scenario_name}.mp4"),
                        stdin=subprocess.PIPE,
                        stdout=None,
                        stderr=None)
                self.proc.stdin.write(frame.tobytes())

    def show_pixel_value(self, arr, offset=PointDouble(0, 0), kind: str="gm"):
        mouse_pos = viz.get_plot_mouse_pos()
        mouse_pos = PointDouble(mouse_pos[0], mouse_pos[1])

        # Shift mouse pos to arr
        mouse_pos.x -= offset.x
        mouse_pos.y -= offset.y
        if kind == "gm":
            mouse_pos = UtilCpp.utm2grid(mouse_pos)
        elif kind == "astar":
            mouse_pos = UtilCpp.utm2astar(mouse_pos)
        else:
            self.logger.log_error("unknown kind to show pixel value")
            return

        val = None
        if mouse_pos is not None:
            if 0 <= mouse_pos.x < arr.shape[0] and 0 <= mouse_pos.y < arr.shape[1]:
                y_ind = max(int(np.floor(mouse_pos.y)), 0)
                x_ind = max(int(np.floor(mouse_pos.x)), 0)
                val = arr[y_ind, x_ind]
        viz.text("Pixel value: " + str(val))

    def vis_internal_heuristics(self, plan):
        if self.show_internal_heuristics:

            # # heuristic Map
            # window_title = "hprox distance map"
            # if viz.begin_window(
            #         window_title,
            #         # size=(400, 400),
            #         # position=(700, 400),
            #         resize=True,
            # ):
            #     self.show_internal_heuristics = viz.get_window_open()
            #
            #     if viz.begin_plot(window_title, flags=viz.PlotFlags.EQUAL):
            #         viz.setup_axis(viz.Axis.X1, "x in m")
            #         viz.setup_axis(viz.Axis.Y1, "y in m")
            #
            #         if viz.plot_selection_ended():
            #             viz.hard_cancel_plot_selection()
            #
            #         dist_map = AStar.hprox_dist_.getNumpyArr()
            #         offset = plan.patch_info.origin_utm
            #         self.show_pixel_value(dist_map, offset, kind="astar")
            #         offset = plan.patch_info.origin_utm
            #         viz.plot_image(window_title, 1-dist_map,
            #                        x=offset.x,
            #                        y=offset.y,
            #                        width=190 * HybridAStar.ASTAR_RES,
            #                        height=190 * HybridAStar.ASTAR_RES,
            #                        uv0=(0, 1), uv1=(1, 0), interpolate=False)
            #         viz.end_plot()
            # viz.end_window()

            # heuristic Map
            window_title = "voronoi distance map"
            if viz.begin_window(
                    window_title,
                    # size=(400, 400),
                    # position=(700, 400),
                    resize=True,
            ):
                self.show_internal_heuristics = viz.get_window_open()

                if viz.begin_plot(window_title, flags=viz.PlotFlags.EQUAL):
                    viz.setup_axis(viz.Axis.X1, "x in m")
                    viz.setup_axis(viz.Axis.Y1, "y in m")

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    dist_map = AStar.voronoi_dist_.getNumpyArr()
                    offset = plan.patch_info.origin_utm
                    self.show_pixel_value(dist_map, offset, kind="astar")
                    offset = plan.patch_info.origin_utm
                    viz.plot_image(window_title, dist_map/10,
                                   x=offset.x,
                                   y=offset.y,
                                   width=190 * HybridAStar.ASTAR_RES,
                                   height=190 * HybridAStar.ASTAR_RES,
                                   uv0=(0, 1), uv1=(1, 0), interpolate=False)
                    viz.end_plot()
            viz.end_window()

            # heuristic Map
            window_title = "obstacle distance map"
            if viz.begin_window(
                    window_title,
                    # size=(400, 400),
                    # position=(700, 400),
                    resize=True,
            ):
                self.show_internal_heuristics = viz.get_window_open()

                if viz.begin_plot(window_title, flags=viz.PlotFlags.EQUAL):
                    viz.setup_axis(viz.Axis.X1, "x in m")
                    viz.setup_axis(viz.Axis.Y1, "y in m")

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    dist_map = AStar.obstacle_dist_.getNumpyArr()
                    offset = plan.patch_info.origin_utm
                    self.show_pixel_value(dist_map, offset, kind="astar")
                    offset = plan.patch_info.origin_utm
                    viz.plot_image(window_title, dist_map/10,
                                   x=offset.x,
                                   y=offset.y,
                                   width=190 * HybridAStar.ASTAR_RES,
                                   height=190 * HybridAStar.ASTAR_RES,
                                   uv0=(0, 1), uv1=(1, 0), interpolate=False)
                    viz.end_plot()
            viz.end_window()

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
                    viz.setup_axis(viz.Axis.X1, "x in m")
                    viz.setup_axis(viz.Axis.Y1, "y in m")

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    # visualize map
                    cmap = matplotlib.cm.get_cmap('jet_r')
                    heur_map_gray = -np.ones((HybridAStar.astar_dim_, HybridAStar.astar_dim_), dtype=np.float64)

                    # TODO (Schumann) do all this in cpp
                    if AStar.closed_set_guidance_ is not None:
                        for obj in AStar.closed_set_guidance_:
                            el = obj[1]
                            heur_map_gray[el.pos.y, el.pos.x] = el.cost_dist_
                    # normalise and color code
                    max_val = np.max(heur_map_gray)
                    heur_map_gray[heur_map_gray == -1] = max_val
                    heur_map_gray_norm = heur_map_gray / max_val
                    heur_map_color = cmap(heur_map_gray_norm)[:, :, 0:3]

                    offset = plan.patch_info.origin_utm
                    self.show_pixel_value(heur_map_gray, offset, kind="astar")
                    viz.plot_image(window_title, heur_map_color,
                                   x=offset.x,
                                   y=offset.y,
                                   width=plan.patch_info.dim_utm,
                                   height=plan.patch_info.dim_utm,
                                   uv0=(0, 1), uv1=(1, 0), interpolate=False)

                    if self.way_x:
                        viz.plot(self.way_x, self.way_y, marker_size=2, color="blue", fmt="-o", label="A* path")

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
                    viz.setup_axis(viz.Axis.X1, "x m")
                    viz.setup_axis(viz.Axis.Y1, "y m")

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    motion_res_map = AStar.motion_res_map_.getNumpyArr()

                    if motion_res_map is not None:
                        offset = plan.patch_info.origin_utm
                        self.show_pixel_value(motion_res_map, offset, kind="astar")
                        viz.plot_image(window_title, motion_res_map,
                                       x=offset.x,
                                       y=offset.y,
                                       width=plan.patch_info.dim_utm,
                                       height=plan.patch_info.dim_utm,
                                       uv0=(0, 1), uv1=(1, 0), interpolate=False)

                    if self.way_x:
                        viz.plot(self.way_x, self.way_y, marker_size=2, color="blue", fmt="-o", label="A* path")

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
                    viz.setup_axis(viz.Axis.X1, "x in m")
                    viz.setup_axis(viz.Axis.Y1, "y in m")

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    movement_cost_map = AStar.movement_cost_map_.getNumpyArr()

                    if movement_cost_map is not None:
                        offset = plan.patch_info.origin_utm
                        self.show_pixel_value(movement_cost_map, offset, kind="astar")
                        viz.plot_image(window_title, movement_cost_map / 5,
                                       x=offset.x,
                                       y=offset.y,
                                       width=plan.patch_info.dim_utm,
                                       height=plan.patch_info.dim_utm,
                                       uv0=(0, 1), uv1=(1, 0), interpolate=False)

                    if self.way_x:
                        viz.plot(self.way_x, self.way_y, marker_size=2, color="blue", fmt="-o", label="A* path")

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
                    viz.setup_axis(viz.Axis.X1, "x in m")
                    viz.setup_axis(viz.Axis.Y1, "y in m")

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    astar_grid = AStar.astar_grid_.getNumpyArr()
                    offset = plan.patch_info.origin_utm
                    self.show_pixel_value(astar_grid, offset, kind="astar")
                    viz.plot_image(window_title, 1 - astar_grid / 2,
                                   x=offset.x,
                                   y=offset.y,
                                   width=plan.patch_info.dim_utm,
                                   height=plan.patch_info.dim_utm,
                                   uv0=(0, 1), uv1=(1, 0), interpolate=False)

                    if self.way_x:
                        viz.plot(self.way_x, self.way_y, marker_size=2, color="blue", fmt="-o", label="A* path")

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
                    viz.setup_axis(viz.Axis.X1, "x in m")
                    viz.setup_axis(viz.Axis.Y1, "y in m")

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    h_prox = AStar.h_prox_arr_.getNumpyArr()
                    if h_prox is not None:
                        offset = plan.patch_info.origin_utm
                        self.show_pixel_value(h_prox, offset, kind="astar")
                        viz.plot_image(window_title, 1 - h_prox,
                                       x=offset.x,
                                       y=offset.y,
                                       width=plan.patch_info.dim_utm,
                                       height=plan.patch_info.dim_utm,
                                       uv0=(0, 1), uv1=(1, 0), interpolate=False)

                    if self.way_x:
                        viz.plot(self.way_x, self.way_y, line_weight=3, color="blue", fmt="-o", label="A* path")

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
                viz.text(plan.overall_state.ego_s)
                viz.text("plan.new_goal: " + str(plan.new_goal))
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
            ego_states = [el.ego_s.value for el in self.state_history]

            step_vals = np.arange(-len(path_states), 0)

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

    def vis_vis_parameters(self, plan):
        window_title = "Vis"
        if viz.begin_window(
                window_title,
        ):
            viz.text("vis fps: " + str(round(1 / self.delta_t_vis)))

            plan.history_len = viz.input("history_len", plan.history_len)
            viz.same_line()
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

                        viz.plot(self.step_vals, driven_yaw, color="red", fmt="-o")

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
                    viz.setup_axis_limits(viz.Axis.Y1, -2 * math.pi, 2 * math.pi, flags=viz.PlotCond.ALWAYS)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    if plan.path is not None:
                        viz.plot(plan.path.yaw_list, color="red", fmt="-o")

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

            window_title = "normals(planned)"
            if viz.begin_window(
                    window_title,
            ):
                if viz.begin_plot(window_title):

                    viz.setup_axis(viz.Axis.X1, "X")
                    if plan.path is not None:
                        viz.setup_axis_limits(viz.Axis.X1, 0, len(plan.path.direction_list),
                                              flags=viz.PlotCond.ALWAYS)

                    viz.setup_axis(viz.Axis.Y1, "Y")
                    viz.setup_axis_limits(viz.Axis.Y1, -10, 10, flags=viz.PlotCond.ALWAYS)

                    if viz.plot_selection_ended():
                        viz.hard_cancel_plot_selection()

                    viz.end_plot()
                viz.end_window()

    def vis_cycle_times(self, plan):
        window_title = "Cycle times"
        if viz.begin_window(window_title):
            if viz.begin_plot(window_title):

                self.set_xaxis_settings4states(plan)

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

            if viz.tree_node("Geofence control"):
                if not self.create_geofence:
                    if viz.button("create geofence"):
                        AStar.restr_geofence_.reset()
                        self.create_geofence = True
                else:
                    if viz.button("save geofence"):
                        AStar.processGeofence()
                        self.create_geofence = False
                viz.tree_pop()

            viz.text("A* params")
            AStar.alpha_ = viz.drag("alpha_", AStar.alpha_)
            AStar.do_max_ = viz.drag("do_max_", AStar.do_max_)
            AStar.do_min_ = viz.drag("do_min_", AStar.do_min_)
            AStar.astar_prox_cost_ = viz.drag("astar_prox_cost_", AStar.astar_prox_cost_)
            AStar.astar_movement_cost_ = viz.drag("astar_movement_cost_", AStar.astar_movement_cost_)
            AStar.astar_lane_movement_cost_ = viz.drag("astar_lane_movement_cost_", AStar.astar_lane_movement_cost_)

            viz.text("Hybrid A* params")
            HybridAStar.switch_cost_ = viz.drag("switch_cost_", HybridAStar.switch_cost_)
            HybridAStar.steer_cost_ = viz.drag("steer_cost_", HybridAStar.steer_cost_)
            HybridAStar.steer_change_cost_ = viz.drag("steer_change_cost_", HybridAStar.steer_change_cost_)
            HybridAStar.h_dist_cost_ = viz.drag("h_dist_cost_", HybridAStar.h_dist_cost_)
            HybridAStar.back_cost_ = viz.drag("back_cost_", HybridAStar.back_cost_)
            HybridAStar.h_prox_cost_ = viz.drag("h_prox_cost_", HybridAStar.h_prox_cost_)

            # viz.text("Smoother params")
            # Smoother.max_iter_ = viz.drag("max_iter_", Smoother.max_iter_)
            # Smoother.wSmoothness_ = viz.drag("wSmoothness_", Smoother.wSmoothness_)
            # Smoother.wObstacle_ = viz.drag("wObstacle_", Smoother.wObstacle_)
            # Smoother.wCurvature_ = viz.drag("wCurvature_", Smoother.wCurvature_)
            # Smoother.alpha_ = viz.drag("alpha_", Smoother.alpha_)

        viz.end_window()

    @staticmethod
    def save_graph(plan):
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

        self.state_history.append(deepcopy(plan.overall_state))

        self.way_x = []
        self.way_y = []
        if plan.astar_path:
            way_x, way_y = UtilCpp.astar2utm([plan.astar_path[0], plan.astar_path[1]])
            self.way_x, self.way_y = UtilCpp.patch_utm2utm(way_x, way_y)

    def map_handling(self, save):
        self.action_obj = ActionObj()
        if save:
            self.action_obj.save_map()
            self.logger.log_info("Save map triggered from vis")
        else:
            self.action_obj.load_map()
            self.logger.log_info("Load map triggered from vis")

    def render_data(self, plan, show_vis: bool = True):
        delta_t_vis, new_timestamp = util.set_and_measure(self.t_last_vis)
        if delta_t_vis < self.vis_period:
            return None, self.pause, self.action_obj

        self.delta_t_vis = delta_t_vis
        self.t_last_vis = new_timestamp

        # Errors in visualization should be ignored
        # try:
        self.initialize()

        main_title = "freespace planner"
        if show_vis:
            viz.set_main_window_title(main_title)

            if self.idx == 0:
                viz.set_main_window_size((1920, 1080))
                viz.set_main_window_pos((0, 0))
        else:
            title = main_title + "(deactivated)"
            viz.set_main_window_title(title)
            viz.set_main_window_size((10, 10))
            viz.set_main_window_pos((0, 0))
            self.idx = 0

        # Only continue if patch_info is set
        if not viz.wait(vsync=False) or not show_vis or plan.patch_info is None or plan.ego_utm_patch is None:
            time.sleep(0.1)
            return None, self.pause, self.action_obj

        nb_history_els = min(plan.history_len, len(plan.driven_path.x_list))
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
                    waypoint_type: WayType = plan.config.WAYPOINT_TYPE
                    waypoint_str = "NONE"
                    if waypoint_type == 0:
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

        self.vis_patch(plan)

        self.vis_collision_checks(plan)

        self.vis_internal_heuristics(plan)

        self.vis_state_details(plan)

        self.vis_vis_parameters(plan)

        self.vis_path_details(plan)

        self.vis_cycle_times(plan)

        self.vis_poses_and_control(plan)

        self.idx = self.idx + 1

        # except Exception as e:
        #     print(termcolor.colored("Catched error in vis thread, staying active:\n" + str(e), "red"))

        viz.save_ini(self.ini_path)

        return self.goal_message, self.pause, self.action_obj
