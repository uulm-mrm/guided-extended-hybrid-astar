from collections import deque
from enum import IntEnum

from hybridastar_planning_lib import Path, NodeHybrid, NodeDisc, Vehicle, PoseDouble, PointDouble, PointInt, Minipatch


class OccEnum(IntEnum):
    FREE = 0
    UNKNOWN = 127
    OCC = 255


class WayType(IntEnum):
    ASTAR_PATH = 0
    HEUR_RED = 1
    NONE = 2


class VehConfig:
    """
    Default config set from Ul1500 for simulation
    """
    def __init__(self, max_steer: float = 0.55, width: float = 2.13, length: float = 5.255, wb: float = 3.165,
                 lf: float = 3.993, lb: float = 1.17, lb_extra: float = 2.17, is_ushift: bool = False,
                 has_capsule: bool = False):
        self.max_steer: float = max_steer
        self.width: float = width
        self.length: float = length
        self.wb: float = wb
        self.lf: float = lf
        self.lb: float = lb
        self.lb_extra: float = lb_extra
        self.is_ushift: bool = is_ushift
        self.has_capsule: bool = has_capsule


class PathHistory:
    def __init__(self, history_len: int):
        self.history_len = history_len
        self.x_list = deque(maxlen=self.history_len)
        self.y_list = deque(maxlen=self.history_len)
        self.yaw_list = deque(maxlen=self.history_len)
        self.direction_list = deque(maxlen=self.history_len)
        self.cost_list = deque(maxlen=self.history_len)
        self.has_capsule = deque(maxlen=self.history_len)

    def set_max_len(self, history_len: int) -> None:
        if self.x_list.maxlen != history_len:
            self.history_len = history_len
            self.x_list = deque(self.x_list, maxlen=self.history_len)
            self.y_list = deque(self.y_list, maxlen=self.history_len)
            self.yaw_list = deque(self.yaw_list, maxlen=self.history_len)
            self.direction_list = deque(self.direction_list, maxlen=self.history_len)
            self.cost_list = deque(self.cost_list, maxlen=self.history_len)
            self.has_capsule = deque(self.has_capsule, maxlen=self.history_len)


class GoalMsg:
    def __init__(self, pose: PoseDouble, remove: bool):
        self.pose = pose
        self.remove = remove


class ActionObj:
    def __init__(self):
        self.map_action = None

    def load_map(self):
        self.map_action = "load"

    def save_map(self):
        self.map_action = "save"


class PatchInfo:
    def __init__(self, origin_utm: PointDouble, dim: int, dim_utm: int):
        self.origin_utm: PointDouble = origin_utm
        self.dim: int = dim
        self.dim_utm: int = dim_utm