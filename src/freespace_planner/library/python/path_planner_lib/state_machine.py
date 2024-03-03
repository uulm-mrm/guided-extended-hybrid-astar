from enum import Enum


class EgoOnPathState(Enum):
    """
    Defines where the current vehicle is on the path
    """
    GOAL = 0
    ON_PATH = 1
    END_OF_PATH = 2


class PathState(Enum):
    """
    Defines if the path is SAFE or will collide
    """
    SAFE = 0
    COLLIDES = 1


class OverallState:
    def __init__(self):
        self.ego_s = EgoOnPathState.ON_PATH
        self.path_s = PathState.SAFE
