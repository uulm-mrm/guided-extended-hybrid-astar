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


class GoalState(Enum):
    """
    Defines what kind of goal the current one is
    """
    EXACT_GOAL = 0
    APPROX_GOAL = 1


class PlanningState(Enum):
    """
    Defines the replanning state
    """
    CYCLIC = 0
    FORCED = 1


class OverallState:
    def __init__(self):
        self.repl_s = PlanningState.CYCLIC
        self.goal_s = GoalState.EXACT_GOAL
        self.ego_s = EgoOnPathState.ON_PATH
        self.path_s = PathState.SAFE
