# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Utility module that contains functions to handle angels or other non specific operations
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

import itertools
import timeit
from functools import wraps
from math import pi

import numpy as np
import math
import yaml
from typing import Iterable
from numpy.typing import ArrayLike

from .data_structures import PoseDouble


def timeme(original_function=None, N: int = 1):

    def _decorate(function):
        @wraps(function)
        def timeit_wrapper(*args, **kwargs):
            start_time = timeit.default_timer()
            result = None
            for _ in range(N):
                result = function(*args, **kwargs)
            end_time = timeit.default_timer()
            total_time = (end_time - start_time) / N
            print(f'Function {function.__name__} Took {total_time:.6f} s ({N} runs)')
            return result

        return timeit_wrapper

    if original_function is not None:
        return _decorate(original_function)

    return _decorate


def load_lib_config(share_dir: str):
    # Load config data
    filename = share_dir + "/config/config.yml"
    f = open(filename)
    return yaml.safe_load(f)


def get_lib_data_path(share_dir: str) -> str:
    filename = share_dir + "/data"
    return filename


def pairwise(iterable: Iterable):
    """
    s -> (s0, s1), (s1, s2), (s2, s3), ...
    Args:
        iterable: 

    Returns:

    """
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)


def pi_pi2zero_2pi(angle: float) -> float:
    """
    Cast angle from [-pi, pi] to [0, 2*pi]
    Args:
        angle:

    Returns:

    """
    angle = angle % (2*pi)
    if angle < 0:
        angle += 2*pi
    return angle


def angles_equal_0_2pi(angle1: float, angle2: float, tol: float) -> bool:
    """
    Angles must already be in range [0 2pi]
    Args:
        angle1:
        angle2:
        tol:

    Returns:

    """

    angle1 = pi_pi2zero_2pi(angle1)
    angle2 = pi_pi2zero_2pi(angle2)

    angle_diffs = [angle1 - angle2, angle2 - angle1]
    angle_diffs = [diff + 2 * pi if diff < 0 else diff for diff in angle_diffs]
    diff = np.min(np.abs(angle_diffs))

    if diff < np.deg2rad(tol):
        return True
    return False


def angle_diff(a1: float, a2: float) -> float:
    diff = a2 - a1
    if diff > math.pi:
        diff -= 2 * math.pi
    elif diff < -math.pi:
        diff += math.pi
    else:
        pass

    return diff


def get_angle_diffs(yaws: list) -> list:
    return [angle_diff(yaw1, yaw2) for (yaw1, yaw2) in pairwise(yaws)]


def get_curvatures(yaws: list, ds: float) -> list:
    return [angle_diff(yaw1, yaw2) / ds for (yaw1, yaw2) in pairwise(yaws)]


def get_curvature_from_xy(x_vals: list, y_vals: list):
    # Taken from: https://www.delftstack.com/howto/numpy/curvature-formula-numpy/
    try:
        x_t = np.gradient(x_vals)
        y_t = np.gradient(y_vals)
    except ValueError:
        return np.zeros((len(x_vals)))

    xx_t = np.gradient(x_t)
    yy_t = np.gradient(y_t)

    # Formula of the absolute curvature depending on the gradients
    curvature = -(xx_t * y_t - x_t * yy_t) / (x_t * x_t + y_t * y_t) ** 1.5
    return curvature


def get_current_map_lims(ego_gm_patch: PoseDouble, dim: int) -> tuple[list[int], list[int]]:
    offset = int(np.floor(dim / 2))
    origin_x = max(ego_gm_patch.x - offset, 0)
    x_lims = [int(origin_x), int(origin_x + dim)]
    origin_y = max(ego_gm_patch.y - offset, 0)
    y_lims = [int(origin_y), int(origin_y + dim)]

    return x_lims, y_lims


def get_local_map(ego_gm_global: PoseDouble, global_map: ArrayLike, gm_dim: int) -> ArrayLike:
    x_lims, y_lims = get_current_map_lims(ego_gm_global, gm_dim)
    return global_map[y_lims[0]:y_lims[1], x_lims[0]:x_lims[1]]
