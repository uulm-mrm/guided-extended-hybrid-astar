# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Utility module that contains functions to handle angels or other non specific operations
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

import itertools
import sys
import timeit
from functools import wraps
from math import pi
import time

import numpy as np
import math
import yaml
from typing import Iterable
from numpy.typing import ArrayLike

from .data_structures import PoseDouble


def set_and_measure(timestamp: float) -> tuple[float, float]:
    new_timestamp: float = time.perf_counter()
    delta_time: float = new_timestamp - timestamp
    return delta_time, new_timestamp


def timeme(original_function=None, N: int = 1, logger=None):

    def _decorate(function):
        @wraps(function)
        def timeit_wrapper(*args, **kwargs):
            start_time = timeit.default_timer()
            result = None
            for _ in range(N):
                result = function(*args, **kwargs)
            end_time = timeit.default_timer()
            total_time = (end_time - start_time) / N
            if logger is None:
                print(f'Function {function.__name__} Took {total_time:.6f} s ({N} runs)')
            else:
                logger.info(f'Function {function.__name__} Took {total_time:.6f} s ({N} runs)')
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


def load_lib_params(share_dir: str):
    # Load param data
    filename = share_dir + "/config/params.yml"
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


def angles_equal(angle1: float, angle2: float, abs_tol: float) -> bool:
    """
    Args:
        angle1:
        angle2:
        abs_tol:

    Returns:

    """

    diff: float = angle_diff(angle1, angle2)

    if abs(diff) < np.deg2rad(abs_tol):
        return True
    return False


def constrain_angle_minpi_pluspi(angle: float) -> float:
    """
    Angles are casted to [-pi, pi)
    :param angle:
    :return:
    """
    return (angle + math.pi) % (2*math.pi) - math.pi


def constrain_angle_minpi_pluspi_fmod(angle: float) -> float:
    """
    Angles are cast to [-pi, pi) using fmod to debug for cpp development
    :param angle:
    :return:
    """

    while angle < 0:
        angle += 2 * math.pi

    return math.fmod(angle+math.pi, 2*math.pi) - math.pi


def angle_diff(a1: float, a2: float) -> float:
    """
    :param a1:
    :param a2:
    :return:
    """

    # ensure [-pi, pi)
    a2 = constrain_angle_minpi_pluspi(a2)
    a1 = constrain_angle_minpi_pluspi(a1)

    diff = a2 - a1

    return constrain_angle_minpi_pluspi(diff)


def get_curvatures(yaws: list, ds: float) -> list:
    return [angle_diff(yaw1, yaw2) / ds for (yaw1, yaw2) in pairwise(yaws)]


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
