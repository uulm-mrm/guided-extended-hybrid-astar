import cv2
import scipy.signal
import yaml

import numpy as np
from numpy.typing import NDArray

from path_planner_lib.data_structures import OccEnum


def load_sim_config(share_dir: str) -> dict:
    # Load config data
    filename = share_dir + "/sim_config/sim_config.yml"
    f = open(filename)
    return yaml.safe_load(f)


def transform_binary_to_gm(binary_map: NDArray) -> NDArray:
    kernel = np.ones((3, 3)) * (1 / 9)
    grid_map = scipy.signal.convolve2d(
        binary_map, kernel, fillvalue=OccEnum.UNKNOWN)
    grid_map[grid_map > 100] = 1.0
    grid_map[grid_map > 1] = 2
    grid_map *= OccEnum.UNKNOWN
    grid_map[grid_map > OccEnum.UNKNOWN] = OccEnum.OCC
    return grid_map


def load_png(filename: str, share_dir: str) -> NDArray:
    # Load png data
    path2env = share_dir + "/sim_data/env_models/" + filename + ".png"

    img_arr = cv2.imread(path2env, cv2.IMREAD_UNCHANGED)

    if img_arr is None:
        raise Exception("Img could not be found under: ", path2env)

    if filename == "Maze":
        img_arr = np.flipud(img_arr[:, :, -1])
        # make border
        img_arr[0, :] = OccEnum.OCC
        img_arr[-1, :] = OccEnum.OCC
        img_arr[:, 0] = OccEnum.OCC
        img_arr[:, -1] = OccEnum.OCC
        # threshold
        img_arr[img_arr >= OccEnum.UNKNOWN] = OccEnum.OCC
        img_arr[img_arr < OccEnum.UNKNOWN] = 0
        img_arr = cv2.resize(img_arr, (int(
            img_arr.shape[0] * 1.5), int(img_arr.shape[0] * 1.5)), interpolation=cv2.INTER_NEAREST)
        img_arr = OccEnum.OCC - img_arr
    elif len(img_arr.shape) > 2:
        img_arr = cv2.cvtColor(img_arr[:, :, :3], cv2.COLOR_BGR2GRAY)
        img_arr = np.flipud(img_arr)
    else:
        img_arr = np.flipud(img_arr)

    return OccEnum.OCC - img_arr


def pad_map(grid_map_gt: NDArray, pad_dim: int) -> NDArray:
    """
    Pad grid map with unknown
    :param grid_map_gt:
    :param pad_dim:
    :return:
    """
    width = grid_map_gt.shape[0]
    gm_dim_global = width + 2 * pad_dim
    first_pad = OccEnum.UNKNOWN * np.ones((pad_dim, width))
    second_pad = OccEnum.UNKNOWN * np.ones((gm_dim_global, pad_dim))
    temp = np.vstack((first_pad, grid_map_gt, first_pad))
    # ground truth map
    return np.hstack((second_pad, temp, second_pad)).astype(np.uint8)
