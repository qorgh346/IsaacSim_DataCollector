# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import copy
import time
import numpy as np
from omni.isaac.demos.utils import math_utils


def get_block_id(color, id_tag="00"):
    """Get block identifier based on color
    
    Args:
        color (string): string representation of color
        id_tag (string, optional): prefix for the object. Defaults to "00".
    
    Returns:
        string: block identifier as string
    """
    return "%s_block_%s" % (id_tag, color)


def get_block_name(index, block_colors):
    """Get block name
    
    Args:
        index (int): index of block id to get
        block_colors (list[string]): list of block colors
    
    Returns:
        [type]: [description]
    """
    return get_block_id(block_colors[index])


def project_block_transform_to_table(T_unprojected, block_height):
    """Project block transform onto table from given height
    """
    T = copy.deepcopy(T_unprojected)
    R = T[0:3, 0:3]
    v = T[0:3, 3]

    up = np.array([0.0, 0.0, 1.0])
    R = math_utils.proj_to_align(R, up)
    v[2] = block_height / 2

    T[0:3, 0:3] = R
    T[0:3, 3] = v
    return T


def natural_push_axis_y(orig, direction_xy):
    """ Returns the y-axis constraint which would be most natural for the specified horizontal push
    direction starting from the given orig.

    direction_xy should be the x and y components of the horizontal direction of motion, assuming
    the z component would be zero. Note that direction_xy can be three dimensional, just the final
    dimension will be ignored.
    """
    down = np.array([0.0, 0.0, -1.0])
    d = np.array([direction_xy[0], direction_xy[1], 0.0])
    proposed_y = np.cross(d, down)
    # if np.dot(orig, d) > 0:
    forward = np.array([1.0, 0.0, 0.0])
    if np.dot(forward, d) > 0:
        return proposed_y
    else:
        return -proposed_y


class ApproachParams(object):
    """Compute approach for a pose
    """

    def __init__(self, direction, standoff=0.1, standoff_std_dev=0.02):
        self.direction = direction
        self.standoff = standoff
        self.standoff_std_dev = standoff_std_dev


def set_gripper_to_push_width(franka):
    franka.end_effector.gripper.move(width=0.025, speed=0.2, wait=False)


def close_open(franka, block_height):
    """Close then open robot grippers
    """
    franka.end_effector.gripper.move(width=0.975 * block_height, speed=0.2, wait=True)
    time.sleep(0.5)
    franka.end_effector.gripper.open(wait=False)
    time.sleep(0.5)


# The pick info assumes the block is on a table and we're picking from above along the world (robot
# base coordinates) z-axis. The block_orig is the origin of the block and the pinch_axis defines the
# axis the robot will be pinching along. The grasp target is derived from this information.
#
# block_height is the height of the block and pinch_depth is how far from the top the robot will
# pinch.
#
# By default, the approach direction is chosen to be from the top and the z-axis of the end-effector
# is constrained to that direction. Both of those can be customized on construction, and any data
# member can be modified manually post-construction.
class PickInfo(object):
    def __init__(
        self,
        block_orig,
        pinch_axis,
        block_height,
        pinch_depth,
        approach_direction=np.array([0.0, 0.0, -1.0]),
        constraint_axis_z_along_approach=True,
    ):
        self.block_height = block_height
        self.pinch_depth = pinch_depth

        self.target_orig = copy.deepcopy(block_orig)
        self.target_orig[2] = block_orig[2] + block_height / 2 - pinch_depth
        self.target_axis_y = pinch_axis
        self.target_axis_z = None

        self.approach_direction = approach_direction
        if constraint_axis_z_along_approach:
            self.target_axis_z = approach_direction
            self.target_axis_x = np.cross(self.target_axis_y, self.target_axis_z)
        else:
            self.target_axis_z = None
            self.target_axis_x = None

        # If the x-axis target isn't facing toward the robot's base, flip it around so it is.
        away_from_robot = block_orig
        away_from_robot[2] = 0.0
        orth_to_from_robot = np.array([-away_from_robot[1], away_from_robot[0], 0.0])
        if np.dot(self.target_axis_y, orth_to_from_robot) < 0.0:
            self.target_axis_y = -self.target_axis_y
            if self.target_axis_x is not None:
                self.target_axis_x = -self.target_axis_x

    def update(self, pick_info):
        for k, v in pick_info.__dict__.items():
            self.__dict__[k] = v


def go_home(franka, config=None):
    """return robot to its retracted pose
    """
    orig = np.array([0.27431321144104004, 5.1372178859310225e-05, 0.4564971923828125])
    axis_z = np.array([0.4785744547843933, 0.00031368513009510934, -0.8780469298362732])
    franka.end_effector.go_local(orig=orig, axis_x=[], axis_y=[], axis_z=axis_z, wait_for_target=False, wait_time=5.0)


def MakeNaturalPickInfo(domain, block_index):
    """Compute pick orientation for a given block index
    """
    T = domain.block_locations.get_T(get_block_name(block_index, domain.block_colors))

    block_orig = T[:3, 3]
    axis_y = domain.franka.end_effector.status.axis_y

    ys = [T[:3, i] for i in range(3)]
    z = np.array([0, 0, 1])
    scores = [np.abs(np.dot(y, z)) for y in ys]

    indices = [i for i, s in enumerate(scores) if s < 0.1]
    candidate_ys = [ys[i] for i in indices]
    list_thing = [(y, np.abs(np.dot(y, axis_y))) for y in candidate_ys]
    target_axis_y, _ = max(list_thing, key=lambda entry: entry[1])
    if np.dot(axis_y, target_axis_y) < 0.0:
        target_axis_y = -target_axis_y

    pick_info = PickInfo(block_orig, target_axis_y, domain.block_height, pinch_depth=domain.grasp_depth)

    return pick_info
