# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from pxr import Gf
import math
import numpy as np
from numpy.linalg import norm
import copy
import traceback


def quat_to_euler_angles(q):
    q_img = q.GetImaginary()
    q_real = q.GetReal()
    # roll (x-axis rotation)
    sinr_cosp = 2 * (q_real * q_img[0] + q_img[1] * q_img[2])
    cosr_cosp = 1 - 2 * (q_img[0] * q_img[0] + q_img[1] * q_img[1])
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (q_real * q_img[1] - q_img[2] * q_img[0])
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (q_real * q_img[2] + q_img[0] * q_img[1])
    cosy_cosp = 1 - 2 * (q_img[1] * q_img[1] + q_img[2] * q_img[2])
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def normalize(v):
    if norm(v) == 0:
        traceback.print_stack()
    v /= norm(v)
    return v


def normalized(v):
    if v is None:
        return None
    return normalize(copy.deepcopy(v))


def proj_orth(v1, v2, normalize_res=False, eps=1e-5):
    v2_norm = norm(v2)
    if v2_norm < eps:
        return v1

    v2n = v2 / v2_norm
    v1 = v1 - np.dot(v1, v2n) * v2n
    if normalize_res:
        return normalized(v1)
    else:
        return v1


def axes_to_mat(axis_x, axis_z, dominant_axis="z"):
    if dominant_axis == "z":
        axis_x = proj_orth(axis_x, axis_z)
    elif dominant_axis == "x":
        axis_z = proj_orth(axis_z, axis_x)
    elif dominant_axis is None:
        pass
    else:
        raise RuntimeError("Unrecognized dominant_axis: %s" % dominant_axis)

    axis_x = axis_x / norm(axis_x)
    axis_z = axis_z / norm(axis_z)
    axis_y = np.cross(axis_z, axis_x)

    R = np.zeros((3, 3))
    R[0:3, 0] = axis_x
    R[0:3, 1] = axis_y
    R[0:3, 2] = axis_z

    return R


# Projects T to align with the provided direction vector v.
def proj_to_align(R, v):
    max_entry = max(enumerate([np.abs(np.dot(R[0:3, i], v)) for i in range(3)]), key=lambda entry: entry[1])
    return axes_to_mat(R[0:3, (max_entry[0] + 1) % 3], v)


def as_np_matrix_t(input):
    """Get 4x4 homogeneous transform matrix for an object

    Args:
        name (string): name of object

    Returns:
        np.matrix: 4x4 homogeneous transform matrix
    """
    result = np.identity(4)
    result[:3, 3] = Gf.Vec3f(input.p.x, input.p.y, input.p.z)
    result[:3, :3] = Gf.Matrix3f(Gf.Quatf(input.r.w, Gf.Vec3f(input.r.x, input.r.y, input.r.z))).GetTranspose()
    return result
