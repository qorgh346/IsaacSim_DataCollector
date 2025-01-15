# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import numpy.linalg as la


class FrameTerminationCriteria(object):
    """
    Termination criteria for a basic frame. Implements the interface expected
    by go_reactive().
    """

    def __init__(self, orig_thresh=0.01, axis_x_thresh=0.01, axis_y_thresh=0.01, axis_z_thresh=0.01):
        self.orig_thresh = orig_thresh
        self.axis_x_thresh = axis_x_thresh
        self.axis_y_thresh = axis_y_thresh
        self.axis_z_thresh = axis_z_thresh

    def __call__(self, target_frame, current_frame):
        is_converged = True
        if "orig" in target_frame and self.orig_thresh:
            err = la.norm(target_frame["orig"] - current_frame["orig"])
            is_converged = is_converged and err <= self.orig_thresh
        if "axis_x" in target_frame and self.axis_x_thresh:
            err = la.norm(target_frame["axis_x"] - current_frame["axis_x"])
            is_converged = is_converged and err <= self.axis_x_thresh
        if "axis_y" in target_frame and self.axis_y_thresh:
            err = la.norm(target_frame["axis_y"] - current_frame["axis_y"])
            is_converged = is_converged and err <= self.axis_y_thresh
        if "axis_z" in target_frame and self.axis_z_thresh:
            err = la.norm(target_frame["axis_z"] - current_frame["axis_z"])
            is_converged = is_converged and err <= self.axis_z_thresh

        return is_converged
