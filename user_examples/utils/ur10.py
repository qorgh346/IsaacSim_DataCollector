# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import time
import omni
import carb
import numpy as np
from pxr import Usd, UsdGeom, Gf, UsdPhysics, PhysxSchema
from omni.isaac.motion_planning import _motion_planning
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.utils.rotations import lookat_to_quatf

from omni.isaac.surface_gripper._surface_gripper import Surface_Gripper

default_config = (-1.57, -1.57, -1.57, -1.57, 1.57, 0)


class LookAtCommander:
    def __init__(self, UR10):
        self.UR10 = UR10

    def go_pos(self, pos):
        # TODO: remove hardcoded look at origin
        self.UR10.end_effector.look_at(Gf.Vec3f(0.4, 0, 0.4), Gf.Vec3f(pos[0], pos[1], pos[2]))

    # TODO implement
    def wait_for_pos(self, error_thresh, check_only):
        return True

    # TODO implement
    def clear(self):
        pass


class Status:
    def __init__(self, mp, rmp_handle):
        self.mp = mp
        self.rmp_handle = rmp_handle
        self.orig = np.array([0, 0, 0])
        self.axis_x = np.array([1, 0, 0])
        self.axis_y = np.array([0, 1, 0])
        self.axis_z = np.array([0, 0, 1])

        self.current_frame = {"orig": self.orig, "axis_x": self.axis_x, "axis_y": self.axis_y, "axis_z": self.axis_z}
        self.target_frame = {"orig": self.orig, "axis_x": self.axis_x, "axis_y": self.axis_y, "axis_z": self.axis_z}
        self.frame = self.current_frame

    def update(self):
        state = self.mp.getRMPState(self.rmp_handle)
        target = self.mp.getRMPTarget(self.rmp_handle)
        self.orig = np.array([state[0].x, state[0].y, state[0].z])
        self.axis_x = np.array([state[1].x, state[1].y, state[1].z])
        self.axis_y = np.array([state[2].x, state[2].y, state[2].z])
        self.axis_z = np.array([state[3].x, state[3].y, state[3].z])

        self.current_frame = {"orig": self.orig, "axis_x": self.axis_x, "axis_y": self.axis_y, "axis_z": self.axis_z}
        self.frame = self.current_frame
        self.current_target = {
            "orig": np.array([target[0].x, target[0].y, target[0].z]),
            "axis_x": np.array([target[1].x, target[1].y, target[1].z]),
            "axis_y": np.array([target[2].x, target[2].y, target[2].z]),
            "axis_z": np.array([target[3].x, target[3].y, target[3].z]),
        }


class EndEffector:
    def __init__(self, dc, mp, ar, rmp_handle):
        self.dc = dc
        self.ar = ar
        self.mp = mp
        self.rmp_handle = rmp_handle
        self.gripper = None
        self.status = Status(mp, rmp_handle)
        self.UpRot = Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)

    def freeze(self):
        self.go_local(
            orig=self.status.orig, axis_x=self.status.axis_x, axis_z=self.status.axis_z, wait_for_target=False
        )

    def go_local(
        self,
        target=None,
        orig=[],
        axis_x=[],
        axis_y=[],
        axis_z=[],
        required_orig_err=0.01,
        required_axis_x_err=0.01,
        required_axis_y_err=0.01,
        required_axis_z_err=0.01,
        orig_thresh=None,
        axis_x_thresh=None,
        axis_y_thresh=None,
        axis_z_thresh=None,
        approach_direction=[],
        approach_standoff=0.1,
        approach_standoff_std_dev=0.001,
        use_level_surface_orientation=False,
        use_target_weight_override=True,
        use_default_config=False,
        wait_for_target=True,
        wait_time=None,
    ):

        self.target_weight_override_value = 10000.0
        self.target_weight_override_std_dev = 0.03
        # TODO: handle all errors?
        if orig_thresh:
            required_orig_err = orig_thresh
        if axis_x_thresh:
            required_axis_x_err = axis_x_thresh
        if axis_y_thresh:
            required_axis_y_err = axis_y_thresh
        if axis_z_thresh:
            required_axis_z_err = axis_z_thresh

        if target:
            orig = target["orig"]
            if "axis_x" in target and target["axis_x"] is not None:
                axis_x = target["axis_x"]
            if "axis_y" in target and target["axis_y"] is not None:
                axis_y = target["axis_y"]
            if "axis_z" in target and target["axis_z"] is not None:
                axis_z = target["axis_z"]

        orig = np.array(orig)
        axis_x = np.array(axis_x)
        axis_y = np.array(axis_y)
        axis_z = np.array(axis_z)
        approach = _motion_planning.Approach((0, 0, -1), 0, 0)

        if len(approach_direction) != 0:
            approach = _motion_planning.Approach(approach_direction, approach_standoff, approach_standoff_std_dev)

        pose_command = _motion_planning.PartialPoseCommand()
        if len(orig) > 0:
            pose_command.set(_motion_planning.Command(orig, approach), int(_motion_planning.FrameElement.ORIG))
        if len(axis_x) > 0:
            pose_command.set(_motion_planning.Command(axis_x), int(_motion_planning.FrameElement.AXIS_X))
        if len(axis_y) > 0:
            pose_command.set(_motion_planning.Command(axis_y), int(_motion_planning.FrameElement.AXIS_Y))
        if len(axis_z) > 0:
            pose_command.set(_motion_planning.Command(axis_z), int(_motion_planning.FrameElement.AXIS_Z))

        self.mp.goLocal(self.rmp_handle, pose_command)

        if wait_for_target and wait_time:
            error = 1
            future_time = time.time() + wait_time

            while error > required_orig_err and time.time() < future_time:
                time.sleep(0.1)
                error = self.mp.getError(self.rmp_handle)

    def look_at(self, gripper_pos, target):
        # Y up works for look at but sometimes flips, go_local might be a safer bet with a  locked y_axis
        orientation = lookat_to_quatf(gripper_pos, target, self.UpRot.TransformDir(Gf.Vec3f(0, 1, 0)))
        mat = Gf.Matrix3d(orientation).GetTranspose()

        self.go_local(
            orig=[gripper_pos[0], gripper_pos[1], gripper_pos[2]],
            axis_x=[mat.GetColumn(0)[0], mat.GetColumn(0)[1], mat.GetColumn(0)[2]],
            axis_z=[mat.GetColumn(2)[0], mat.GetColumn(2)[1], mat.GetColumn(2)[2]],
        )


# UR10 objects that contains implementation details for robot control
class UR10:
    def __init__(
        self,
        stage,
        prim,
        dc,
        mp,
        world=None,
        default_config=None,
        is_ghost=False,
        sgp=None,
        compensate_gravity=True,
        urdf="/urdf/ur10_robot_no_mat.urdf",
    ):
        self.dc = dc
        self.mp = mp
        self.prim = prim
        self.stage = stage
        # get handle to the articulation for this UR10
        self.ar = self.dc.get_articulation(prim.GetPath().pathString)
        self.is_ghost = is_ghost
        self.compensate_gravity = compensate_gravity
        self.stopped = True

        self.base = self.dc.get_articulation_root_body(self.ar)

        self._mp_extension_path = get_extension_path_from_name("omni.isaac.motion_planning")
        self._mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")

        self._rmp_data = self._mg_extension_path + "/policy_configs"

        self.rmp_handle = self.mp.registerRmp(
            self._rmp_data + urdf,
            self._rmp_data + "/ur10/rmpflow_suction/ur10_robot_description.yaml",
            self._rmp_data + "/ur10/rmpflow_suction/ur10_rmpflow_config.yaml",
            prim.GetPath().pathString,
            "ee_suction_link",
            True,
        )
        physxSceneAPI = None
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(prim)

        if physxSceneAPI is not None:
            self.mp.setFrequency(self.rmp_handle, physxSceneAPI.GetTimeStepsPerSecondAttr().Get())
        else:
            self.mp.setFrequency(self.rmp_handle, 60)

        print("UR10 rmp handle", self.rmp_handle)
        if world is not None:
            self.world = world
            self.world.rmp_handle = self.rmp_handle
            self.world.register_parent(self.base, self.prim, "base_link")

        self.end_effector = EndEffector(self.dc, self.mp, self.ar, self.rmp_handle)

        if sgp is not None:
            self.end_effector.gripper = Surface_Gripper(self.dc)
            self.end_effector.gripper.initialize(sgp)

        if default_config:
            self.mp.setDefaultConfig(self.rmp_handle, default_config)
        self.target_visibility = True
        if self.is_ghost:
            self.target_visibility = False

        self.imageable = UsdGeom.Imageable(self.prim)
        print(prim.GetPath().pathString)

    def __del__(self):
        self.mp.unregisterRmp(self.rmp_handle)
        self.end_effector.gripper = None
        print("Destructor called, UR10 deleted.")

    def set_pose(self, pos, rot):
        self._mp.setTargetLocal(self.rmp_handle, pos, rot)

    def set_speed(self, speed_level):
        pass

    def stop(self):
        self.stopped = True

    def update(self):
        if self.stopped and self.compensate_gravity:
            body_count = self.dc.get_articulation_body_count(self.ar)
            for bodyIdx in range(body_count):
                body = self.dc.get_articulation_body(self.ar, bodyIdx)
                self.dc.set_rigid_body_disable_gravity(body, True)
            self.stopped = False
        if self.end_effector.gripper is not None:
            if self.end_effector.gripper.update is not None:
                self.end_effector.gripper.update()
        self.end_effector.status.update()
        if self.imageable:
            if self.target_visibility is not self.imageable.ComputeVisibility(Usd.TimeCode.Default()):
                if self.target_visibility:
                    self.imageable.MakeVisible()
                else:
                    self.imageable.MakeInvisible()

    def send_config(self, config):
        if self.is_ghost is False:
            self.mp.setDefaultConfig(self.rmp_handle, config)
