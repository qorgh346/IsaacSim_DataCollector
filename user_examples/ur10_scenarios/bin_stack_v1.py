# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import random
import numpy as np

from pxr import Gf, UsdGeom, UsdPhysics
from enum import Enum
import omni
import carb
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.utils._isaac_utils import math as math_utils
from omni.isaac.examples.user_examples.utils.world import World
from omni.isaac.examples.user_examples.utils.ur10 import UR10, default_config
from omni.isaac.surface_gripper._surface_gripper import Surface_Gripper_Properties
from omni.isaac.core.utils.bounds import create_bbox_cache, compute_aabb, recompute_extents, compute_combined_aabb
from pxr import UsdGeom

from .scenario_v1 import set_scale, set_translate, set_rotate, create_ur10, Scenario, create_background, create_objects, setup_physics
from copy import copy

import omni.physx
from collections import deque

import asyncio
# import rospy
# from std_msgs.msg import String

class SM_events(Enum):
    START = 0
    WAYPOINT_REACHED = 1
    GOAL_REACHED = 2
    ATTACHED = 3
    DETACHED = 4
    TIMEOUT = 5
    STOP = 6
    BROKENGRIP = 7
    NONE = 8  # no event ocurred, just clocks


class SM_states(Enum):
    STANDBY = 0  # Default state, does nothing unless enters with event START
    PICKING = 1
    ATTACH = 2
    PLACING = 3
    DETACH = 4
    FLIPPING = 5

# class BAE_states(Enum):
#     NOACTION = 0
#     PICKUP = 1
#     CARRY = 2
#     PLACEDOWN = 3
#     INITPOSE = 4
#     PREATTACH = 5

statedic = {0: "orig", 1: "axis_x", 2: "axis_y", 3: "axis_z"}


class PickAndPlaceStateMachine(object):
    """
    Self-contained state machine class for Robot Behavior. Each machine state may react to different events,
    and the handlers are defined as in-class functions
    """


    def __init__(self, stage, robot, ee_prim, target_bodies, default_position):
        self._goal_reached = False
        self.robot = robot
        self.dc = robot.dc
        self.end_effector = ee_prim
        self.end_effector_handle = None
        self._stage = stage
        self.current = None
        self.target_bodies = target_bodies
        self.start_time = 0.0
        self.start = False
        self._time = 0.0
        self.default_timeout = 0.5
        self.default_position = copy(default_position)
        self.target_position = default_position
        self.reset = False
        self.waypoints = deque()
        self.thresh = {}
        self.base_pose = [641, 300, 118.180]#[637, 300, 118.180]# [660, 300, 118.180]
        #  [13200.0, 14190, 118.180]
        self.step_start = None
        self.start_flag = False

        self.current_box_name = None
        self.current_box_vh_rel = dict()
        self.picked_box = 0
        self.current_pallet_name = None
        self.able_get_box = False
        # Threshold to clear waypoints/goal
        # (any waypoint that is not final will be cleared with the least precision)
        self.precision_thresh = [
            [0.0005, 0.0025, 0.0025, 0.0025],
            [0.0005, 0.005, 0.005, 0.005],
            [0.05, 0.2, 0.2, 0.2],
            [0.08, 0.4, 0.4, 0.4],
            [0.18, 0.6, 0.6, 0.6],
        ]
        self.add_bin = None
        self.packingFinished = False
        self.pick_count = 0

        # Event management variables
        # Used to verify if the goal was reached due to robot moving or it had never left previous target
        self._is_moving = False
        self._attached = False  # Used to flag the Attached/Detached events on a change of state from the end effector
        self._detached = False
        self._upright = False  # Used to indicate if the bin is being picked facing up, so the proper state is called
        self._flipped = False
        self._closed = False

        # Constants for lifting, fliping, and intermediary goals
        self.upside_goal = _dynamic_control.Transform()
        self.upside_offset = _dynamic_control.Transform()
        self.upside_flip = _dynamic_control.Transform()

        self.upside_goal.p = (0.80808, 0.70978, -0.03)
        self.upside_goal.r = (0.12380744620354063, -0.510572739737588, -0.1093972128961598, 0.8438124456962951)

        self.upside_offset.p = [0.51855, 0.9221, 0.1370]
        self.upside_offset.r = self.upside_goal.r

        self.upside_flip.p = [0.51855, 0.9221, 0.1370]
        self.upside_flip.r = self.upside_goal.r
        # flip 180 degrees along y axis

        # Define the state machine handling functions
        self.sm = {}
        # Make empty state machine for all events and states
        for s in SM_states:
            self.sm[s] = {}
            for e in SM_events:
                self.sm[s][e] = self._empty
                self.thresh[s] = 0

        # Use a same event handler for broken grip on all events
        for s in SM_states:
            self.sm[s][SM_events.BROKENGRIP] = self._all_broken_grip

        # Fill in the functions to handle each event for each status
        self.sm[SM_states.STANDBY][SM_events.START] = self._standby_start
        self.sm[SM_states.STANDBY][SM_events.GOAL_REACHED] = self._standby_goal_reached
        self.thresh[SM_states.STANDBY] = 3

        self.sm[SM_states.PICKING][SM_events.GOAL_REACHED] = self._picking_goal_reached
        self.sm[SM_states.PICKING][SM_events.NONE] = self._picking_no_event
        self.thresh[SM_states.PICKING] = 1

        self.sm[SM_states.FLIPPING][SM_events.GOAL_REACHED] = self._flipping_goal_reached
        self.thresh[SM_states.FLIPPING] = 2

        self.sm[SM_states.PLACING][SM_events.GOAL_REACHED] = self._placing_goal_reached
        self.thresh[SM_states.PLACING] = 0

        self.sm[SM_states.ATTACH][SM_events.GOAL_REACHED] = self._attach_goal_reached
        self.sm[SM_states.ATTACH][SM_events.ATTACHED] = self._attach_attached
        self.thresh[SM_states.ATTACH] = 0

        self.sm[SM_states.DETACH][SM_events.GOAL_REACHED] = self._detach_goal_reached
        self.sm[SM_states.DETACH][SM_events.DETACHED] = self._detach_detached
        self.thresh[SM_states.DETACH] = 0

        self.current_state = SM_states.STANDBY
        self.previous_state = -1
        self._physx_query_interface = omni.physx.get_physx_scene_query_interface()

        x = [114, 77] # [100, 79, 58]
        y = [-14, 13] # [-62, -31, 0]
        self.stack_coordinates = np.array(
            [
                [x[0], y[0]],
                [x[1], y[0]],
                [x[0], y[1]],
                [x[1], y[1]],
            ]
        )
        self.stack_size = np.zeros([9, 1])
        self.stack_transition = ((3, 6, 8), (1, 3, 6))
        self.current_stack_list = [0, 0]
        self.current_stack = 0

        self.total_bins = 0
        self.packed_bins = 0

        # pallet placement
        # self.pallet_placement = {
        #     "place_01": [1.14, -0.14, -0.70],
        #     "place_02": [0.79, -0.14, -0.70],
        #     "place_03": [1.14, 0.13, -0.70],
        #     "place_04": [0.79, 0.13, -0.70]
        # }

        self.pallet_placement = {
            "place_01": [0.80, -0.10, -0.70],
            "place_02": [1.14, -0.10, -0.70],
            "place_03": [0.80, 0.14, -0.70],
            "place_04": [1.14, 0.14, -0.70]
        }


    # Auxiliary functions

    def get_current_place_pose(self):
        """
        Gets the (x,y) coordinates for the current stack placement
        """
        while self.stack_size[self.current_stack_list[self.current_stack]] > 3:
            self.advance_stack()
        return self.stack_coordinates[self.current_stack_list[self.current_stack]]

    def advance_stack(self):
        """
        Moves to next stack, prioritizing filling the left-bottom-most stack, but maintaining
        a single bin ahead of the other stacks
        """
        self.current_stack_list[self.current_stack] = (self.current_stack_list[self.current_stack] + 1) % 9
        if self.current_stack_list[self.current_stack] in self.stack_transition[self.current_stack]:
            self.current_stack = (self.current_stack + 1) % 2

    def _empty(self, *args):
        """
        Empty function to use on states that do not react to some specific event
        """
        pass

    def change_state(self, new_state):
        """
        Function called every time a event handling changes current state
        """
        self.current_state = new_state
        self.start_time = self._time
        carb.log_warn(str(new_state))

    def goalReached(self):
        """
        Checks if the robot has reached a certain waypoint in the trajectory
        """
        if self._is_moving:
            state = self.robot.end_effector.status.current_frame
            target = self.robot.end_effector.status.current_target
            error = 0
            for i in [0, 2, 3]:
                k = statedic[i]
                state_v = state[k]
                target_v = target[k]
                error = np.linalg.norm(state_v - target_v)
                # General Threshold is the least strict
                thresh = self.precision_thresh[-1][i]
                # if the target is a goal point, use the defined threshold for the current state
                if len(self.waypoints) == 0:
                    thresh = self.precision_thresh[self.thresh[self.current_state]][i]
                if error > thresh:
                    return False
            self._is_moving = False
            return True
        return False

    def get_current_state_tr(self):
        """
        Gets current End Effector Transform, converted from Motion position and Rotation matrix
        """
        # Gets end effector frame
        state = self.robot.end_effector.status.current_frame

        orig = state["orig"] * 100.0

        mat = Gf.Matrix3f(
            *state["axis_x"].astype(float), *state["axis_y"].astype(float), *state["axis_z"].astype(float)
        )
        q = mat.ExtractRotation().GetQuaternion()
        (q_x, q_y, q_z) = q.GetImaginary()
        q = [q_x, q_y, q_z, q.GetReal()]
        tr = _dynamic_control.Transform()
        tr.p = list(orig)
        tr.r = q
        return tr

    def ray_cast(self, x_offset=0.15, y_offset=3.0, z_offset=0.0):
        # print("=============== ray_cast()")
        """
        Projects a raycast forward from the end effector, with an offset in end effector space defined by (x_offset, y_offset, z_offset)
        if a hit is found on a distance of 100 centimiters, returns the object usd path and its distance
        """
        tr = self.get_current_state_tr()
        # print('     a target', tr.p)

        # print('     b target', tr.p)

        offset = _dynamic_control.Transform()
        offset.p = (x_offset, y_offset, z_offset)
        raycast_tf = math_utils.mul(tr, offset)
        origin = raycast_tf.p
        # print('     raycast', raycast_tf.p)
        origin[0] = -origin[0]
        origin[1] = -origin[1]
        origin = [x + y for x,y in zip(origin, self.base_pose)]
        # print('    origin', origin)

        rotate = raycast_tf.r
        rotate2 = [0, 1, 0, 0]
        r3 = [0] * 4
        r3[0] = rotate[0] * rotate2[0] - rotate[1] * rotate2[1] - rotate[2] * rotate2[2] - rotate[3] * rotate2[3]
        r3[1] = rotate[0] * rotate2[1] + rotate[1] * rotate2[0] + rotate[2] * rotate2[3] - rotate[3] * rotate2[2]
        r3[2] = rotate[0] * rotate2[2] - rotate[1] * rotate2[3] + rotate[2] * rotate2[0] + rotate[3] * rotate2[1]
        r3[3] = rotate[0] * rotate2[3] + rotate[1] * rotate2[2] - rotate[2] * rotate2[1] + rotate[3] * rotate2[0]
        raycast_tf.r = r3

        rayDir = math_utils.get_basis_vector_x(raycast_tf.r)
        hit = self._physx_query_interface.raycast_closest(origin, rayDir, 100.0)
        # print('hit2', hit)
        if hit["hit"]:
            usdGeom = UsdGeom.Mesh.Get(self._stage, hit["rigidBody"])
            distance = hit["distance"]
            # print("     getpath:{}, distance:{}".format(usdGeom.GetPath().pathString, distance))
            return usdGeom.GetPath().pathString, distance
        return None, 10000.0

    def get_target(self): # VERSION_2
        """
        Indicates if there is any bin at the bottom of the conveyor belt, if there is, the current target object is set
        with the found value
        """
        origin = (36.0, -44.0, -50.0)
        origin = [x + y for x, y in zip(origin, self.base_pose)]  # [617.0, 158.0, 80.18]
        rayDir = (-1, 0, 0)
        hit = self._physx_query_interface.raycast_closest(origin, rayDir, 100.0)
        # print('hit1', hit)
        if hit["hit"]:
            self.current = hit["rigidBody"]
            # print("    self.current:{}, hit[hit]:{}, hit[rigidBody]:{}".format(self.current, hit["hit"], hit["rigidBody"]))
            return True
        self.current = None
        # print("     self.current:", self.current)
        return False


    def get_target_pallet(self):
        """
        Indicates if there is any bin at the bottom of the conveyor belt, if there is, the current target object is set
        with the found value
        """
        # origin = (-104.2, 7.0, -80.9)
        origin = (-75.0, 0, -92.9) # [13059.0, 14269.0, 25.28]
        origin = [x + y for x, y in zip(origin, self.base_pose)]
        rayDir = (1, 0, 0)
        hit1 = self._physx_query_interface.raycast_closest(origin, rayDir, 50.0)
        # print('hit1: ', hit1)
        rayDir = (-1, 0, 0)
        hit2 = self._physx_query_interface.raycast_closest(origin, rayDir, 50.0)
        # print('hit2: ', hit2)

        if hit1["hit"]:
            # print('-----hit for pallet ----', hit)
            usdGeom = UsdGeom.Mesh.Get(self._stage, hit1["rigidBody"])
            self.current_pallet_name = usdGeom.GetPath().pathString.split('/')[-1]
            if 'Pallet' in self.current_pallet_name:
                return True
        elif hit2["hit"]:
            usdGeom = UsdGeom.Mesh.Get(self._stage, hit2["rigidBody"])
            self.current_pallet_name = usdGeom.GetPath().pathString.split('/')[-1]
            if 'Pallet' in self.current_pallet_name:
                return True
        return False

    def lerp_to_pose(self, pose, n_waypoints=1):
        """
        adds spherical linear interpolated waypoints from last pose in the waypoint list to the provided pose
        if the waypoit list is empty, use current pose
        """

        if len(self.waypoints) == 0:
            start = self.get_current_state_tr()
            start.p = math_utils.mul(start.p, 0.01)
        else:
            start = self.waypoints[-1]

        if n_waypoints > 1:
            for i in range(n_waypoints):
                self.waypoints.append(math_utils.slerp(start, pose, (i + 1.0) / n_waypoints))
        else:
            self.waypoints.append(pose)

    def move_to_zero(self):
        """
        clears the robot target, so it returns to its rest pose
        """
        self._is_moving = False
        self.robot.end_effector.go_local(
            orig=[], axis_x=[], axis_y=[], axis_z=[], use_default_config=True, wait_for_target=False, wait_time=5.0
        )

    def move_to_target(self):
        """
        moves the end effector to the current target pose
        """
        xform_attr = self.target_position
        self._is_moving = True

        orig = np.array([xform_attr.p.x, xform_attr.p.y, xform_attr.p.z])
        axis_y = np.array(math_utils.get_basis_vector_y(xform_attr.r))
        axis_z = np.array(math_utils.get_basis_vector_z(xform_attr.r))
        self.robot.end_effector.go_local(
            orig=orig,
            axis_x=[],
            axis_y=axis_y,
            axis_z=axis_z,
            use_default_config=True,
            wait_for_target=False,
            wait_time=5.0,
        )


    def get_target_to_object(self, offset_up=25, offset_down=25):
        # print("================ get_target_to_object() ===================")
        """
        Gets target pose to end effector on a given target, with an offset on the end effector actuator direction given
        by [offset_up, offset_down]
        """
        offset = _dynamic_control.Transform()
        offset.p.z = offset_up
        offset.r = (0, 0.7071, 0, 0.7071)
        body_handle = self.dc.get_rigid_body(self.current)
        obj_pose = self.dc.get_rigid_body_pose(body_handle)
        obj_pose.p = [y-x for x, y in zip(self.base_pose, obj_pose.p)]
        obj_pose.p.x = - obj_pose.p.x
        obj_pose.p.y = - obj_pose.p.y
        # obj_pose.p = [x + y for x, y in zip(self.base_pose, obj_pose.p)]
        # obj_pose.p = math_utils.mul(obj_pose.p, 0.36)

        rotate = obj_pose.r
        rotate2 = [0, 1, 0, 0]
        r3 = [0] * 4
        r3[0] = rotate[0] * rotate2[0] - rotate[1] * rotate2[1] - rotate[2] * rotate2[2] - rotate[3] * rotate2[3]
        r3[1] = rotate[0] * rotate2[1] + rotate[1] * rotate2[0] + rotate[2] * rotate2[3] - rotate[3] * rotate2[2]
        r3[2] = rotate[0] * rotate2[2] - rotate[1] * rotate2[3] + rotate[2] * rotate2[0] + rotate[3] * rotate2[1]
        r3[3] = rotate[0] * rotate2[3] + rotate[1] * rotate2[2] - rotate[2] * rotate2[1] + rotate[3] * rotate2[0]
        obj_pose.r = r3

        offset_1 = _dynamic_control.Transform()
        tr = self.get_current_state_tr()
        rx = math_utils.dot(math_utils.get_basis_vector_y(obj_pose.r), math_utils.get_basis_vector_y(tr.r))
        if math_utils.get_basis_vector_z(obj_pose.r).z > 0:
            # self._upright = True
            if rx < 0:  # rotate target by 180 degrees on z axis
                offset_1.r = (0, 0, 1, 0)
        else:  # If bin is upside down, pick by bottom
            offset.p.z = offset_down
            if rx < 0:  # rotate target by 180 degrees on z axis
                offset_1.r = (1, 0, 0, 0)
            else:
                offset_1.r = (0, -1, 0, 0)
            # self._upright = False

        target_position = math_utils.mul(math_utils.mul(obj_pose, offset_1), offset)
        target_position.p = math_utils.mul(target_position.p, 0.01)
        # print("      B target_position:", target_position)
        # print("      A target_position:", target_position)
        # print("============================ ")
        return target_position

    def set_target_to_object(self, offset_up=25, offset_down=25, n_waypoints=1, clear_waypoints=True):
        """
        Clears waypoints list, and sets a new waypoint list towards the target pose for an object.
        """
        target_position = self.get_target_to_object(offset_up, offset_down)
        # linear interpolate to target pose
        if clear_waypoints:
            self.waypoints.clear()
        self.lerp_to_pose(target_position, n_waypoints=n_waypoints)
        # Get first waypoint target
        self.target_position = self.waypoints.popleft()

    def step(self, timestamp, start=False, reset=False):
        """
            Steps the State machine, handling which event to call
        """
        if self.current_state != self.previous_state:
            self.previous_state = self.current_state
        if not self.start:
            self.start = start
            # self.step_start = True
        self.reset = reset

        # Process events
        if reset:
            self.packingFinished = False
            self.current_state = SM_states.STANDBY
            self.robot.end_effector.gripper.open()
            self._closed = False
            self.start = False
            self._upright = False
            self.waypoints.clear()
            self.target_position = self.default_position
            self.move_to_target()
            self.current_stack_list = [-1, 0]
            self.current_stack = 0
            self.total_bins = 0
            self.packed_bins = 0
            self.stack_size *= 0
            self.reset = False
            self.current = None
            self.step_start = None
        elif self.goalReached():
            if len(self.waypoints) == 0:
                # print("check goalReached")
                if self.packed_bins == 4 and SM_states.STANDBY:
                    self.packingFinished = True
                    self.packed_bins = 0
                self.sm[self.current_state][SM_events.GOAL_REACHED]()
            else:
                self.target_position = self.waypoints.popleft()
                self.move_to_target()
                self.start_time = self._time
        elif start and self.step_start is None:
            if self.get_target_pallet():
                # self.sm[self.current_state][SM_events.NONE]()
                self.step_start = True
            else:
                self.start = False
                self.step_start = False
        elif self.current_state == SM_states.STANDBY and self.start and self.get_target():
            self.sm[self.current_state][SM_events.START]()
        elif self._attached:
            self._attached = False
            self.sm[self.current_state][SM_events.ATTACHED]()
        elif self._detached:
            self._detached = False
            self.sm[self.current_state][SM_events.DETACHED]()
        elif self._time - self.start_time > self.default_timeout:
            self.sm[self.current_state][SM_events.TIMEOUT]()
            # self.sm[self.current_state][SM_events.START]()
        else:
            self.sm[self.current_state][SM_events.NONE]()


    def _standby_start(self, *args):
        """
        Handles the start event when in standby mode.
        Proceeds to pick up the next bin on the queue, and set the arm
        to move towards the bin from current  position.
        switches to picking state.
        """
        # print("===============STANDBY START===============")
        if self.total_bins < 80:
            # Tell motion planner controller to ignore current object as an obstacle
            self.target_bodies[self.current].suppress()
            self.pick_count = 0
            self.lerp_to_pose(self.default_position, 1)
            self.lerp_to_pose(self.default_position, 60)
            # set target above the current bin with offset of 20 cm
            self.set_target_to_object(12, 20, clear_waypoints=False)
            # start arm movement
            self.move_to_target()
            # Move to next state
            self.change_state(SM_states.PICKING)

    def _standby_goal_reached(self, *args):
        """
        Finished processing a bin, moves up the stack position for next bin placement
        """
        # print("===============STANDBY GOAL REACHED===============")
        self.move_to_zero()
        # self.advance_stack()
        self.start = True
        # print("=============================================")

    def _flipping_goal_reached(self, *args):
        # print("===============FLIPPING GOAL REAHCED===============")
        """
        Reached the goal base pose for flipping.
        sets the robot to flip the arm to the bin upside pose, and towards the placement goal in the platform.
        Sets the next state as Detach.
        """

        self.lerp_to_pose(self.upside_flip, n_waypoints=1)
        target_position = self.upside_offset
        self.lerp_to_pose(target_position, 1)
        self.lerp_to_pose(self.upside_goal, 80)
        self.lerp_to_pose(self.upside_goal, 30)
        self.target_position = self.waypoints.popleft()
        self.move_to_target()
        self.change_state(SM_states.DETACH)
        self._flipped = True
        if self.add_bin is not None:
            self.add_bin()
        # print("=============================================")

    def _attach_goal_reached(self, *args):
        # print("===============ATACH GOAL REACHED===============")
        """
        Handles a state machine step when the target goal is reached, and the machine is on attach state
        """
        self.robot.end_effector.gripper.close()
        self._closed = True
        self.lerp_to_pose(self.target_position, 60)  # Wait 1 second in place for attachment
        if self.robot.end_effector.gripper.is_closed():
            self._attached = True
        else:  # Failed to attach so return grasp to try again
            # move up 25 centimiters and return to picking state
            offset = _dynamic_control.Transform()
            offset.p = (-0.25, 0.0, 0.0)
            self.target_position = math_utils.mul(self.target_position, offset)
            self.move_to_target()
            self.change_state(SM_states.PICKING)
        # print("=============================================")

    def _attach_attached(self, *args):
        # print("===============ATTACH ATTACHED===============")
        """
        Handles a state machine step when the target goal is reached, and the machine is on attach state
        """
        offset = _dynamic_control.Transform()
        offset.p = (-0.20, 0.0, 0.0)
        target = math_utils.mul(self.target_position, offset)
        self.lerp_to_pose(target, 10)
        # if not self._upright:
        target.p.z = 0.15
        target.r = [0, 0.7071, 0, 0.7071]
        self.lerp_to_pose(target, 10)
        self.target_position = self.waypoints.popleft()
        self.move_to_target()
        self.pick_count += 1
        target = _dynamic_control.Transform()
        if self.total_bins % 4 == 0:
            x,y,_= self.pallet_placement['place_01']
        elif self.total_bins % 4 == 1:
            x,y,_ = self.pallet_placement['place_02']
        elif self.total_bins % 4 == 2:
            x,y,_ = self.pallet_placement['place_03']
        else:
            x,y,_ = self.pallet_placement['place_04']

        target.r = [0, 0.35836791862248063, 0, 0.9335804383673595]
        target.p.x = x
        target.p.y = y
        target.p.z = 0.15
        # If bin was picked from the bottom directly from the conveyor belt, send it to an intermediary waypoint
        # before sending it to its target so it clears the robot base
        if self.pick_count == 1:
            target.p.x = 0.5
            target.p.y = 0.5
            self.lerp_to_pose(target, 25)
            target.p.y = 0
            self.lerp_to_pose(target, 25)
            target.p.x = x
            target.p.y = y
            if self.add_bin is not None:
                self.add_bin()

            self.lerp_to_pose(target, 50)
            self.lerp_to_pose(target, 60)
            target.r = [0, 0.7071, 0, 0.7071]
            self.lerp_to_pose(target, 1)
            self.lerp_to_pose(target, 30)
            self.change_state(SM_states.PLACING)
        # print("=============================================")

    def _detach_goal_reached(self, *args):
        # print("===============DETACH GOAL REAHCED===============")
        """
        Handles the goal reached event while in detach state.
        If the bin is still fixed to the robot, opens the gripper and waits in place for it to open.
        Then sets the next state which may be picking the bin again from the bottom, if it was upright, otherwise
        sets it to the standby pose and state
        """
        if self.robot.end_effector.gripper.is_closed():
            self.robot.end_effector.gripper.open()
            self._closed = False
            # Lerp to its same pose to wait a few timesteps before entering this event again.
            self.lerp_to_pose(self.target_position, n_waypoints=4)
            self._detached = True
            self.thresh[SM_states.DETACH] = 3
        else:
            # Sends the arm to an intermediary pose that will help clear the robot base more easily
            target = copy(self.default_position)
            # target.p = [0.8, 0.4, 0.4]
            # target.r = [-0.271, 0.650, 0.271, 0.651]
            self.lerp_to_pose(target, n_waypoints=1)
            self.target_position = self.waypoints.popleft()
            self.move_to_target()
            self.lerp_to_pose(self.default_position, n_waypoints=1)
            self.lerp_to_pose(self.default_position, n_waypoints=60)

            self.start = False
            self.total_bins += 1
            self.packed_bins += 1
            self.change_state(SM_states.STANDBY)
            self.thresh[SM_states.DETACH] = 0


    def _detach_detached(self, *args):
        # print("===============DETACH DETACHED===============")
        """
        Event that happens right after the arm succesfully detaches from the object it was holding.
        """
        offset = _dynamic_control.Transform()

        offset.p = (-0.10, 0.0, 0.0)
        # Move the arm up slowly 10 cm
        self.lerp_to_pose(math_utils.mul(self.target_position, offset), n_waypoints=30)
        offset.p = (-0.30, 0.0, 0.0)
        # Move the arm further up 20 extra cm, but faster
        self.lerp_to_pose(math_utils.mul(self.target_position, offset), n_waypoints=1)

        self.target_position = self.waypoints.popleft()
        self.move_to_target()
        # print("=============================================")

    def _picking_goal_reached(self, *args):
        # print("===============PICKING GOAL REACHED===============")
        """
        Handles a state machine step when goal was reached event happens, while on picking state
        ensures the bin obstacle is suppressed for the planner, Updates the target position
        to where the bin surface is, and send the robot to move towards it. No change of state happens
        """
        if self._flipped:
            self._flipped = False
            return
        obj, distance = self.ray_cast()
        if obj is not None:
            # print(obj, distance)
            # Set target towards surface of the bin
            tr = self.get_current_state_tr()
            offset = _dynamic_control.Transform()
            offset.p = (distance +0.15, 0, 0)

            target = math_utils.mul(tr, offset)
            target.p = math_utils.mul(target.p, 0.01)
            offset.p.x = -0.05
            pre_target = math_utils.mul(target, offset)
            self.lerp_to_pose(pre_target, n_waypoints=90)
            self.lerp_to_pose(target, n_waypoints=60)
            self.lerp_to_pose(target, n_waypoints=30)
            self.target_position = self.waypoints.popleft()
            self.move_to_target()

            self.change_state(SM_states.ATTACH)
        # print("=============================================")

    def _picking_no_event(self, *args):
        # print("===============PICKING NO EVNET===============")
        """
        Handles a state machine step when no event happened, while on picking state
        ensures the bin obstacle is suppressed for the planner, Updates the target position
        to where the bin is, and send the robot to move towards it. No change of state happens
        """
        # if not self._flipped:
        # set target above the current bin with offset of 25 cm
        # self.set_target_to_object(20, 16, 3)
        # self.thresh[SM_states.PICKING] = 0
        # # start arm movement
        # self.move_to_target()
        # print("=============================================")

    def _placing_goal_reached(self, *args):
        # print("===============PLACING GOAL REACHED===============")
        """
        robot reached the placing pose. If it's placing on the final destination,.identifies how farther down it needs to go,
        and places the bin either on top of another bin, or on a predefined grid pose.
        """

        target = copy(self.target_position)

        if self.total_bins % 4 == 0:
            target.p = self.pallet_placement['place_01']
        elif self.total_bins % 4 == 1:
            target.p = self.pallet_placement['place_02']
        elif self.total_bins % 4 == 2:
            target.p = self.pallet_placement['place_03']
        else:
            target.p = self.pallet_placement['place_04']
        target.p.z += 0.05
        self.lerp_to_pose(target, 30)
        target.p.z -= 0.05
        self.lerp_to_pose(target, 20)  # Moves down the last 5cm at ~1cm/s
        self.lerp_to_pose(target, 50)  # Waits in place for arm to stabilize for about one second
        self.target_position = self.waypoints.popleft()
        self.move_to_target()
        self.change_state(SM_states.DETACH)

    def _all_broken_grip(self, *args):
        self._closed = False
        tr = self.get_current_state_tr()
        self.waypoints.clear()
        self.lerp_to_pose(tr, 60)
        self.lerp_to_pose(self.default_position, 90)
        self.target_position = self.waypoints.popleft()
        self.move_to_target()
        self.current_state = SM_states.STANDBY

    # def close_gripper(self):
    #     self.CLOSE = False
    #     if not self.robot.end_effector.gripper.is_closed():
    #         self.robot.end_effector.gripper.close()
    #         self._closed = True
    #     if self.robot.end_effector.gripper.is_closed():
    #         self._attached = True
    #     self.change_state(SM_states.PICKING)
    #     self.current_state_bae = BAE_states.PICKUP

    # def open_gripper(self):
    #     self.OPEN = False
    #     self.pick_count = 0
    #     # self.current_box_name = None
    #     # self.current_pallet_name = None
    #     if self.robot.end_effector.gripper.is_closed():
    #         self.robot.end_effector.gripper.open()
    #         self._closed = False
    #         self._detached = True
    #         self.total_bins += 1
    #         self.packed_cargos.append(self.current_box_name)
    #     self.change_state(SM_states.DETACH)
    #     self.current_state_bae = BAE_states.NOACTION

    # def change_action_command(self, action):
    #     self._goal_reached = False
    #
    #     if action == "MoveArmToInitPose":
    #         self.INIT_POSE = True
    #     elif action == "detect_to_item":
    #         self.DETECT_TO_ITEM = True
    #     elif action == "preattach_to_item":
    #         self.PREATTACH = True
    #     elif action == "attach_to_item":
    #         self.ATTACH = True
    #     elif action == "CloseGripper":
    #         self.CLOSE = True
    #     elif action == "postattach_to_item":
    #         self.POSTATTACH = True
    #     elif action == "detect_to_pallet":
    #         self.DETECT_TO_PALLET = True
    #     elif action == "predetach_from_item":
    #         self.PREDETACH = True
    #     elif action == "detach_from_item":
    #         self.DETACH = True
    #     elif action == "OpenGripper":
    #         self.OPEN = True
    #     elif action == "postdetach_from_item":
    #         self.POSTDETACH = True


    # def init_position(self):
    #     self.INIT_POSE = False
    #     position = _dynamic_control.Transform()
    #     position.p = [-0.0645, 0.7, 0.6]
    #     position.r = [-0.33417784954541885, 0.33389792551856345, 0.6230546169232118, 0.6234102056738156]
    #     self.lerp_to_pose(position, 30)
    #     self.target_position = self.waypoints.popleft()
    #     self.move_to_target()
    #     self.pick_count = 0
    #     self.change_state(SM_states.STANDBY)
    #     self.current_state_bae = BAE_states.INITPOSE


    # def detect_to_item(self):
    #     self.DETECT_TO_ITEM = False
    #     detect_position = _dynamic_control.Transform()
    #     detect_position.p = [-0.0645, 0.7214, 0.495]
    #     detect_position.r = [-0.33417784954541885, 0.33389792551856345, 0.6230546169232118, 0.6234102056738156]
    #     self.lerp_to_pose(self.default_position, 20)
    #     self.target_position = self.waypoints.popleft()
    #     self.move_to_target()
    #     self.current_state_bae = BAE_states.PREATTACH

    # def preattach_to_item(self):
    #     self.PREATTACH = False
    #     self.able_get_box = self.get_target()
    #     if self.total_bins < 50 and self.get_target():
    #         self.target_bodies[self.current].suppress()
    #         self.pick_count = 0
    #         self.set_target_to_object(12, 20, n_waypoints=10, clear_waypoints=False)
    #         self.move_to_target()
    #         self.change_state(SM_states.PICKING)
    #         self.current_state_bae = BAE_states.PREATTACH



    # def attach_to_item(self):
    #     self.ATTACH = False
    #     obj, distance = self.ray_cast()
    #     if obj is not None:
    #         # Set target towards surface of the bin
    #         tr = self.get_current_state_tr()
    #         offset = _dynamic_control.Transform()
    #         offset.p = (distance + 0.15, 0, 0)
    #
    #         target = math_utils.mul(tr, offset)
    #         target.p = math_utils.mul(target.p, 0.01)
    #         offset.p.x = -0.05
    #         pre_target = math_utils.mul(target, offset)
    #         self.lerp_to_pose(pre_target, n_waypoints=90)
    #         self.lerp_to_pose(target, n_waypoints=90)
    #         self.target_position = self.waypoints.popleft()
    #         self.move_to_target()
    #         self.change_state(SM_states.PICKING)
    #         # self.current_state_bae = BAE_states.PICKUP

    # def postattach_to_item(self):
    #     self.POSTATTACH = False
    #     result = self.get_target_pallet()
    #
    #     target = _dynamic_control.Transform()
    #     target.p = [-0.0789143, 0.500831, 0.15]
    #     target.r = [0, 0.7071, 0, 0.7071]
    #     self.lerp_to_pose(target, 30)
    #     self.target_position = self.waypoints.popleft()
    #     self.move_to_target()
    #     self.pick_count += 1
    #     self.picked_box += 1
    #     self.change_state(SM_states.PLACING)
    #     self.current_state_bae = BAE_states.PICKUP

    # def detect_to_pallet(self):
    #     self.DETECT_TO_PALLET = False
    #     target = _dynamic_control.Transform()
    #     target.r = [0, 0.35836791862248063, 0, 0.9335804383673595]
    #     if self.pick_count == 1:
    #         target.p.x = 0.5
    #         target.p.y = 0.5
    #         self.lerp_to_pose(target, 25)
    #         target.p.y = 0
    #         self.lerp_to_pose(target, 25)
    #     if self.total_bins % 4 == 0:
    #         x, y, _ = self.pallet_placement['place_01']
    #     elif self.total_bins % 4 == 1:
    #         x, y, _ = self.pallet_placement['place_02']
    #     elif self.total_bins % 4 == 2:
    #         x, y, _ = self.pallet_placement['place_03']
    #     else:
    #         x, y, _ = self.pallet_placement['place_04']
    #     target.r = [0, 0.35836791862248063, 0, 0.9335804383673595]
    #     target.p.x = x
    #     target.p.y = y
    #     target.p.z = 0.15
    #     self.lerp_to_pose(target, 110)
    #
    #     # self.lerp_to_pose(target, 60)
    #     self.target_position = self.waypoints.popleft()
    #     self.move_to_target()
    #     self.change_state(SM_states.PLACING)
    #     self.current_state_bae = BAE_states.CARRY


    # def predetach_from_item(self):
    #     self.PREDETACH = False
    #     target = self.target_position
    #     target.r = [0, 0.7071, 0, 0.7071]
    #     self.lerp_to_pose(target, 1)
    #     self.lerp_to_pose(target, 30)
    #     self.target_position = self.waypoints.popleft()
    #     self.move_to_target()
    #     self.change_state(SM_states.PLACING)
    #     self.current_state_bae = BAE_states.PLACEDOWN
    #     ##### bae



    # def detach_from_item(self):
    #     self.DETACH = False
    #     target = copy(self.target_position)
    #
    #     if self.total_bins % 4 == 0:
    #         target.p = self.pallet_placement['place_01']
    #     elif self.total_bins % 4 == 1:
    #         target.p = self.pallet_placement['place_02']
    #     elif self.total_bins % 4 == 2:
    #         target.p = self.pallet_placement['place_03']
    #     else:
    #         target.p = self.pallet_placement['place_04']
    #     self.lerp_to_pose(target, 120)
    #     self.lerp_to_pose(target, 75)
    #     self.target_position = self.waypoints.popleft()
    #     self.move_to_target()
    #     self.change_state(SM_states.PLACING)
    #     self.current_state_bae = BAE_states.PLACEDOWN
    # def postdetach_from_item(self):
    #     self.POSTDETACH = False
    #     offset = _dynamic_control.Transform()
    #     offset.p = (-0.10, 0.0, 0.0)
    #     # Move the arm up slowly 10 cm
    #     self.lerp_to_pose(math_utils.mul(self.target_position, offset), n_waypoints=30)
    #     offset.p = (-0.30, 0.0, 0.0)
    #     # Move the arm further up 20 extra cm, but faster
    #     self.lerp_to_pose(math_utils.mul(self.target_position, offset), n_waypoints=1)
    #     self.target_position = self.waypoints.popleft()
    #     self.move_to_target()
    #     self.total_bins += 1
    #     self.stack_size[self.current_stack_list[self.current_stack]] += 1
    #     self.change_state(SM_states.STANDBY)
    #     self.current_state_bae = BAE_states.INITPOSE



class BinStack(Scenario):
    """
    Defines an obstacle avoidance scenario
    Scenarios define the life cycle within kit and handle init, startup, shutdown etc.
    """

    def __init__(self, dc, mp, start_bin):
        super().__init__(dc, mp)
        self._paused = True
        self._start = False
        self._reset = False
        self._time = 0
        self.pick_and_place = None
        self._pending_disable = False
        self.start_bin = start_bin  # 8
        self.max_bins = 80
        self.current_bin = 0
        self.unpicked_bins = 0
        self._bins = {}
        self.add_bin_timeout = -1
        self.bin_added_timeout = 0
        self.base_pose = [641, 300, 118.180]#[637, 300, 118.180]# [660, 300, 118.180]
        self._waypoints_backup = None
        self.stopped = True
        self._pending_stop = False
        self.pick_and_place_step = None
        self.packingFinished = False

    def on_startup(self):
        super().on_startup()

    def step(self, step):
        if self._timeline.is_playing():
            if self._pending_stop:
                self.stop_tasks()
                return
            # Disable requires a one simulation step after they have been moved
            # from their previous location to work.
            if self._pending_disable:
                self.disable_bins()

            # Updates current references and locations for the robot.
            self.world.update()
            self.ur10_solid.update()

            target = self._stage.GetPrimAtPath(self.env_path + "/target")
            xform_attr = target.GetAttribute("xformOp:transform")

            if self._reset:
                self._paused = False
                self.reset_step_start()
                self.reset_packingFinished()
            if not self._paused:
                self._time += step
                if self.add_bin_timeout > 0:
                    self.add_bin_timeout -= 1
                    if self.add_bin_timeout == 0:
                        self.create_new_bin()
                self.pick_and_place.step(self._time, self._start, self._reset)
                if self._start and self.current_bin == 0 and self.pick_and_place.step_start:
                    self.create_new_bin()
                    self.reset_packingFinished()
                if self._reset:
                    self.reset_packingFinished()
                    self._paused = True
                    self._time = 0
                    p = self.default_position.p
                    r = self.default_position.r
                    set_translate(target, Gf.Vec3d(p.x * 100, p.y * 100, p.z * 100))
                    set_rotate(target, Gf.Matrix3d(Gf.Quatd(r.w, r.x, r.y, r.z)))
                    self.reset_step_start()
                else:
                    state = self.ur10_solid.end_effector.status.current_target
                    state_1 = self.pick_and_place.target_position
                    tr = state["orig"] * 100.0
                    set_translate(target, Gf.Vec3d(tr[0], tr[1], tr[2]))
                    set_rotate(target, Gf.Matrix3d(Gf.Quatd(state_1.r.w, state_1.r.x, state_1.r.y, state_1.r.z)))
                if self.pick_and_place.packingFinished:
                    self.packingFinished = True
                    self.pause_tasks()
                if self.pick_and_place.step_start is not None:
                    if self.pick_and_place.step_start:
                        self.pick_and_place_step = True
                    else:
                        self.pick_and_place_step = False
                        self.pause_tasks()
                self._start = False
                self._reset = False

            if self._paused:
                translate_attr = xform_attr.Get().GetRow3(3)
                rotate_x = xform_attr.Get().GetRow3(0)
                rotate_y = xform_attr.Get().GetRow3(1)
                rotate_z = xform_attr.Get().GetRow3(2)

                orig = np.array(translate_attr) / 100.0
                axis_x = np.array(rotate_x)
                axis_y = np.array(rotate_y)
                axis_z = np.array(rotate_z)
                self.ur10_solid.end_effector.go_local(
                    orig=orig,
                    axis_x=axis_x,
                    axis_y=axis_y,
                    axis_z=axis_z,
                    use_default_config=True,
                    wait_for_target=False,
                    wait_time=5.0,
                )

    def create_UR10(self, version, background=True):
        super().create_UR10(version)
        if self.asset_path is None:
            return
        if background:
            self.ur10_table_usd = self.asset_path + "/ur10_palletizer_v1.usd"
            self.env_path = "/environments/env"

            # Load robot environment and set its transform
            create_background(self._stage, self.background_usd, [660 * 2, 300 * 2, -0.5], Gf.Quatd(0, 0, 1, 1000))
            create_ur10(self._stage, self.env_path, self.ur10_table_usd, Gf.Vec3d(641, 300, 118.180),
                        Gf.Quatd(0, 0, 1, 1000))  # 0, 0, 0 637, 300, 118

            # Set robot end effectora
            orig = [-0.0645, 0.7214, 0.495]
            self.default_position = _dynamic_control.Transform()
            self.default_position.p = orig
            self.default_position.r = [-0.33417784954541885, 0.33389792551856345, 0.6230546169232118, 0.6234102056738156]

            GoalPrim = self._stage.DefinePrim(self.env_path + "/target", "Xform")
            p = self.default_position.p
            r = self.default_position.r
            set_translate(GoalPrim, Gf.Vec3d(p.x * 100, p.y * 100, p.z * 100))
            set_rotate(GoalPrim, Gf.Matrix3d(Gf.Quatd(r.w, r.x, r.y, r.z)))

            # Init Box Position
            a = [self.small_klt_usd for i in range(self.start_bin, self.max_bins)]
            b = ["/background/Box{0:02d}".format(i+1) for i in range(self.start_bin, self.max_bins)]
            c = [Gf.Vec3d(-50000 - 50 * i, 150, 0) for i in range(self.max_bins - self.start_bin)]
            create_objects(self._stage, a, b, c)

        # Setup physics simulation
        setup_physics(self._stage)

    def add_bin(self, *args):
        self.create_new_bin(args)

    def create_new_bin(self, *args):
        if self.current_bin < self.max_bins:
            if self.current_bin < self.max_bins:
                i = self.current_bin
                self._dc.set_rigid_body_disable_simulation(self.bin_handles[i], False)

                tf = _dynamic_control.Transform()
                tf.p = [7, 150, -25]
                tf.p.x = -tf.p.x
                tf.p.y = -tf.p.y
                tf.p = [x + y for x, y in zip(tf.p, self.base_pose)]

                self._dc.set_rigid_body_pose(self.bin_handles[i], tf)
                self._dc.set_rigid_body_linear_velocity(self.bin_handles[i], [0, -30.0, 0])
                self._bin_objects[self.bin_paths[i]].unsuppress()
                self.current_bin += 1
                self.unpicked_bins += 1
                self.bin_added_timeout = 100
                self._add_bin_enabled = False

    def disable_bins(self, *args):
        for i in range(self.max_bins - self.start_bin ):
            self._dc.set_rigid_body_disable_simulation(self.bin_handles[i], True)
        self._pending_disable = False
        self.unpicked_bins = 0
        self.bin_added_timeout = 0
        self._add_bin_enabled = True

    def add_new_bin(self):
        if self.current_bin < self.max_bins:
            self.unpicked_bins -= 1
            self.add_bin_timeout = int(random.random() * 200) + 100

    def register_assets(self, *args):
        # Prim path of two blocks and their handles
        prim = self._stage.GetPrimAtPath(self.env_path)
        self.bin_paths = ["/background/Box{0:02d}".format(i+1) for i in range(self.start_bin, self.max_bins)]
        self.bin_handles = [self._dc.get_rigid_body(i) for i in self.bin_paths]

        # Create world and robot object
        ur10_path = str(prim.GetPath()) + "/ur10"
        self.world = World(self._dc, self._mp)
        sgp = Surface_Gripper_Properties()
        sgp.parentPath = ur10_path + "/ee_link"
        sgp.d6JointPath = sgp.parentPath + "/d6FixedJoint"
        sgp.gripThreshold = 1
        sgp.forceLimit = 5.0e5
        sgp.torqueLimit = 5.0e4
        sgp.bendAngle = np.pi / 24  # 7.5 degrees
        sgp.stiffness = 1.0e5
        sgp.damping = 1.0e4
        sgp.disableGravity = True
        tr = _dynamic_control.Transform()
        tr.p.x = 16.2
        sgp.offset = tr

        self.ur10_solid = UR10(
            self._stage,
            self._stage.GetPrimAtPath(ur10_path),
            self._dc,
            self._mp,
            self.world,
            default_config,
            sgp=sgp,
            urdf="/ur10/ur10_robot_suction.urdf",
        )

        i = 0
        for p in self._stage.GetPrimAtPath(str(prim.GetPath()) + "/sortbot_housing/RMPObstacle").GetChildren():
            self.world.register_object(0, p.GetPath().pathString, "housing_" + str(i))
            self.world.make_obstacle("housing_" + str(i), 3, p.GetAttribute("xformOp:scale").Get())
            i += 1

        self._bin_objects = {}

        for i, (bin_handle, bin_path) in enumerate(zip(self.bin_handles, self.bin_paths)):
            self.world.register_object(bin_handle, bin_path, "{}_bin".format(i))
            self.world.make_obstacle("{}_bin".format(i), 3, np.asarray(self.small_bin_scale))
            obj = self.world.get_object_from_name("{}_bin".format(i))
            self._dc.set_rigid_body_disable_simulation(bin_handle, True)
            obj.suppress()
            self._bin_objects[bin_path] = obj
            self._obstacles.append(obj)
        i = 0

        self.pick_and_place = PickAndPlaceStateMachine(
            self._stage,
            self.ur10_solid,
            self._stage.GetPrimAtPath(self.env_path + "/ur10/ee_link"),
            self._bin_objects,
            self.default_position,
            # self._bin_holder_obstacle,
        )
        self.pick_and_place.add_bin = self.add_new_bin


    def perform_tasks(self, *args):
        self._start = True
        self._paused = False
        self.packingFinished = False
        if self.pick_and_place_step is not None:
            return self.pick_and_place_step

    def stop_tasks(self, *args):
        self.reset_packingFinished()
        self.reset_step_start()
        if self.pick_and_place is not None:
            if self._timeline.is_playing():
                self.ur10_solid.stop()
                self._reset = True
                self.current_bin = 0
                self.add_bin_timeout = -1
                self._pending_disable = True
                for i in range(self.max_bins - self.start_bin):
                    tf = _dynamic_control.Transform()
                    tf.p = [-50000 - 50 * i, 150, 0]
                    self._dc.set_rigid_body_pose(self.bin_handles[i], tf)
                    self._dc.set_rigid_body_linear_velocity(self.bin_handles[i], [0, 0, 0])
                    self._dc.set_rigid_body_angular_velocity(self.bin_handles[i], [0, 0, 0])
                self._pending_stop = False
            else:
                self._pending_stop = True

    def pause_tasks(self, *args):
        # self._paused = not self._paused
        self._paused = True
        if self._paused:
            selection = omni.usd.get_context().get_selection()
            selection.set_selected_prim_paths(["/environments/env/target"], False)
            target = self._stage.GetPrimAtPath("/environments/env/target")
            xform_attr = target.GetAttribute("xformOp:transform")
            translate_attr = np.array(xform_attr.Get().GetRow3(3))
            if np.linalg.norm(translate_attr) < 0.01:
                p = self.default_position.p
                r = self.default_position.r
                set_translate(target, Gf.Vec3d(p.x * 100, p.y * 100, p.z * 100))
                set_rotate(target, Gf.Matrix3d(Gf.Quatd(r.w, r.x, r.y, r.z)))

        return self._paused

    def packing_result(self):
        return self.packingFinished

    def reset_packingFinished(self):
        self.packingFinished = False
        if self.pick_and_place is not None:
            self.pick_and_place.packingFinished = False

    def reset_step_start(self):
        self.pick_and_place_step = None
        if self.pick_and_place is not None:
            self.pick_and_place.step_start = None
## choi
    def getObjectMass(self, prim_path):
        max_payload = 0.56  # temp UR10 maxPayload
        object_mass = 0.56
        # print("@@@@ prim_path @@@@ ", prim_path)

        # if the mass property exists,
        if prim_path.HasAPI(UsdPhysics.MassAPI):
            MassAPI = UsdPhysics.MassAPI.Get(self._stage, prim_path.GetPath())

        # if the mass property does not exist,
        else:
            MassAPI = UsdPhysics.MassAPI.Apply(prim_path)
            MassAPI.CreateMassAttr(object_mass)
            MassAPI = UsdPhysics.MassAPI.Get(self._stage, prim_path.GetPath())

        mass = MassAPI.GetMassAttr().Get()

        if mass > max_payload:  # cannot load over maxPayload
            MassAPI.CreateMassAttr(object_mass)
            mass = MassAPI.GetMassAttr().Get()

        return mass

    def getObjectPose(self, prim_path):
        translate_attr, rotation_attr = '', ''
        properties = prim_path.GetPropertyNames()
        if "xformOp:translate" in properties:
            translate_attr = prim_path.GetAttribute("xformOp:translate").Get()
            rotation_attr = prim_path.GetAttribute("xformOp:orient").Get()

        return translate_attr, str(rotation_attr)

    def getObjectProperty(self, prim_path):
        propertiesDict = {}
        properties = prim_path.GetPropertyNames()
        if "physics:angularVelocity" in properties:
            angularVelocity = prim_path.GetAttribute("Angular Velocity").Get()
            propertiesDict['angularVelocity'] = angularVelocity
        if "physics:velocity" in properties:
            linearVelocity = prim_path.GetAttribute("Linear Velocity").Get()
            propertiesDict['LinearVelocity'] = linearVelocity

        return propertiesDict

    def get_AABB(self, prim_path):
        bbox_cache = create_bbox_cache()
        aabb_list = compute_aabb(bbox_cache, prim_path)
        return aabb_list/100

    def get_AABB_list(self, prim_list):
        bbox_cache = create_bbox_cache()
        aabb_list = compute_combined_aabb(bbox_cache, prim_list)
        return aabb_list/100
