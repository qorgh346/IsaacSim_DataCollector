# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import copy
import numpy as np
from omni.isaac.demos.utils import math_utils
from .franka import LookAtCommander
from .state_machine import State, HierarchicalState, Behavior, NextStateTransition, ThinkAndRun, MultiHierarchicalState
from .behavior_helpers import (
    set_gripper_to_push_width,
    close_open,
    ApproachParams,
    natural_push_axis_y,
    project_block_transform_to_table,
    MakeNaturalPickInfo,
    get_block_id,
    get_block_name,
)
from .reactive_behavior import FrameTerminationCriteria


class BlocksWorldSuppressors(object):
    """ Collection of suppressors used for control of block world collision suppression.

    Includes suppressors for each block and the table.
    """

    def __init__(self, franka, world, block_colors):
        """ Initialize all suppressors. """
        self.block_suppressors = [world.get_object_from_name(get_block_id(color)) for color in block_colors]
        self.table_suppressor = world.get_object_from_name("table")

    def suppress_blocks(self):
        """ Suppress all blocks. """
        for suppressor in self.block_suppressors:
            suppressor.suppress()

    def unsuppress_blocks(self):
        """ Unsuppress all blocks. """
        for suppressor in self.block_suppressors:
            suppressor.unsuppress()

    def suppress_everything(self):
        """ Suppress everything, including all blocks and the table. """
        self.suppress_blocks()
        self.table_suppressor.suppress()

    def unsuppress_everything(self):
        """ Unsuppress everything, including all blocks and the table. """
        self.unsuppress_blocks()
        self.table_suppressor.unsuppress()


class LogicalState(object):
    def __init__(self):
        self.block_tower_state = []  # List of block colors current in the block tower stack (bottom to top).
        self.active_block = None  # Current active block (can be None)
        self.gripper_block_state = None  # Gives the block that is in the gripper (can be None)

    def tower_height(self):
        """ Returns the current number of blocks stacked on the tower.
        """
        return len(self.block_tower_state)

    def print(self):
        print("---- logical state ----")
        print("tower state:", self.block_tower_state)
        print("active block:", self.active_block)
        print("gripper block state:", self.gripper_block_state)
        print()


class Domain(object):
    """ Structure containing everything of interest to make it easy to pass information and tools
    around.

    Includes all commanders for controlling the robot (franka object, a look-at commander which uses
    the newer TaskSpaceCommander interface to control the look-at target, and a config modulator for
    setting the default config), suppressors, perception interfaces, an meta information about the
    blocks and various constants needed by the behaviors.

    In most cases, behavior state machines are passed an object of this type on construction so they
    internally have access to all of this information.
    """

    def __init__(
        self, franka, ghost_domains, blocks_world_suppressors, block_colors, block_locations, step_rate_hz=30.0
    ):
        self.franka = franka
        self.ghost_domains = ghost_domains
        self.look_at_commanders = {"short": LookAtCommander(self.franka)}
        self.blocks_world_suppressors = blocks_world_suppressors
        self.block_colors = block_colors
        self.block_locations = block_locations
        self.step_rate_hz = step_rate_hz
        self.step_rate = 1.0 / step_rate_hz

        self.logical_state = LogicalState()

        self.up = np.array([0.0, 0.0, 1.0])
        self.down = -self.up
        self.grasp_height = 0.035
        self.block_height = 0.0515
        self.grasp_depth = self.block_height - self.grasp_height
        self.goal_x = 0.5
        self.goal_y = 0.0
        self.tower_location = np.array([self.goal_x, self.goal_y, 0.0])
        self.time = 0
        self.stop = False

        self.close_force = 0.1

    def __del__(self):
        print("Delete Domain")
        self.franka = None
        self.ghost_domains = None

    def gripper_closed(self):
        return False

    def clear_all_commanders(self):
        for tag, commander in self.look_at_commanders.iteritems():
            commander.clear()
        self.franka.end_effector.go_local(orig=[], axis_x=[], axis_y=[], axis_z=[], use_default_config=True)

    def num_blocks(self):
        return len(self.block_colors)

    def visualize_target(self, T):
        pass

    def visualize_block(self, T, rgba=[0, 1, 1, 0.8]):
        pass

    def rand_block_index(self):
        """ Returns a uniformely distributed random block color. """
        return np.random.randint(0, len(self.block_colors))

    def rand_block_color(self):
        """ Returns a uniformely distributed random block color. """
        return self.block_colors[self.rand_block_index()]

    def transforms(self):
        """ Collects the homogeneous transforms of the blocks in the order specified by the
        block_colors member passed in on construction.
        """
        return [(self.block_locations.get_T("00_block_" + color), color) for color in self.block_colors]

    def azure_position_transforms(self):
        """ Collects the homogeneous transforms of the blocks in the order specified by the
        block_colors member passed in on construction.
        """
        return self.transforms()

    def current_end_effector_frame(self):
        """ Returns the current end-effector frame as a dictionay containing frame element tags
        {'orig', 'axis_x', 'axis_y', 'axis_z'} (each mapping to a 3D numpy vector).
        """
        return self.franka.end_effector.status.current_frame

    def send_block_locations_to_backend(self):
        """ Send all the latest block locations to the backend projected onto the table.
        """

    def tick(self, dt):
        if self.ghost_domains is not None:
            for ghost_domain in self.ghost_domains:
                ghost_domain.tick(dt)
        self.time = self.time + dt


class Retract(State):
    def __init__(self, domain, open_gripper=False):
        self.domain = domain
        self.open_gripper = open_gripper
        self.orig = [0.2743133008480072, 5.1372178859310225e-05, 0.4564969837665558]

    def enter(self):
        print("Entering retract")
        if self.open_gripper:
            self.domain.franka.end_effector.gripper.commander.open(wait=False, speed=0.2)
        else:
            self.domain.franka.end_effector.gripper.move(width=0.0, speed=0.2, wait=False)
        self.domain.franka.end_effector.go_local(
            orig=self.orig, axis_x=[], axis_y=[], axis_z=[], use_default_config=True, wait_for_target=False
        )


class MoveUp(State):
    """ Moves the end-effector up slightly for a give time duration.

    Moves toward a target .1 meters up from its current end-effector location, but transitions away
    after wait_time seconds. If wait_time isn't set, transitions immediately. Transitions to the
    specified next_state (can be set after construction by setting the member).
    """

    def __init__(self, domain, wait_time=None, next_state=None):
        self.domain = domain
        self.next_state = next_state
        self.wait_time = wait_time

    def enter(self):
        self.start_time = self.domain.time

        perturbation_up = np.array([0.0, 0.0, 0.1])
        ee_orig = self.domain.franka.end_effector.status.orig
        self.domain.franka.end_effector.go_local(
            orig=ee_orig + perturbation_up, use_default_config=True, wait_for_target=False, wait_time=5.0
        )

    def transition(self):
        if self.wait_time is not None and (self.domain.time - self.start_time) < self.wait_time:
            return self
        return self.next_state


class MoveTowardOrig(State):
    """ Moves the end-effector toward a point above the origin for a given time duration.

    Perturbs .1 meters toward a point at (0, 0, .7) (.7 meters above the origin). If wait_time is
    specified, waits the given amount of time (seconds) before transitioning.  Transitions to the
    specified next_state (can be set after construction by setting the member).
    """

    def __init__(self, domain, wait_time=None, next_state=None):
        self.domain = domain
        self.next_state = next_state
        self.wait_time = wait_time

    def setup(self, wait_time=None):
        self.wait_time = wait_time

    def enter(self):
        self.start_time = self.domain.time

        p = np.array([0.0, 0.0, 0.7])
        o = self.domain.franka.end_effector.status.orig
        v = math_utils.normalized(p - o)
        self.domain.franka.end_effector.go_local(orig=o + 0.1 * v, use_default_config=True, wait_for_target=False)

    def transition(self):
        if self.wait_time is not None and (self.domain.time - self.start_time) < self.wait_time:
            return self
        else:
            return self.next_state


class RetractABit(HierarchicalState):
    """ Retract the end-effector a bit by lifting it up using MoveUp and (optionally) moving it
    toward the origin using MoveTowardOrig.

    The move_up_wait_time is hard coded currently to .2.
    """

    def __init__(self, domain, use_move_toward_orig=True, wait_time=None):
        self.domain = domain
        move_up_wait_time = 0.2
        self.move_up = MoveUp(self.domain, wait_time=move_up_wait_time)

        if use_move_toward_orig:
            second_stage_wait_time = 0.0
            if wait_time is not None:
                second_stage_wait_time = max(0.0, wait_time - move_up_wait_time)
            self.move_toward_orig = MoveTowardOrig(self.domain, wait_time=second_stage_wait_time)
            self.move_up.next_state = self.move_toward_orig
        super().__init__(self.move_up)


def get_default_config_by_target_orig(target_orig):
    # Bent so head is kind of vertical
    default_config_far = np.array(
        [
            -0.04501747728389455,
            0.5396176035907823,
            0.03117166895484715,
            -1.7757421112660823,
            -0.054097083644728934,
            2.2946507021586102,
            0.8159214135450051,
        ]
    )

    # Default config from the robot descriptor
    default_config_near = np.array([0.00, -1.3, 0.00, -2.87, 0.00, 2.00, 0.75])
    x = target_orig[0]
    x_min = 0.4
    x_max = 0.6
    if x < x_min:
        alpha = 0.0
    elif x > x_max:
        alpha = 1.0
    else:
        alpha = (x - x_min) / (x_max - x_min)
    default_config = alpha * default_config_far + (1.0 - alpha) * default_config_near
    return default_config


class PinchBlock(State):
    """ Pinch block primitive.

    Reaches toward a specified target from above maintaining the given target_axis_y partial
    orientation constraint, and pinches and releases the block.

    The step() method is lightweight as usual until the pinch behavior. It blocks for the entire
    duration of the pinch and release behavior (effectively pausing the state machine) since that
    behavior should not be preempted.

    Used in the PinchAlign behavior.
    """

    def __init__(self, domain, target_orig, target_axis_y):
        self.domain = domain
        self.target = {"orig": target_orig, "axis_y": target_axis_y}
        self.termination_criteria = FrameTerminationCriteria(
            orig_thresh=0.001, axis_x_thresh=0.001, axis_y_thresh=0.001, axis_z_thresh=0.001
        )
        self.next_state = None

    def enter(self):
        # input("Pinch block")
        self.domain.franka.end_effector.gripper.open(wait=False)
        self.domain.franka.send_config(get_default_config_by_target_orig(self.target["orig"]))
        self.domain.franka.end_effector.go_local(
            target=self.target,
            approach_direction=self.domain.down,
            approach_standoff=0.2,
            approach_standoff_std_dev=0.001,
            use_default_config=False,
            wait_for_target=False,
        )
        self.is_done = False

    def has_arrived(self):
        return self.termination_criteria(self.target, self.domain.current_end_effector_frame())

    def step(self):
        """ When then go_local has reached its target, this method will block and run a close_open()
        call.
        """
        if self.has_arrived():
            close_open(self.domain.franka, self.domain.block_height)
            self.is_done = True

    def transition(self):
        if self.is_done:
            return self.next_state
        return self


class RotateTowardAxisRotatePartially(State):
    def __init__(self, domain, final_target_axis_y, up_only=False, max_height=0.1, rotation_time=0.5):
        """ Rotate smoothly toward the given axis. 

        The name of this state is a misnomer. It actually rotates entirely toward the axis.
        Previously it was used in conjunction with a shift up state, but now all functionality is
        implemented here.
        """
        self.domain = domain
        self.final_target_axis_y = final_target_axis_y
        self.up_only = up_only

        self.termination_criteria = FrameTerminationCriteria(
            orig_thresh=0.001, axis_x_thresh=0.001, axis_y_thresh=0.001, axis_z_thresh=0.001
        )
        self.rotation_time = rotation_time
        self.next_state = None
        self.max_height = max_height

    def enter(self):
        self.start_orig = self.domain.franka.end_effector.status.orig
        self.start_axis_y = self.domain.franka.end_effector.status.axis_y

        self.start_time = self.domain.time

    def step(self):
        elapse_secs = self.domain.time - self.start_time
        eps = 0.0
        alpha = eps + (1.0 - eps) * min(1.0, elapse_secs / self.rotation_time)
        if self.up_only:
            max_th = np.pi / 2
            approach_direction = self.domain.down
        else:
            max_th = np.pi
            approach_direction = self.domain.up
        th = alpha * max_th

        delta_height = self.max_height * np.sin(th)

        orig = self.start_orig + delta_height * self.domain.up
        # eps = .25
        eps = 0.5
        alpha_shifted = (1 + 2 * eps) * alpha - eps
        alpha2 = min(1.0, max(0.0, alpha_shifted))
        axis_y = math_utils.normalized(alpha2 * self.final_target_axis_y + (1.0 - alpha2) * self.start_axis_y)
        # print("axis_y:", axis_y)

        self.target = {"orig": orig, "axis_y": axis_y}

        self.domain.franka.send_config(get_default_config_by_target_orig(self.target["orig"]))
        self.domain.franka.end_effector.go_local(
            target=self.target,
            approach_direction=approach_direction,
            approach_standoff=0.2,
            approach_standoff_std_dev=0.001,
            use_default_config=False,
            wait_for_target=False,
        )

    def exit(self):
        pass

    def has_arrived(self):
        return self.termination_criteria(self.target, self.domain.current_end_effector_frame())

    def transition(self):
        if (self.domain.time - self.start_time) >= self.rotation_time:
            return self.next_state
        else:
            return self


class RotateTowardAxis(HierarchicalState):
    """ Rotates toward the specified axis. Enables smooth rotations transitioning between
    vastly different orientations.
    """

    def __init__(self, domain, final_target_axis_y, up_only=False, max_height=0.1, rotation_time=0.5):
        self.domain = domain

        self.rotate_state = RotateTowardAxisRotatePartially(
            domain, final_target_axis_y, up_only, max_height, rotation_time
        )
        self.next_state = None

        super().__init__(self.rotate_state)

    def transition(self):
        next_state = super().transition()
        if next_state is not None:
            return next_state
        else:
            return self.next_state


class PinchAlignLift(State):
    """ Final lift state of the pinch alignment behavior.

    Lifts .03 meters and transitions when it's arrives using default termination criteria.
    """

    def __init__(self, domain, next_state=None):
        self.domain = domain
        self.next_state = next_state
        self.termination_criteria = FrameTerminationCriteria()

    def enter(self):
        current_orig = self.domain.franka.end_effector.status.orig
        current_axis_y = self.domain.franka.end_effector.status.axis_y
        self.target = {"orig": current_orig + 0.05 * self.domain.up, "axis_y": current_axis_y}
        self.domain.franka.end_effector.go_local(
            target=self.target,
            approach_direction=self.domain.up,
            approach_standoff=0.07,
            approach_standoff_std_dev=0.001,
            use_default_config=False,
            wait_for_target=False,
        )

    def has_arrived(self):
        return self.termination_criteria(self.target, self.domain.current_end_effector_frame())

    def transition(self):
        if self.has_arrived():
            return self.next_state
        else:
            return self

    def exit(self):
        self.domain.blocks_world_suppressors.unsuppress_everything()


class PinchAlignCycle(HierarchicalState):
    """ Composite (hierarchical) pinch alignment behavior.

    Cycles for the specified number of cycles. Exits by running the PinchAlignLift behavior only
    after finishing all cycles.

    Each cycle consists of two pinches defined by two difference target axis_y constraints on the
    same target orig. These targets are for the end-effector, so they need to be shifted up from the
    block center location.

    No next state is currently implemented.
    """

    def __init__(
        self, domain, pinch_target_orig, pinch_target_axis_y1, pinch_target_axis_y2, start_from_pinch_block_state=False
    ):
        self.domain = domain
        self.rotate_toward_pinch1 = RotateTowardAxis(
            domain, pinch_target_axis_y1, up_only=True, max_height=0, rotation_time=0.5
        )
        self.pinch_block1 = PinchBlock(domain, pinch_target_orig, pinch_target_axis_y1)
        self.rotate_toward_pinch2 = RotateTowardAxis(
            domain, pinch_target_axis_y2, up_only=True, max_height=0.07, rotation_time=2.0
        )
        self.pinch_block2 = PinchBlock(domain, pinch_target_orig, pinch_target_axis_y2)
        self.lift = PinchAlignLift(domain)

        self.rotate_toward_pinch1.next_state = self.pinch_block1
        self.pinch_block1.next_state = self.rotate_toward_pinch2
        self.rotate_toward_pinch2.next_state = self.pinch_block2
        self.pinch_block2.next_state = self.lift

        if start_from_pinch_block_state:
            start_state = self.pinch_block1
        else:
            start_state = self.rotate_toward_pinch1
        super().__init__(start_state)

    def enter(self):
        super().enter()

    def transition(self):
        """ Cycle for the specified number of cycles and then terminate by returning None
        """
        next_state = super().transition()
        if next_state is not None:
            return next_state
        else:
            return None


class PinchAlign(HierarchicalState):
    """ Run pinch alignment for the specified block.

    Uses the "best" perception info provided by the BlocksPerceptionTracker to determine how to
    pinch. That info can be a "expected" location created by a previous alignment behavior.
    """

    def __init__(
        self,
        domain,
        block_index=None,
        suppress_block=True,
        maintain_block_suppression_on_exit=False,
        start_from_pinch_block_state=False,
    ):
        self.domain = domain
        self._block_index = block_index
        self.suppress_block = suppress_block
        self.maintain_block_suppression_on_exit = maintain_block_suppression_on_exit
        self.start_from_pinch_block_state = start_from_pinch_block_state

        # The initial state will be filled in below.
        super().__init__(init_state=None)

    @property
    def block_index(self):
        if self._block_index is not None:
            return self._block_index
        else:
            return self.domain.logical_state.active_block

    def setup(self):
        T = self.domain.block_locations.get_T(get_block_name(self.block_index, self.domain.block_colors))
        block_orig = T[:3, 3]
        axis_y = self.domain.franka.end_effector.status.axis_y

        ys = [T[:3, i] for i in range(3)]
        z = np.array([0, 0, 1])
        scores = [np.abs(np.dot(y, z)) for y in ys]

        indices = [i for i, s in enumerate(scores) if s < 0.1]
        candidate_ys = [ys[i] for i in indices]
        list_thing = [(y, np.abs(np.dot(y, axis_y))) for y in candidate_ys]
        pinch_axis_y1, _ = max(list_thing, key=lambda entry: entry[1])
        if np.dot(axis_y, pinch_axis_y1) < 0.0:
            pinch_axis_y1 = -pinch_axis_y1

        pinch_axis_y2, _ = min(list_thing, key=lambda entry: entry[1])
        if np.dot(axis_y, pinch_axis_y2) < 0.0:
            pinch_axis_y2 = -pinch_axis_y2

        # If the x-axis target isn't facing toward the robot's base, flip it around so it is.
        away_from_robot = copy.deepcopy(block_orig)
        away_from_robot[2] = 0.0
        orth_to_from_robot = np.array([-away_from_robot[1], away_from_robot[0], 0.0])

        if np.dot(pinch_axis_y1, orth_to_from_robot) < 0.0:
            pinch_axis_y1 = -pinch_axis_y1

        if np.dot(pinch_axis_y2, orth_to_from_robot) < 0.0:
            pinch_axis_y2 = -pinch_axis_y2

        pinch_target_orig = copy.deepcopy(block_orig)
        pinch_target_orig[0] = self.domain.goal_x
        pinch_target_orig[1] = self.domain.goal_y
        pinch_target_orig[2] += self.domain.block_height / 2 - self.domain.grasp_depth
        pinch_target_axis_y1 = pinch_axis_y1
        pinch_target_axis_y2 = pinch_axis_y2
        pinch_align_cycle = PinchAlignCycle(
            self.domain,
            pinch_target_orig,
            pinch_target_axis_y1,
            pinch_target_axis_y2,
            start_from_pinch_block_state=self.start_from_pinch_block_state,
        )

        init_state = pinch_align_cycle
        super().__init__(init_state)

    def enter(self):
        self.setup()
        if self.suppress_block:
            self.domain.blocks_world_suppressors.block_suppressors[self.block_index].suppress()
        super().enter()

    def exit(self):
        if self.suppress_block and not self.maintain_block_suppression_on_exit:
            self.domain.blocks_world_suppressors.block_suppressors[self.block_index].unsuppress()


default_config_near = np.array(
    [
        -0.055515498785073294,
        0.14637242948826007,
        0.04632050262342223,
        -2.2019981154057064,
        -0.0030298385539081723,
        2.833585982004801,
        0.8042955116331576,
    ]
)


class LookAt(State):
    """ Look at a specific point by controlling the look-at frame.

    Gets the look at point from domain.block_locations. If block_index is specified looks at that
    block. Otherwise, looks at a random block if block_index is None.
    """

    def __init__(
        self,
        domain,
        look_at_commander_tag="short",
        update_continuously=False,
        block_index="random",
        look_time=1.0,
        next_state=None,
    ):
        self.domain = domain
        self._block_index = block_index
        self.look_time = look_time
        self.look_at_commander = self.domain.look_at_commanders[look_at_commander_tag]
        self.update_continuously = update_continuously
        self.look_at_point = None
        self.next_state = next_state
        self.is_done = True
        self.random_index = None

    @property
    def block_index(self):
        if self.random_index is not None:
            return self.random_index

        if self._block_index is not None:
            return self._block_index
        else:
            if self.domain.logical_state.active_block is None:
                raise RuntimeError("active block is None in logical state")
            return self.domain.logical_state.active_block

    def send_look_at(self):
        color = self.domain.block_colors[self.block_index]

        block_T = self.domain.block_locations.get_T("00_block_" + color)
        self.look_at_point = block_T[:3, 3]
        self.domain.visualize_block(block_T, [1, 0.5, 1, 0.8])
        self.look_at_commander.go_pos(pos=self.look_at_point)

    def enter(self):
        if self._block_index == "random":
            self.random_index = self.domain.rand_block_index()
        else:
            self.random_index = None

        self.send_look_at()
        self.domain.franka.send_config(default_config_near)
        self.domain.franka.end_effector.go_local(orig=[], axis_x=[], axis_y=[], axis_z=[], use_default_config=False)
        self.converged_time = None
        self.is_done_start = None

    def step(self):
        if self.update_continuously:
            self.send_look_at()

        # Do nothing. We've already sent the command. In the future, we might update the look at
        # position.
        pass

    def transition(self):
        if self.look_at_commander.wait_for_pos(error_thresh=0.01, check_only=True):
            # Transition immediately when we've converged. Looking at the block is only for effect
            # currently.
            # print("<Converged. Transitioning>")
            now = self.domain.time
            if self.converged_time is None:
                self.converged_time = now
            elif (now - self.converged_time) >= self.look_time:
                return self.next_state
            elif self.is_done is not None:
                if self.is_done_start is None:
                    self.is_done_start = now
                elif (now - self.is_done_start) >= 2.0:
                    return self.next_state
            return self
        else:
            self.converged_time = None
        # print("<not converged>")
        return self  # Don't transition

    def exit(self):
        print("Exiting look at")
        self.domain.franka.end_effector.freeze()
        self.look_at_commander.clear()


class ApproachTarget(State):
    """ Base class for approaching a target smoothly over a long distance.

    Includes smoothing out of the rotation so it doesn't whip itself around as strongly. Ideally
    that should be implemented using exponential map geometry at the RMP level, but these
    constraints are often partial pose constraints (just a single rotational axis constraint), so
    implementing that is nontrivial. This base class instead implements an incremental blending from
    the current axis toward the target axis contraint.
    """

    def __init__(self, domain, use_default_config=False, next_state=None):
        self.domain = domain
        self.use_default_config = use_default_config
        self.next_state = next_state

    def set_target(
        self, target, approach, termination_criteria=FrameTerminationCriteria(), rotation_blend_std_dev=0.15
    ):
        """ Set the target for this state along with approach parameters. target should be a dict of
        frame elements and approach should has the ApproachParams interface.
        """
        self.target = target
        self.approach = approach
        self.termination_criteria = termination_criteria
        self.rotation_blend_std_dev = rotation_blend_std_dev

    def calc_current_target(self):
        """ Calculate the current target that blends the axis constraints for smoother rotations.
        """
        partial_pose = copy.deepcopy(self.target)
        current_frame = self.domain.franka.end_effector.status.frame

        dist = np.linalg.norm(partial_pose["orig"] - current_frame["orig"])
        dist_scaled = dist / self.rotation_blend_std_dev
        alpha = np.exp(-0.5 * dist_scaled * dist_scaled)
        for tag in ["axis_x", "axis_y", "axis_z"]:
            if tag in partial_pose:
                partial_pose[tag] = alpha * partial_pose[tag] + (1.0 - alpha) * current_frame[tag]

        return partial_pose

    def step(self):
        self.domain.franka.send_config(default_config_near)
        self.domain.franka.end_effector.go_local(
            target=self.calc_current_target(),
            approach_direction=self.approach.direction,
            approach_standoff=self.approach.standoff,
            approach_standoff_std_dev=self.approach.standoff_std_dev,
            use_default_config=self.use_default_config,
            wait_for_target=False,
        )

    def has_arrived(self):
        return self.termination_criteria(self.target, self.domain.current_end_effector_frame())

    def transition(self):
        # print("<checking transition>")
        if self.has_arrived():
            print("<transitioning to push block state>")
            return self.next_state

        # print("<no transition>")
        return self


class ApproachPush(ApproachTarget):
    """ Approach a push behavior using the ApproachTarget interface.
    """

    def setup(self, start_partial_pose, approach):
        self.set_target(start_partial_pose, approach)

    def enter(self):
        print("<enter approach push>")
        print("Setting to push width")
        set_gripper_to_push_width(self.domain.franka)
        print("push width set")
        print("Returning from enter()")


class PushBlock(State):
    """ Push along a specified push direction maintaining an initial partial orientation constraint.
    The push is defined by the start partial pose (starting from the given orig and maintaining the
    partial orientation constraint throughout), and pushes along the push direction for the
    specified push distance for the specified push time (defines the speed (up to added convergence
    time)).
    """

    def __init__(self, domain, next_state=None):
        self.domain = domain
        self.next_state = next_state

        self.termination_criteria = FrameTerminationCriteria()

        self.start_partial_pose = None

    def setup(self, start_partial_pose, push_direction, push_dist, push_time):
        self.start_partial_pose = start_partial_pose
        self.push_direction = push_direction
        self.push_dist = push_dist
        self.push_time = push_time

        num_steps = self.push_time * self.domain.step_rate_hz
        print("num_steps:", num_steps)
        self.steps = np.linspace(0, self.push_dist, num_steps + 1)
        self.step_index = 0

    def enter(self):
        print("<enter push block>")
        self.final_target = None

    def step(self):
        print("<step push_block>")
        if self.step_index >= len(self.steps):
            return

        is_last = self.step_index == len(self.steps) - 1
        d = self.steps[self.step_index]

        target = copy.deepcopy(self.start_partial_pose)
        target["orig"] += d * self.push_direction
        self.domain.franka.send_config(default_config_near)
        self.domain.franka.end_effector.go_local(target=target, use_default_config=False, wait_for_target=False)
        self.step_index += 1

        if is_last:
            self.final_target = target

    def is_done(self):
        """ Is done when it's reaches its final target. This will be an amount of time after the
        push duration allowing the system to converge.
        """
        if self.final_target is not None and self.termination_criteria(
            self.final_target, self.domain.franka.end_effector.status.current_frame
        ):
            return True
        return False

    def transition(self):
        if self.is_done():
            return self.next_state
        return self


class Push(HierarchicalState):
    """ Full hierarchical push block state.

    ApproachPush --> PushBlock --> RetractABit
    """

    def __init__(self, domain, only_lift_on_finish=False):
        self.domain = domain

        # Created in reverse order to set up next state transitions
        self.retract_a_bit = RetractABit(self.domain, use_move_toward_orig=(not only_lift_on_finish), wait_time=0.3)
        self.push_block = PushBlock(self.domain, next_state=self.retract_a_bit)
        self.approach_push = ApproachPush(self.domain, next_state=self.push_block)
        super().__init__(self.approach_push)

    def setup(self, start_partial_pose, approach, push_direction, push_dist, push_time):
        self.approach_push.setup(start_partial_pose, approach)
        self.push_block.setup(start_partial_pose, push_direction, push_dist, push_time)


class SlideBlock(Push):
    """ Semantic interface to the Push behavior.

    Defines the push parameters in terms of block semantics. push_standoff_alpha is a fraction of
    the block height (cube side length) to standoff the push, and push_depth is the depth under the
    block's height push set the end-effector during the push.

    If only_lift_on_finish at the end of the behavior it will only lift straight up inteh
    RetractABit behavior. Otherwise, both lifts and retracts toward the orig.
    """

    def __init__(self, domain, push_standoff_alpha, push_depth, push_time, only_lift_on_finish=False, next_state=None):
        super().__init__(domain, only_lift_on_finish)
        self.push_standoff_alpha = push_standoff_alpha
        self.push_depth = push_depth
        self.push_time = push_time
        self.next_state = next_state

    def setup(self, block_location, push_direction, push_dist, target_axis_y=None):
        target_orig = block_location - self.push_standoff_alpha * self.domain.block_height * push_direction
        target_orig[2] = self.domain.block_height - self.push_depth
        toward_robot = -block_location

        if target_axis_y is None:
            if np.dot(push_direction, toward_robot) > 0:
                target_axis_y = np.cross(push_direction, self.domain.up)
            else:
                target_axis_y = np.cross(-push_direction, self.domain.up)
            target_axis_y = math_utils.normalized(target_axis_y)

        self.start_partial_pose = {"orig": target_orig, "axis_y": target_axis_y}
        approach = ApproachParams(self.domain.down)
        super().setup(self.start_partial_pose, approach, push_direction, push_dist, self.push_time)

    def transition(self):
        next_state = super().transition()
        if next_state is not None:
            return next_state
        return self.next_state


class PushAwayFromTower(SlideBlock):
    """ Slide a block away from the tower locations to clear out that region.

    Implement the violation resolver interface by implementing find_and_record_violations(). That
    method will be called before enter() to population a self.violations list member.
    """

    def __init__(self, domain, required_radius, push_standoff_alpha, push_depth, push_time, return_to_state=None):
        self.required_radius = required_radius
        self.return_to_state = return_to_state

        self.violations = []

        super().__init__(
            domain=domain, push_standoff_alpha=push_standoff_alpha, push_depth=push_depth, push_time=push_time
        )

    def find_and_record_violations(self):
        """ Finds and records the current violations. This method is called before each transition
        in, so the violations will be available on entry.
        """

        current_tower = copy.deepcopy(self.domain.logical_state.block_tower_state)
        if len(current_tower) == 0:
            current_tower.append(0)

        self.violations = []
        for T, color in self.domain.azure_position_transforms():
            i = self.domain.block_colors.index(color)
            if i in current_tower:
                continue

            p = T[:3, 3]
            print("block location:", p, ", tower_location:", self.domain.tower_location)
            v = p - self.domain.tower_location
            dist = np.linalg.norm(v)
            print("checking: %s, dist: %f" % (color, dist))

            if dist < self.required_radius:
                print("<violation>")
                self.violations.append(((p, color), self.required_radius - dist))
            else:
                print("<clear>")
            print()
        return len(self.violations) > 0

    def enter(self):
        """ Choses a random violation to resolve on entry.
        """
        print("Entering push")
        self.start_time = self.domain.time

        set_gripper_to_push_width(self.domain.franka)

        print("violations:", self.violations)
        (p, active_color), _ = max(self.violations, key=lambda entry: entry[1])

        push_direction = p - self.domain.tower_location
        # If the push direction is zero, choose a random direction.
        while np.linalg.norm(push_direction) == 0.0:
            push_direction = np.random.randn(3)
        push_direction[2] = 0.0  # Ensure horizontal
        push_direction = math_utils.normalized(push_direction)

        margin = 0.03
        push_dist = self.required_radius - np.linalg.norm(p - self.domain.tower_location) + margin
        self.setup(p, push_direction, push_dist)
        super().enter()

    def transition(self):
        next_state = super().transition()
        if next_state is not None:
            return next_state
        return self.return_to_state


class PushBlocksApart(HierarchicalState):
    """ Push two blocks away from each other to separate them by a particular distance.

    Implement the violation resolver interface by implementing find_and_record_violations(). That
    method will be called before enter() to population a self.violations list member.
    """

    def __init__(self, domain, required_separation, push_standoff_alpha, push_depth, push_time, return_to_state=None):
        self.domain = domain
        self.required_separation = required_separation
        self.push_standoff_alpha = push_standoff_alpha
        self.push_depth = push_depth
        self.push_time = push_time
        self.return_to_state = return_to_state

        self.violations = []

        # Sets init_state to None initially but it's set up correctly in enter()
        super().__init__(init_state=None)

    def find_and_record_violations(self):
        self.violations = []
        block_colors = self.domain.block_colors
        self.transforms = [T for T, _ in self.domain.azure_position_transforms()]
        for i in range(len(block_colors)):
            a_in_tower = i in self.domain.logical_state.block_tower_state
            T_i = self.transforms[i]
            p_i = T_i[:3, 3]
            for j in range(i + 1, len(block_colors)):
                b_in_tower = i in self.domain.logical_state.block_tower_state
                T_j = self.transforms[j]
                p_j = T_j[:3, 3]

                if a_in_tower and b_in_tower:
                    continue

                dist = np.linalg.norm(p_i - p_j)
                print("(%d vs %d) dist: %f" % (i, j, dist))
                if dist < self.required_separation:
                    self.violations.append(((i, j), dist))
        return len(self.violations) > 0

    def enter(self):
        set_gripper_to_push_width(self.domain.franka)

        rand_violation, _ = min(self.violations, key=lambda entry: entry[1])
        print("rand_violation:", rand_violation)
        p1 = self.transforms[rand_violation[0]][:3, 3]
        p2 = self.transforms[rand_violation[1]][:3, 3]
        v = math_utils.normalized(p2 - p1)
        v[2] = 0.0  # Ensure horizontal
        push_dist = self.required_separation / 2

        slide_block1 = SlideBlock(
            self.domain, self.push_standoff_alpha, self.push_depth, self.push_time, only_lift_on_finish=True
        )
        slide_block1.setup(p1, -v, push_dist)
        target_axis_y = slide_block1.start_partial_pose["axis_y"]  # Second slide should use the same axis_y constraint

        slide_block2 = SlideBlock(self.domain, self.push_standoff_alpha, self.push_depth, self.push_time)
        slide_block2.setup(p2, v, push_dist, target_axis_y)

        slide_block1.next_state = slide_block2
        self.init_state = slide_block1

        super().enter()

    def transition(self):
        next_state = super().transition()
        if next_state is not None:
            return next_state
        return self.return_to_state


class PullBlockCloser(SlideBlock):
    """ Pulls a block closer that seems to be somewhat out of reach.

    Implement the violation resolver interface by implementing find_and_record_violations(). That
    method will be called before enter() to population a self.violations list member.
    """

    def __init__(self, domain, max_radius, push_standoff_alpha, push_depth, push_time, return_to_state=None):
        self.max_radius = max_radius
        self.return_to_state = return_to_state

        self.violations = []

        super().__init__(
            domain=domain, push_standoff_alpha=push_standoff_alpha, push_depth=push_depth, push_time=push_time
        )

    def find_and_record_violations(self):
        """ Finds and records the current violations. This method is called before each transition
        in, so the violations will be available on entry.
        """
        self.violations = []
        for i, (T, _) in enumerate(self.domain.azure_position_transforms()):
            p = T[:3, 3]
            if np.linalg.norm(p) > self.max_radius:
                self.violations.append(p)
        return len(self.violations) > 0

    def enter(self):
        """ Choses a random violation to resolve on entry.
        """

        set_gripper_to_push_width(self.domain.franka)

        print("reach violations:", self.violations)
        p = self.violations[np.random.randint(0, len(self.violations))]
        push_direction = math_utils.normalized(-p)

        # If the push direction is zero, choose a random direction.
        while np.linalg.norm(push_direction) == 0.0:
            push_direction = np.random.randn(3)
        push_direction[2] = 0.0  # Ensure horizontal
        push_direction = math_utils.normalized(push_direction)

        push_dist = min(0.1, np.linalg.norm(p) - self.max_radius + 0.05)
        self.setup(p, push_direction, push_dist)
        super().enter()

    def transition(self):
        next_state = super().transition()
        if next_state is not None:
            return next_state
        return self.return_to_state


class Dispatch(State):
    """ Dispatch to a random violation resolution state with violations.

    Violation resolvers should implement the violation resolvers interface by implementing
    find_and_record_violations(). Those methods are used to find violations during the transition()
    call to dispatch to a random resolution state with active violations.
    """

    def __init__(self, domain, never_exit=False):
        self.domain = domain
        self.never_exit = never_exit

        self.callbacks = []
        self.violation_resolvers = []

    def add_violation_resolver(self, violation_resolver):
        self.violation_resolvers.append(violation_resolver)

    def enter(self):
        for cb in self.callbacks:
            cb()

    def transition(self):
        possible_next_states = [
            resolver for resolver in self.violation_resolvers if resolver.find_and_record_violations()
        ]
        print("Dispatch -- num next states:", len(possible_next_states))
        if len(possible_next_states) > 0:
            return possible_next_states[np.random.randint(0, len(possible_next_states))]

        if self.never_exit:
            return self
        else:
            return None


class Scanning(HierarchicalState):
    """ Scan the blocks by cycling choosing a random block and looking at it three times.
    """

    def __init__(self, domain, dispatch_state):
        self.domain = domain
        self.dispatch_state = dispatch_state

        look_at3 = LookAt(domain, look_at_commander_tag="short", block_index="random", look_time=0.0, next_state=None)
        look_at2 = LookAt(
            domain, look_at_commander_tag="short", block_index="random", look_time=0.0, next_state=look_at3
        )
        look_at1 = LookAt(
            domain, look_at_commander_tag="short", block_index="random", look_time=0.0, next_state=look_at2
        )

        super().__init__(look_at1)

    def enter(self):
        self.domain.franka.end_effector.gripper.move(width=0.0, speed=0.2, wait=False)
        super().enter()

    def transition(self):
        if self.active_state is None:
            return self.dispatch_state
        return self


class PickBlockApproach(ApproachTarget):
    """ Approach behavior for the pick block composite behavior.

    Uses a PickInfo object to define the parameters of the pick. block_index is suppled on
    initialization to enable suppression of the block.

    The step() method will block during the gripper closer phase.

    Postcondition of this behavior is that the gripper_block_state logical state entry becomes the
    block_index passed in, specifying that that block is successfully in the gripper.
    """

    def __init__(self, domain, pick_info, block_index, use_full_orientation_constraint=True):
        self.pick_info = pick_info
        self.block_index = block_index
        self.use_full_orientation_constraint = use_full_orientation_constraint

        self.close_force = domain.close_force
        self.termination_criteria = FrameTerminationCriteria(orig_thresh=0.001)

        super().__init__(domain=domain, use_default_config=True, next_state=None)

    def enter(self):
        print("Pick block approach")
        self.domain.franka.end_effector.gripper.open(wait=False)
        target = {}

        # Projected version -- TODO: move this to the calling point
        axis_y = math_utils.normalized(self.pick_info.target_axis_y)
        axis_y[2] = 0.0

        if self.pick_info.target_axis_z is not None:
            axis_z = self.pick_info.target_axis_z
        else:
            axis_z = np.array([0.0, 0.0, -1.0])

        axis_z = math_utils.normalized(math_utils.proj_orth(axis_z, axis_y))

        axis_x = np.cross(axis_y, axis_z)

        # Be robust to ill-normalized transforms.

        if self.use_full_orientation_constraint:
            target = {"orig": self.pick_info.target_orig, "axis_x": axis_x, "axis_y": axis_y, "axis_z": axis_z}
        else:
            target = {"orig": self.pick_info.target_orig, "axis_y": axis_y}

        bad_tags = [tag for tag in target if target[tag] is None]
        for tag in bad_tags:
            del target[tag]

        approach = ApproachParams(direction=self.pick_info.approach_direction, standoff=0.07, standoff_std_dev=0.001)

        self.set_target(target=target, approach=approach, rotation_blend_std_dev=0.05)
        self.is_suppressed = False
        self.is_finished = False

    def step(self):
        """ Returns quickly throughout the approach motion, and suppresses both the table collisions
        and the specified block_index collisions when within .1 meters of the block. However, once
        it has arrived at the target it closes the gripper, and during that gripper close operation,
        this step() method will block effectively pausing the state machine to prevent preemption.
        """
        super().step()

        current_orig = self.domain.franka.end_effector.status.orig
        if not self.is_suppressed and np.linalg.norm(self.pick_info.target_orig - current_orig) <= 0.1:
            self.domain.blocks_world_suppressors.table_suppressor.suppress()
            self.domain.blocks_world_suppressors.block_suppressors[self.block_index].suppress()
            self.is_suppressed = True

        if not self.is_finished and self.has_arrived():
            self.domain.franka.end_effector.gripper.close(force=self.close_force)
            self.domain.logical_state.gripper_block_state = self.block_index
            self.is_finished = True

    def transition(self):
        if self.is_finished:
            return self.next_state
        else:
            return self


class PickBlockLift(State):
    """ Lift behavior for the pick block composite behavior. Lifts slightly after the pick.

    Uses a PickInfo object to define the parameters of the pick. block_index is suppled on
    initialization to enable suppression of the block.
    """

    def __init__(self, domain, pick_info, max_wait_time=0.25):
        self.domain = domain
        self.pick_info = pick_info
        self.max_wait_time = max_wait_time
        self.termination_criteria = FrameTerminationCriteria(orig_thresh=0.001)
        self.next_state = None

    def enter(self):
        self.target = {"orig": self.pick_info.target_orig + np.array([0.0, 0.0, 0.025])}
        self.domain.franka.end_effector.go_local(target=self.target, use_default_config=True, wait_for_target=False)
        self.domain.blocks_world_suppressors.table_suppressor.unsuppress()
        self.start_time = self.domain.time

    def has_arrived(self):
        return self.termination_criteria(self.target, self.domain.current_end_effector_frame())

    def transition(self):
        if self.has_arrived() or (self.domain.time - self.start_time) >= self.max_wait_time:
            return self.next_state
        else:
            return self


class PickBlock(HierarchicalState):
    """ Full hierarchical pick block behavior.

    PickBlockApproach --> PickBlockLift

    Uses a PickInfo object to define the parameters of the pick. block_index is suppled on
    initialization to enable suppression of the block.
    """

    def __init__(self, domain, pick_info, block_index, use_full_orientation_constraint=True):
        self.domain = domain
        self.pick_info = pick_info
        self.pick_block_approach = PickBlockApproach(domain, pick_info, block_index, use_full_orientation_constraint)
        self.pick_block_lift = PickBlockLift(domain, pick_info)

        self.pick_block_approach.next_state = self.pick_block_lift

        self.next_state = None
        super().__init__(self.pick_block_approach)

    def update_pick_info(self, pick_info):
        self.pick_info.update(pick_info)

    def transition(self):
        next_state = super().transition()
        if next_state is not None:
            return next_state
        return self.next_state


class PlaceBlockCore(State):
    """ Core underlying behavior state for the place block behavior.

    Performs suppression of everything at the appropriate time just before the place occures.
    step() blocks during the gripper release phase to prevent preemption.

    Postcondition of this behavior is that the gripper_block_state logical state entry becomes None.
    """

    def __init__(self, domain, block_place_orig, target_axis_y=np.array([0.0, 1.0, 0.0])):
        """ block_place_orig should be the center of the block.
        """
        self.domain = domain
        self.target_orig = copy.deepcopy(block_place_orig)
        self.target_orig[2] += -self.domain.block_height / 2 + self.domain.grasp_height
        self.target_axis_y = target_axis_y
        self.target_axis_z = np.array([0.0, 0.0, -1.0])

        self.next_state = None

    def enter(self):
        # print("Sending y-axis to: ", self.target_axis_y)
        self.domain.franka.end_effector.go_local(
            orig=self.target_orig,
            axis_y=self.target_axis_y,
            axis_z=self.target_axis_z,
            approach_direction=self.domain.down,
            approach_standoff=0.07,
            approach_standoff_std_dev=0.001,
            use_default_config=True,
            wait_for_target=False,
        )
        self.is_suppressed = False
        self.is_done = False

    def step(self):
        orig = self.domain.franka.end_effector.status.orig
        axis_y = self.domain.franka.end_effector.status.axis_y
        axis_z = self.domain.franka.end_effector.status.axis_z

        orig_err = np.linalg.norm(self.target_orig - orig)
        axis_y_err = np.linalg.norm(self.target_axis_y - axis_y)
        axis_z_err = np.linalg.norm(self.target_axis_z - axis_z)

        suppress_thresh = 0.03
        if not self.is_suppressed and orig_err <= suppress_thresh:
            self.domain.blocks_world_suppressors.suppress_everything()
            self.is_suppressed = True

        # print("orig_err: %f, axis_y_err: %f, axis_z_err: %f" % (orig_err, axis_y_err, axis_z_err))
        if orig_err <= 0.0025 and axis_y_err <= 0.01 and axis_z_err <= 0.01:
            # This call will block until the gripper is open
            print("Opening gripper")
            self.domain.franka.end_effector.gripper.open(wait=True)
            self.domain.logical_state.gripper_block_state = None  # Clear the gripper block state
            self.is_done = True

    def transition(self):
        if self.is_done:
            return self.next_state
        else:
            return self


class PlaceBlockLift(State):
    """ A simple lift state to lift slightly after releasing the block.

    Unsuppresses all collision controllers on entry before sending the go_local so unsuppression and
    go_local (moving up) are working in conjunction.
    """

    def __init__(self, domain):
        self.domain = domain
        self.termination_criteria = FrameTerminationCriteria(orig_thresh=0.02)
        self.next_state = None

    def enter(self):
        target_orig = self.domain.franka.end_effector.status.orig + 0.1 * self.domain.up
        self.target = {"orig": target_orig}
        self.domain.franka.end_effector.go_local(
            target=self.target,
            approach_direction=self.domain.up,
            approach_standoff=0.07,
            approach_standoff_std_dev=0.01,
            use_default_config=True,
            wait_for_target=False,
        )

    def has_arrived(self):
        return self.termination_criteria(self.target, self.domain.current_end_effector_frame())

    def step(self):
        pass

    def exit(self):
        self.domain.blocks_world_suppressors.unsuppress_everything()

    def transition(self):
        if self.has_arrived():
            return self.next_state
        else:
            return self


class PlaceBlock(HierarchicalState):
    """ Full block placement hierarchical state.

    PlaceBlockCore --> PlaceBlockLift

    These internal states handle suppression and unsuppression of environmental collisions at
    appropriate times.
    """

    def __init__(self, domain, block_place_orig, target_axis_y):
        self.domain = domain

        self.place_block_core = PlaceBlockCore(self.domain, block_place_orig, target_axis_y)
        self.place_block_lift = PlaceBlockLift(self.domain)

        self.place_block_core.next_state = self.place_block_lift

        super().__init__(self.place_block_core)


class UpdatePickInfo(State):
    def __init__(self, domain, block_index, pick_state, next_state):
        self.domain = domain
        self.block_index = block_index
        self.pick_state = pick_state
        self.next_state = next_state

    def enter(self):
        print("Updating block info for pick.")
        pick_info = MakeNaturalPickInfo(self.domain, self.block_index)
        self.pick_state.update_pick_info(pick_info)

    def transition(self):
        return self.next_state


class VerifyPick(State):
    def __init__(self, domain, block_index, check_condition, success_state, failure_state):
        self.domain = domain
        self.block_index = block_index
        self.check_condition = check_condition
        self.success_state = success_state
        self.failure_state = failure_state

    def enter(self):
        self.is_success = None

    def transition(self):
        self.is_success = self.check_condition()
        if self.is_success:
            return self.success_state
        else:
            return self.failure_state

    def exit(self):
        pass


class PickAndPlaceMachine(HierarchicalState):
    """ Complete pick and place behavior sequencing a pick and a place.

    PickBlock --> PlaceBlock

    These sub state machines handle suppressing and unsuppressing environmental collisions at
    appropriate times. block_index is supplied on construction to enable the pick behavior to
    suppress just the block in question.
    """

    def __init__(
        self,
        domain,
        pick_info,
        block_place_orig,
        target_axis_y,
        block_index,
        use_full_orientation_constraint_on_pick=True,
    ):
        self.domain = domain

        look_at_block = LookAt(domain, look_at_commander_tag="short", block_index=block_index, look_time=3.0)
        self.look_at_block_behavior = Behavior(look_at_block)

        self.pick_block = PickBlock(domain, pick_info, block_index, use_full_orientation_constraint_on_pick)
        self.place_block = PlaceBlock(domain, block_place_orig, target_axis_y)

        self.update_pick_info = UpdatePickInfo(
            domain=domain, block_index=block_index, pick_state=self.pick_block, next_state=self.pick_block
        )

        self.verify_pick = VerifyPick(
            domain=domain,
            block_index=block_index,
            check_condition=lambda: not self.domain.gripper_closed(),
            success_state=self.place_block,
            failure_state=self.look_at_block_behavior,
        )

        self.look_at_block_behavior.terminal_transition = NextStateTransition(self.update_pick_info)
        self.pick_block.next_state = self.verify_pick
        super().__init__(self.pick_block)


class HideGhost(State):
    """ Hides Ghost"""

    def __init__(self, domain):
        self.domain = domain
        self.next_state = None

    def enter(self):
        self.domain.franka.target_visibility = False

    def has_arrived(self):
        return True

    def step(self):
        pass

    def exit(self):
        pass

    def transition(self):
        if self.has_arrived():
            return self.next_state
        else:
            return self


class UnhideGhost(State):
    """ Unhides Ghost"""

    def __init__(self, domain):
        self.domain = domain
        self.next_state = None

    def enter(self):
        self.domain.franka.target_visibility = True

    def has_arrived(self):
        return True

    def step(self):
        pass

    def exit(self):
        pass

    def transition(self):
        if self.has_arrived():
            return self.next_state
        else:
            return self


class GhostReset(ApproachTarget):
    """Moves Ghost to the current end effector position"""

    def __init__(self, domain, main_domain):
        self.domain = domain
        self.main_domain = main_domain

        self.pick_info = None
        self.block_index = None
        self.use_full_orientation_constraint = True

        self.close_force = self.domain.close_force
        self.termination_criteria = FrameTerminationCriteria(orig_thresh=0.5)
        self.target = self.main_domain.franka.end_effector.status.current_frame
        self.rotation_blend_std_dev = 0.05
        self.use_default_config = False
        self.next_state = None
        self.approach = ApproachParams(direction=np.array([0.0, 0.0, -1.0]), standoff=0.02, standoff_std_dev=0.1)
        self.reset_time = 2

    def setup(self):
        print("setup Ghost Reset")
        self.target = self.main_domain.franka.end_effector.status.current_target
        self.set_target(self.target, self.approach, termination_criteria=self.termination_criteria)

    def enter(self):
        print("enter Ghost Reset")
        self.target = self.main_domain.franka.end_effector.status.current_target
        self.target["orig"] = np.array([0.4, 0, 0.4])
        self.set_target(self.target, self.approach, termination_criteria=self.termination_criteria)
        self.start_time = self.domain.time

    def transition(self):
        if self.reset_time is not None and (self.domain.time - self.start_time) > self.reset_time:
            return self.next_state
        if self.has_arrived():
            return self.next_state
        return self


class PickAndPlaceGhostMachine(HierarchicalState):
    """ Complete pick and place behavior sequencing a pick and a place.

    PickBlock --> PlaceBlock

    These sub state machines handle suppressing and unsuppressing environmental collisions at
    appropriate times. block_index is supplied on construction to enable the pick behavior to
    suppress just the block in question.
    """

    def __init__(
        self,
        domain,
        pick_info,
        block_place_orig,
        target_axis_y,
        block_index,
        use_full_orientation_constraint_on_pick=True,
    ):
        self.domain = domain
        think_machines = []
        for ghost_domain in self.domain.ghost_domains:

            unhide_robot = Behavior(UnhideGhost(ghost_domain))
            pick_block = Behavior(
                PickBlockApproach(
                    ghost_domain, copy.deepcopy(pick_info), block_index, use_full_orientation_constraint_on_pick
                )
            )
            unhide_robot.terminal_transition = NextStateTransition(pick_block)

            think_machines.append(unhide_robot)

        look_at_block = LookAt(domain, look_at_commander_tag="short", block_index=block_index, look_time=3.0)
        self.look_at_block_behavior = Behavior(look_at_block)
        run_machine = PickBlock(domain, pick_info, block_index, use_full_orientation_constraint_on_pick)
        if len(self.domain.ghost_domains) > 0:
            self.pick_block = Behavior(ThinkAndRun(think_machines, run_machine, 0.5))
        else:
            self.pick_block = Behavior(run_machine)
        self.place_block = PlaceBlock(domain, block_place_orig, target_axis_y)

        self.update_pick_info = UpdatePickInfo(
            domain=domain, block_index=block_index, pick_state=self.pick_block, next_state=self.pick_block
        )

        hide_machines = []
        for ghost_domain in self.domain.ghost_domains:
            hide_robots = HideGhost(ghost_domain)
            hide_machines.append(hide_robots)

        self.hide_robots = Behavior(MultiHierarchicalState(hide_machines))

        self.verify_pick = VerifyPick(
            domain=domain,
            block_index=block_index,
            check_condition=lambda: not self.domain.gripper_closed(),
            success_state=self.hide_robots,
            failure_state=self.look_at_block_behavior,
        )

        self.look_at_block_behavior.terminal_transition = NextStateTransition(self.update_pick_info)
        self.pick_block.terminal_transition = NextStateTransition(self.verify_pick)
        self.hide_robots.terminal_transition = NextStateTransition(self.place_block)

        super().__init__(self.pick_block)


class PickAndPlace(HierarchicalState):
    def __init__(
        self,
        domain,
        block_index,
        block_place_orig,
        target_axis_y=np.array([0.0, 1.0, 0.0]),
        use_full_orientation_constraint_on_pick=True,
    ):
        self.domain = domain
        self.block_index = block_index
        self.block_place_orig = block_place_orig
        self.target_axis_y = target_axis_y
        self.use_full_orientation_constraint_on_pick = use_full_orientation_constraint_on_pick

    def set_recovery_state(self, state):
        self.pick_and_place.recovery_state = state

    def setup(self):
        pick_info = MakeNaturalPickInfo(self.domain, self.block_index)
        pick_and_place = PickAndPlaceGhostMachine(
            self.domain,
            pick_info,
            self.block_place_orig,
            self.target_axis_y,
            self.block_index,
            self.use_full_orientation_constraint_on_pick,
        )
        self.pick_and_place = pick_and_place

        super().__init__(init_state=pick_and_place)

    def enter(self):
        self.setup()
        super().enter()

    def exit(self):
        if self.active_state is None:
            T = np.eye(4)
            T[0:3, 3] = self.block_place_orig


class Freeze(State):
    def __init__(self, domain, movement_time=None):
        self.domain = domain
        self.movement_time = movement_time

    def enter(self):
        print("freeze: enter")
        self.domain.franka.end_effector.freeze()
        if self.movement_time is not None:
            x = self.domain.franka.end_effector.status.orig
            xd = self.domain.franka.end_effector.status.orig_vel
            dt = self.movement_time
            x_target = x + dt * xd

            self.domain.franka.end_effector.go_local(orig=x_target, use_default_config=False, wait_for_target=False)

    def transition(self):
        # Always immediately transition out
        print("freeze: transition away")
        return None


class WorldSuppressedBehavior(Behavior):
    """ A Behavior that suppresses all collisions on entry and unsuppresses them all on exit.
    """

    def __init__(self, domain, behavior_machine, terminal_transition=None):
        self.domain = domain
        super().__init__(behavior_machine, terminal_transition)

    def enter(self):
        self.domain.blocks_world_suppressors.suppress_everything()
        super().enter()

    def exit(self):
        self.domain.blocks_world_suppressors.unsuppress_everything()
        super().exit()


class WorldUnsuppressedBehavior(Behavior):
    """ A Behavior that unsuppresses all collisions on entry and exit.
    """

    def __init__(self, domain, behavior_machine, terminal_transition=None):
        self.domain = domain
        super().__init__(behavior_machine, terminal_transition)

    def enter(self):
        self.domain.blocks_world_suppressors.unsuppress_everything()
        super().enter()

    def exit(self):
        self.domain.blocks_world_suppressors.unsuppress_everything()
        super().exit()


class ResolveViolations(HierarchicalState):
    def __init__(
        self,
        domain,
        required_radius=0.175,
        required_separation=0.15,
        max_radius=0.75,
        push_standoff_alpha=0.5,
        push_depth=0.00,
        push_time=1.0,
        start_from_dispatch_without_suppression=False,
        never_exit=False,
    ):
        self.domain = domain
        self.never_exit = never_exit

        dispatch = Dispatch(domain, never_exit=never_exit)
        scanning = Scanning(domain, dispatch)

        push_away_from_tower = PushAwayFromTower(
            domain, required_radius, push_standoff_alpha, push_depth, push_time, return_to_state=scanning
        )
        dispatch.add_violation_resolver(push_away_from_tower)
        dispatch.callbacks.append(self.domain.send_block_locations_to_backend)

        push_blocks_apart = PushBlocksApart(
            domain, required_separation, push_standoff_alpha, push_depth, push_time, return_to_state=scanning
        )
        dispatch.add_violation_resolver(push_blocks_apart)

        pull_block_closer = PullBlockCloser(
            domain, max_radius, push_standoff_alpha, push_depth, push_time, return_to_state=scanning
        )
        dispatch.add_violation_resolver(pull_block_closer)

        # Store here to see if we have any violations
        self.dispatch = dispatch
        self.scanning = scanning

        if start_from_dispatch_without_suppression:
            violation_resolution_behavior = dispatch
        else:
            violation_resolution_behavior = WorldSuppressedBehavior(domain, scanning, terminal_transition=None)
        super().__init__(init_state=violation_resolution_behavior)

    def enter(self):
        self.domain.franka.set_speed(speed_level="fast")
        super().enter()

    def step(self):
        super().step()


class StackBlocksEconomical(HierarchicalState):
    """ Full block stacking behavior stacking blocks in the order of the colors given in
    domain.block_colors.

    Runs pick-and-place, and a second on-tower pinch alignment.
    """

    def __init__(self, domain):
        self.domain = domain
        init_state = latest_behavior = Behavior()
        for block_index, color in enumerate(domain.block_colors):
            look_at_block = LookAt(
                domain,
                look_at_commander_tag="short",
                update_continuously=True,
                block_index=block_index,
                look_time=300.0,
            )

            reset_robots = [Behavior(look_at_block)]
            for ghost_domain in domain.ghost_domains:
                reset_robots.append(Behavior(GhostReset(ghost_domain, domain)))

            look_at_block_behavior = Behavior(MultiHierarchicalState(reset_robots))

            block_tower_place_orig = np.array(
                [domain.goal_x, domain.goal_y, domain.block_height / 2 + block_index * domain.block_height]
            )
            tower_pick_and_place_behavior = Behavior(PickAndPlace(domain, block_index, block_tower_place_orig))
            second_pinch_align_behavior = WorldSuppressedBehavior(
                domain, PinchAlign(domain, block_index, suppress_block=False)
            )

            latest_behavior.terminal_transition = NextStateTransition(look_at_block_behavior)
            look_at_block_behavior.terminal_transition = NextStateTransition(tower_pick_and_place_behavior)
            tower_pick_and_place_behavior.terminal_transition = NextStateTransition(second_pinch_align_behavior)
            latest_behavior = second_pinch_align_behavior

        super().__init__(init_state)
