# Copyright (c) 2018-2020, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import math
import carb

from pxr import UsdGeom, Gf
from omni.isaac.core.utils.rotations import quat_to_euler_angles
from omni.isaac.dynamic_control import _dynamic_control
import numpy as np
from omni.debugdraw import _debugDraw
from .quintic_path_planner import QuinticPolynomial, quintic_polynomials_planner
from .stanley_control import State, pid_control, stanley_control, normalize_angle, Kp, calc_target_index


def calc_speed_profile(cyaw, max_speed, target_speed, min_speed=1):
    speed_profile = np.array(cyaw) / max([abs(c) for c in cyaw]) * max_speed

    # direction = 1.0

    # # Set stop point
    # for i in range(len(cyaw) - 1):
    #     dyaw = abs(cyaw[i + 1] - cyaw[i])
    #     switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

    #     if switch:
    #         direction *= -1

    #     if direction != 1.0:
    #         speed_profile[i] = -target_speed
    #     else:
    #         speed_profile[i] = target_speed

    #     if switch:
    #         speed_profile[i] = 0.0

    # speed down
    res = min(int(len(cyaw) / 3), int(max_speed * 60))

    # # print("slow", slow, len(cyaw))
    for i in range(1, res):
        speed_profile[-i] = min(speed_profile[-i], speed_profile[-i] / (float(res - i)) ** 0.5)  # / (res))
        if speed_profile[-i] <= min_speed:
            speed_profile[-i] = min_speed

    return speed_profile


class RobotController:
    def __init__(
        self,
        stage,
        dc,
        articulation_path,
        odom_prim_path,
        wheel_joint_names,
        wheel_speed,
        goal_offset_threshold,
        wheel_base,
        wheel_radius,
    ):
        self._stage = stage
        self._stage_unit = UsdGeom.GetStageMetersPerUnit(self._stage)
        self._dc = dc
        self._articulation_path = articulation_path
        self._odom_prim_path = odom_prim_path
        self._wheel_joint_names = wheel_joint_names
        self._wheel_speed = wheel_speed
        self._goal_offset_threshold = goal_offset_threshold
        self._reached_goal = [True, True]
        self._enable_navigation = False
        self._goal = [400, 400, 0]
        self._go_forward = False
        self._wheel_base = wheel_base
        self._wheel_radius = wheel_radius
        self.state = State(wheel_base, x=0, y=0, yaw=0, v=0)
        self.target_idx = 0
        self._debugDraw = _debugDraw.acquire_debug_draw_interface()
        self.cx = []

    def _get_odom_data(self):
        self.imu = self._dc.get_rigid_body(self._odom_prim_path)
        imu_pose = self._dc.get_rigid_body_pose(self.imu)
        roll, pitch, yaw = quat_to_euler_angles(np.array([imu_pose.r.w, imu_pose.r.x, imu_pose.r.y, imu_pose.r.z]))
        # print(roll, pitch, yaw)
        self.current_robot_translation = [imu_pose.p.x, imu_pose.p.y, imu_pose.p.z]
        self.current_robot_translation = [i * self._stage_unit for i in self.current_robot_translation]
        self.current_robot_orientation = [normalize_angle(roll), normalize_angle(pitch), normalize_angle(yaw)]
        self.current_speed = self._dc.get_rigid_body_local_linear_velocity(self.imu).x * self._stage_unit

    def reached_goal(self):
        return self._reached_goal[0] and self._reached_goal[1]

    def get_goal(self):
        return self._goal

    def draw_path(self, step=None):
        if self._enable_navigation and not self._reached_goal[0]:
            for i in range(len(self.cx) - 1):
                # self._debugDraw.draw_line(
                #     carb.Float3(self.cx[i], self.cy[i], 14),
                #     self.y_color,
                #     carb.Float3(self.cx[i] + 20 * math.cos(self.cyaw[i]), self.cy[i] + 20 * math.sin(self.cyaw[i]), 14),
                #     self.y_color,
                # )
                self._debugDraw.draw_line(
                    carb.Float3(self.cx[i] / self._stage_unit, self.cy[i] / self._stage_unit, 0.14 / self._stage_unit),
                    self.argb[i],
                    carb.Float3(
                        self.cx[i + 1] / self._stage_unit, self.cy[i + 1] / self._stage_unit, 0.14 / self._stage_unit
                    ),
                    self.argb[i - 1],
                )

    def update(self, step):
        v = 0
        w = 0
        if self._enable_navigation:
            self._get_odom_data()
            # self.draw_path()
            theta = self.current_robot_orientation[2]
            theta_goal = self._goal[2]
            theta_diff = math.atan2(math.sin(theta_goal - theta), math.cos(theta_goal - theta))
            x_diff = float(self._goal[0]) - self.current_robot_translation[0]
            y_diff = float(self._goal[1]) - self.current_robot_translation[1]
            rho = np.hypot(x_diff, y_diff)
            self.state = State(
                self._wheel_base * Kp,
                x=self.current_robot_translation[0],
                y=self.current_robot_translation[1],
                yaw=self.current_robot_orientation[2] % (2 * np.pi),
                v=self.current_speed,
            )
            self._reached_goal = [
                rho < self._goal_offset_threshold[0] or self.rotate_only and rho < self._goal_offset_threshold[0] * 5,
                abs(theta_diff) <= self._goal_offset_threshold[1],
            ]
            if self._reached_goal[0] and self._reached_goal[1]:
                self.control_command(0, 0)
                self._enable_navigation = False
                return
            if not self.rotate_only:
                ai = pid_control(self.sp[self.target_idx], self.state.v) / step
                di, self.target_idx = stanley_control(self.state, self.cx, self.cy, self.cyaw, self.target_idx)

                self.state.update(ai, di, step)
                v = self.state.v
                w = self.state.w

            if self._reached_goal[0] or self.rotate_only:
                if self._reached_goal[0]:
                    self.rotate_only = True
                v = 0
                if theta_diff > 0:
                    w = min(((theta_diff) * Kp / step), 1)
                else:
                    w = max(((theta_diff) * Kp / step), -1)
            # print(rho, abs(theta_diff), v, w, self.sp[self.target_idx], self.current_speed, self.target_idx)

            kw = 0.5
            # Allow additional steering to use differential drive (backwards spin on one wheel to tighten the cornering radius)
            if not self._reached_goal[0] and v > 0:
                kw = 0.5 + abs((self._wheel_base * w) / v) * (0.5 * Kp / step)
                # print(kw)
            command_left = (v - kw * self._wheel_base * w) / self._wheel_radius
            command_right = (v + kw * self._wheel_base * w) / self._wheel_radius
            # print(command_left, command_right)
            self.control_command(command_left, command_right)

    def control_setup(self):
        self.ar = self._dc.get_articulation(self._articulation_path)
        self.wheel_left = self._dc.find_articulation_dof(self.ar, self._wheel_joint_names[0])
        self.wheel_right = self._dc.find_articulation_dof(self.ar, self._wheel_joint_names[1])

        self.vel_props = _dynamic_control.DofProperties()
        # self.vel_props.drive_mode = _dynamic_control.DRIVE_VEL
        self.vel_props.damping = 1e7
        self.vel_props.stiffness = 0
        self._dc.set_dof_properties(self.wheel_left, self.vel_props)
        self._dc.set_dof_properties(self.wheel_right, self.vel_props)

    def control_command(self, left_wheel_speed, right_wheel_speed):
        # Wake up articulation every move command to ensure commands are applied
        self._dc.wake_up_articulation(self.ar)
        # Normalizes both wheels speed if any speed will be clipped
        if abs(left_wheel_speed) > self._wheel_speed[0]:
            factor = abs(self._wheel_speed[0] / left_wheel_speed)
            right_wheel_speed = right_wheel_speed * factor
            left_wheel_speed = left_wheel_speed * factor
        if abs(right_wheel_speed) > self._wheel_speed[1]:
            factor = abs(self._wheel_speed[1] / right_wheel_speed)
            left_wheel_speed = left_wheel_speed * factor
            right_wheel_speed = right_wheel_speed * factor

        self._dc.set_dof_velocity_target(self.wheel_left, left_wheel_speed)
        self._dc.set_dof_velocity_target(self.wheel_right, right_wheel_speed)

    def set_goal(self, x, y, theta, sv=0.5, sa=0.05, gv=0.5, ga=0.05, max_speed=2.0):
        theta = normalize_angle(theta)
        self._goal = [x, y, theta]

        max_accel = 1.5  # max accel [m/ss]
        max_jerk = 0.3  # max jerk [m/sss]
        self._get_odom_data()
        self.target_idx = 0
        x_diff = self.current_robot_translation[0] - x
        y_diff = self.current_robot_translation[1] - y
        rho = np.hypot(x_diff, y_diff)
        if rho / 5.0 > self._goal_offset_threshold[0]:
            self.tt, self.cx, self.cy, self.cyaw, self.ck, s, j = quintic_polynomials_planner(
                self.current_robot_translation[0],
                self.current_robot_translation[1],
                self.current_robot_orientation[2],
                sv,
                sa,
                x,
                y,
                theta,
                gv,
                ga,
                max_accel,
                max_jerk,
                1 / 60.0,
            )
            self.cx = np.array(self.cx)
            self.cy = np.array(self.cy)
            self.sp = calc_speed_profile(np.array(self.ck), max_speed, 0.5, 0.05)
            color = [(0, t / np.max(self.sp), 0) for t in self.sp]
            self.y_color = int.from_bytes(b"\xff\xff\x00\x00", byteorder="big")
            rgb_bytes = [(np.clip(c, 0, 1.0) * 255).astype("uint8").tobytes() for c in color]
            argb_bytes = [b"\xff" + b for b in rgb_bytes]
            self.argb = [int.from_bytes(b, byteorder="big") for b in argb_bytes]
            self.rotate_only = False
        else:
            self.rotate_only = True

    def enable_navigation(self, flag):
        self._enable_navigation = flag
