# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import numpy as np
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.demos.utils import math_utils


class Object:
    """Definition of an object in the world
    """

    def __init__(self, mp, rmp_handle, handle, asset_path, name):
        """Initialize object
        
        Args:
            mp (_motion_planning): Handle to motion planning extension
            rmp_handle ([type]): Handle to motion generatior for the robot associated with this object
            handle (_dynamic_control.Handle): Dynamic control handle to get physics information
            asset_path (string): USD path the asset
            name (string): Name for object
        """
        self.mp = mp
        self.rmp_handle = rmp_handle
        self.obstacle = False
        self.suppressed = False

        self.handle = handle
        self.pose_T = np.identity(4)
        self.pose = _dynamic_control.Transform()
        self.asset_path = asset_path
        self.name = name

    def update(self, pose):
        """update pose of this object
        
        Args:
            pose (_dynamic_control.Transform()): 6dof transform
        """
        self.pose = pose

    def suppress(self):
        """Disable this object as an obstacle for the given RMP handle
        """
        if self.obstacle and not self.suppressed:
            self.mp.disableObstacle(self.rmp_handle, self.asset_path)
            self.suppressed = True

    def unsuppress(self):
        """Enable this object as an obstacle for the given RMP handle
        """
        if self.obstacle and self.suppressed:
            self.mp.enableObstacle(self.rmp_handle, self.asset_path)
            self.suppressed = False

    def makeObstacle(self, obstacle_type, scale):
        """Make this object an obstacle
        
        Args:
            obstacle_type (int): 1: cylinder, 2: sphere, 3: cube
            scale (float3): cylinder: [radius, radius, height], sphere: [radius, radius, radius], cube: [length, width, height]
        """
        # print("Adding obstacle from path:", self.asset_path)
        self.mp.addObstacle(self.rmp_handle, self.asset_path, obstacle_type, scale)
        self.obstacle = True


class World:
    """World contains objects that the robot will interact with and avoid
    """

    def __init__(self, dc, mp):
        """Initializa world
        
        Args:
            dc (_dynamic_control): handle to dynamic control extension
            mp (_motion_planning): handle to motion planning extension
        """
        self.dc = dc
        self.mp = mp
        self.rmp_handle = None
        self.tracked_objects = dict()
        self.tracked_objects_map = dict()
        self.parent = None
        self.handles = []

    def get_T(self, name):
        """Get 4x4 homogeneous transform matrix for an object
        
        Args:
            name (string): name of object
        
        Returns:
            matrix: 4x4 homogeneous transform matrix
        """
        return math_utils.as_np_matrix_t(self.tracked_objects_map[name].pose)

    def register_parent(self, handle, path, name):
        """Register parent for the world that relative transforms will be computed against
        
        Args:
            handle (_dynamic_control.Handle): Dynamic control handle to get physics information
            path (string): USD path for parent
            name (string): name for parent object
        """
        self.parent = Object(self.mp, self.rmp_handle, handle, path, name)

    def register_object(self, handle, path, name):
        """Register a new object in this world
        
        Args:
            handle (_dynamic_control.Handle): Dynamic control handle to get physics information
            path (string): USD path for parent
            name (string): name for parent object
        """
        obj = Object(self.mp, self.rmp_handle, handle, path, name)
        obj.asset_path = path
        obj.name = name
        self.tracked_objects_map[name] = obj
        self.handles.append((handle, obj.asset_path))

    def make_obstacle(self, name, obstacle_type, scale):
        """Make this object into an obstacle
        
        Args:
            name (string): name of object to make an obstacle
            obstacle_type (int): object type
            scale (float3): scale for given object type
        """
        self.tracked_objects_map[name].makeObstacle(obstacle_type, scale)

    def get_object_handle_from_name(self, name):
        """Given the name of the object, returns handle
        
        Args:
            name (string): name of object
        
        Returns:
            _dynamic_control.Handle: Dynamic control handle to get physics information
        """
        return self.tracked_objects_map[name].handle

    def get_object_from_name(self, name):
        """Given the name of the object, returns the object
        
        Args:
            name (string): name of object
        
        Returns:
            Object: reference to object
        """
        return self.tracked_objects_map[name]

    def update(self):
        """Update relative pose of all objects in this world
        """
        poses = self.mp.updateGetRelativePoses(self.rmp_handle, self.handles)
        for index, (name, obj) in enumerate(self.tracked_objects_map.items()):
            obj.update(poses[index])

    def reset(self, data):
        """Reset objects to a given pose
        
        Args:
            data (list of (string, float3)): List of tuples containing an object name and a pose
        """
        for name, pos in data:
            handle = self.get_object_handle_from_name(name)
            pose = self.dc.get_rigid_body_pose(self.parent.handle).p
            self.dc.set_rigid_body_pose(
                handle, _dynamic_control.Transform((pos[0] + pose[0], pos[1] + pose[1], pos[2] + pose[2]), (0, 0, 0, 1))
            )
