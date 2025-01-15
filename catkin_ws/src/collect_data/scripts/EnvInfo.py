#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from collect_data.msg import Mass, AABB, ActionRelation, Gripper, Location


pallet_list = ['Pallet_{0:02d}'.format(i + 1) for i in range(4)]
box_list = ['Carton_{0:02d}'.format(i+1) for i in range(110)]


#################################
# Item Info #
class Rack_Info():
    def __init__(self):
        self.object_info = {}
        self.rack_list = ['Rack_02']

        self.move_range = [132.55, 141.45, 141.85, 143.65]  # min_x, min_y, max_x, max_y

        for i, obj in enumerate(self.rack_list):
            # print(obj)
            self.object_info['Rack'][obj] = {
                "name": obj
            }

    def is_trans_in_moverange(self, data_dict):
        result = False
        if 'pose' in data_dict.keys():
            x = data_dict['pose']['translation'][0]
            y = data_dict['pose']['translation'][1]
            if x > self.move_range[0] and x < self.move_range[2] and y > self.move_range[1] and y < self.move_range[3]:
                result = True
        return result

    def is_coor_in_moverange(self,coor):
        x = coor[0]
        y = coor[1]
        if x > self.move_range[0] and x < self.move_range[2] and y > self.move_range[1] and y < self.move_range[3]:
            return True
        else:
            return False

    def cb_pose(self, data):
        # print(obj)
        for idx in range(len(data.transforms)):
            # print(transform)
            tf = data.transforms[idx].transform
            id = data.transforms[idx].child_frame_id

            tmp = {}
            tmp['translation'] = list()
            tmp['translation'].append(tf.translation.x)
            tmp['translation'].append(tf.translation.y)
            tmp['translation'].append(tf.translation.z)
            tmp['rotation'] = list()
            tmp['rotation'].append(tf.rotation.x)
            tmp['rotation'].append(tf.rotation.y)
            tmp['rotation'].append(tf.rotation.z)
            tmp['rotation'].append(tf.rotation.w)

            if 'Rack' in id and id in self.rack_list:
                if self.is_coor_in_moverange(tmp['translation']):
                    self.object_info['Rack'][id]['pose'] = tmp


    def cb_aabb(self, data):
        # print(data)
        id = data.child_frame_id
        if 'Rack' in id and id in self.rack_list :
            if self.is_trans_in_moverange(self.object_info['Rack'][id]):
                self.object_info['Rack'][id]['aabb'] = data.array

class Pallet_Info():
    def __init__(self):

        pallet_list = ['Pallet_{0:02d}'.format(i + 1) for i in range(4)]

        self.object_info = {'Pallet': {}}

        for i, obj in enumerate(pallet_list):
            # print(obj)
            self.object_info['Pallet'][obj] ={
                "name": obj
            }

    def cb_pose(self, data):
        # print(obj)
        for idx in range(len(data.transforms)):
            # print(transform)
            tf = data.transforms[idx].transform
            id = data.transforms[idx].child_frame_id

            tmp = {}
            tmp['translation'] = list()
            tmp['translation'].append(tf.translation.x)
            tmp['translation'].append(tf.translation.y)
            tmp['translation'].append(tf.translation.z)
            tmp['rotation'] = list()
            tmp['rotation'].append(tf.rotation.x)
            tmp['rotation'].append(tf.rotation.y)
            tmp['rotation'].append(tf.rotation.z)
            tmp['rotation'].append(tf.rotation.w)

            if 'Pallet' in id:
                self.object_info['Pallet'][id]['pose'] = tmp

    def cb_aabb(self, data):
        # print(data)
        id = data.child_frame_id
        if 'Pallet' in id:
            self.object_info['Pallet'][id]['aabb'] = data.array

    def cb_location(self, data):
        id = data.child_frame_id
        location = data.location
        # if data.visible == 0:
        #     visible = 'visible'
        # else:
        #     visible = 'unvisible'
        visible = data.visible
        if 'Pallet' in id:
            self.object_info['Pallet'][id]['location'] = location
            self.object_info['Pallet'][id]['visible'] = visible

class BoxInfo():
    def __init__(self):
        for id in box_list:
            self.object_info['Carton'][id] = {
                "name": id
            }

    def cb_pose(self, data):
        # print(obj)
        tf = data.transforms[0].transform
        id = data.transforms[0].child_frame_id
        tmp = {}
        tmp['translation'] = list()
        tmp['translation'].append(tf.translation.x)
        tmp['translation'].append(tf.translation.y)
        tmp['translation'].append(tf.translation.z)
        tmp['rotation'] = list()
        tmp['rotation'].append(tf.rotation.x)
        tmp['rotation'].append(tf.rotation.y)
        tmp['rotation'].append(tf.rotation.z)
        tmp['rotation'].append(tf.rotation.w)

        if 'Carton' in id:
            self.object_info['Carton'][id]['pose'] = tmp

    def cb_mass(self, data):
        tf = Mass()
        # print(data)
        id = data.child_frame_id
        if 'Carton' in id:
            self.object_info['Carton'][id]['mass'] = data.mass

    def cb_aabb(self, data):
        id = data.child_frame_id
        if 'Carton' in id:
            self.object_info['Carton'][id]['aabb'] = data.array

    def cb_location(self, data):
        id = data.child_frame_id
        location = data.location
        visible = data.visible

        if 'Carton' in id:
            self.object_info['Carton'][id]['location'] = location
            self.object_info['Carton'][id]['visible'] = visible

#################################
# Robot Info #
class AMRInfo():
    def __init__(self, robot):
        self.robot_info['ARMLIFT'] = {
            robot: {"name": robot}
        }
        self.armlift_obj_rel = self.init_relation()

    def init_relation(self):
       relations = {
            'L-Rack': [],
            'L-Node': [],
       }
       return relations

    def cb_pose(self, data, robot):

        tf = data.transforms[0].transform
        tmp = {}
        # tmp['name'] = robot
        tmp['translation'] = list()
        tmp['translation'].append(tf.translation.x)
        tmp['translation'].append(tf.translation.y)
        tmp['translation'].append(tf.translation.z)
        tmp['rotation'] = list()
        tmp['rotation'].append(tf.rotation.x)
        tmp['rotation'].append(tf.rotation.y)
        tmp['rotation'].append(tf.rotation.z)
        tmp['rotation'].append(tf.rotation.w)

        if self.is_coor_in_moverange(tmp['translation']):
            self.robot_info['ARMLIFT'][robot]['pose'] = tmp
        # robot_info[robot]['nearest_node'] = nearest_node(tmp['translation'])

    def cb_odometry(self, data, robot):
        pos = data.pose.pose
        twist = data.twist.twist

        tmp = {}
        tmp['position'] = list()
        tmp['position'].append(pos.position.x)
        tmp['position'].append(pos.position.y)
        tmp['position'].append(pos.position.z)
        tmp['orientation'] = list()
        tmp['orientation'].append(pos.orientation.x)
        tmp['orientation'].append(pos.orientation.y)
        tmp['orientation'].append(pos.orientation.z)
        tmp['orientation'].append(pos.orientation.w)
        tmp['linear'] = list()
        tmp['linear'].append(twist.linear.x)
        tmp['linear'].append(twist.linear.y)
        tmp['linear'].append(twist.linear.z)
        tmp['angular'] = list()
        tmp['angular'].append(twist.angular.x)
        tmp['angular'].append(twist.angular.y)
        tmp['angular'].append(twist.angular.z)

        if self.is_trans_in_moverange(self.robot_info['ARMLIFT'][robot]):
            self.robot_info['ARMLIFT'][robot]['odometry'] = tmp

    def cb_jointstate(self, data, robot):
        joint = JointState()
        joint = data

        tmp = {}
        tmp['position'] = joint.position
        tmp['velocity'] = joint.velocity
        tmp['effort'] = joint.effort
        if self.is_trans_in_moverange(self.robot_info['ARMLIFT'][robot]):
            self.robot_info['ARMLIFT'][robot]['joint_state'] = tmp
            self.get_lift_state(robot, tmp)



    def cb_aabb(self, data, robot):
        if self.is_trans_in_moverange(self.robot_info['ARMLIFT'][robot]):
            self.robot_info['ARMLIFT'][robot]['aabb'] = data.array



    def get_lift_state(self, robot, joint_state):
        if joint_state['position'][-1] > 0:
            self.robot_info['ARMLIFT'][robot]['lift_state'] = 'lift_up'
        else:
            self.robot_info['ARMLIFT'][robot]['lift_state'] = 'lift_down'

class PalletizerInfo():
    def __init__(self, robot):

        self.robot_info['Palletizer'] = {
            robot: {"name": robot }
        }
        self.palleitzer_obj_rel = self.init_relation()
        self.current_pallet_name = pallet_list[0]
        self.current_carton_name = box_list[0]
        self.p_carton_rel = ['pickingUp', 'carrying', 'none']
        self.p_pallet_rel = ['placingCartonOn', 'none']
        self.p_palletizer_rel = ['noAction', 'otherAction', 'none']

    def init_relation(self):
       relations = {
            'P-Carton': [],
            'P-Pallet': [],
            'P-Palletizer': []
       }
       return relations

    def cb_palletizer_obj(self, data):
        sub = data.sub
        rel = data.rel
        obj = data.obj
        # print(data)
        type = obj.split('_')[0]
        if 'Carton' == type:
            self.current_carton_name = obj
            self.palleitzer_obj_rel['P-Carton'] = [[sub, rel, obj]]
        elif 'Pallet' == type:
            self.current_pallet_name = obj
            self.palleitzer_obj_rel['P-Pallet'] = [[sub, rel, obj]]
        elif 'Palletizer' == type:
            self.palleitzer_obj_rel['P-Palletizer'] = [[sub, rel, obj]]


    def cb_arm_pose(self, data, robot):

        tf = data.transforms[0].transform
        tmp = {}
        # tmp['name'] = robot
        tmp['translation'] = list()
        tmp['translation'].append(tf.translation.x)
        tmp['translation'].append(tf.translation.y)
        tmp['translation'].append(tf.translation.z)
        tmp['rotation'] = list()
        tmp['rotation'].append(tf.rotation.x)
        tmp['rotation'].append(tf.rotation.y)
        tmp['rotation'].append(tf.rotation.z)
        tmp['rotation'].append(tf.rotation.w)

        self.robot_info['Palletizer'][robot]['arm_pose'] = tmp

    def cb_gripper_pose(self, data, robot):

        tf = data.transforms[0].transform
        tmp = {}
        # tmp['name'] = robot
        tmp['translation'] = list()
        tmp['translation'].append(tf.translation.x)
        tmp['translation'].append(tf.translation.y)
        tmp['translation'].append(tf.translation.z)
        tmp['rotation'] = list()
        tmp['rotation'].append(tf.rotation.x)
        tmp['rotation'].append(tf.rotation.y)
        tmp['rotation'].append(tf.rotation.z)
        tmp['rotation'].append(tf.rotation.w)

        self.robot_info['Palletizer'][robot]['gripper_pose'] = tmp

    def cb_gripper_state(self, data, robot):

        if data.state:
            state = 'close'
        else:
            state = 'open'

        self.robot_info['Palletizer'][robot]['gripper_state'] = state

    def cb_jointstate(self, data, robot):

        joint = JointState()
        joint = data

        tmp = {}
        tmp['position'] = joint.position
        tmp['velocity'] = joint.velocity
        self.robot_info['Palletizer'][robot]['joint_state'] = tmp


    def cb_aabb(self, data, robot):
        # id = data.child_frame_id
        self.robot_info['Palletizer'][robot]['aabb'] = data.array

