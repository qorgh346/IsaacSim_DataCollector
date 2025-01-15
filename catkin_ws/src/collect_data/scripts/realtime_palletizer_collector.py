#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from sensor_msgs.msg import JointState

from nav_msgs.msg import Odometry
from collect_data.msg import Mass, AABB, ActionRelation, Gripper, Location
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
import pickle as pkl
import os
import json
import math
import copy
import numpy as np
from EnvInfo import Pallet_Info, BoxInfo, PalletizerInfo

collect = {}
map_vertext = dict()
data_dir = '/[~~~]/src/collect_data/dataset/palletizer_data'



def write_json(json_data, f_name):
    print("=== Write {} Json File ===".format(f_name))
    file = "{}/{}".format(data_dir, f_name)
    with open(file, 'w', encoding='utf-8') as f:
        json.dump(json_data, f, indent="\t")


def write_pkl(lidar_json, f_name):
    print("=== Write {} Lidar pkl File ===".format(f_name))
    file = os.path.join(data_dir, f_name)
    with open(file, 'wb') as f:
        pkl.dump(lidar_json, f)


def select_valid_obj(objects_info, pallet_id, carton_id):
    carton_info = objects_info['Carton']
    pallet_info = objects_info['Pallet']
    new_dict = {
        'Carton': {
            carton_id: carton_info[carton_id]
        },
        'Pallet': {
            pallet_id: pallet_info[pallet_id]
        }
    }

    return new_dict

def delete_overlap_rel(rel_list):
    new_rel = dict()
    for key, value in rel_list.items():
        for r in value:
            if not key in new_rel.keys() :
                new_rel[key] = [r]
            elif r not in new_rel[key]:
                new_rel.append(r)
    return new_rel

def removeAll(dir):
    if len(os.listdir(dir)) > 2:
        if os.path.exists(dir):
            for file in os.scandir(dir):
                os.remove(file.path)
            return "Remove All File"
    else:
        if not os.path.isdir(dir):
            os.mkdir(dir)
            return "Create Folder"
    return "Not"


def get_last_timestamp(TIME_STAMP):
    global collect
    new = dict()
    times = len(collect.keys())
    if times == 0 or times < TIME_STAMP:
        return False, new

    count = 0
    for i in range(times-TIME_STAMP, times):
        new['timestamp_{}'.format(count)] = collect['timestamp_{}'.format(i)]
        count +=1
    return True, new

def listener(TIME_STAMP, SLEEP):
    rospy.init_node('init', anonymous=True)
    global collect

    count = 0
    file_num = 0

    palletizer = PalletizerInfo("Palletizer_01")
    pallet = Pallet_Info()
    carton = BoxInfo()

    # Palletizer
    rospy.Subscriber('/Palletizer_joint_states', JointState, palletizer.cb_jointstate, ("Palletizer_01"))
    rospy.Subscriber('/Palletizer_aabb', AABB, palletizer.cb_aabb, ("Palletizer_01"))
    rospy.Subscriber('/Arm_pose', TFMessage, palletizer.cb_arm_pose, ("Palletizer_01"))
    rospy.Subscriber('/Gripper_pose', TFMessage, palletizer.cb_gripper_pose, ("Palletizer_01"))
    rospy.Subscriber('/Gripper_state', Gripper, palletizer.cb_gripper_state, ("Palletizer_01"))

    # Palletizer Relation
    # mcArbi 프레임워크 속 규칙 기반 맥락 추론기로부터 Palletizer_action_rel Topic값 저장
    # mcArbi 프레임워크 (코드 공개 권한 X)
    rospy.Subscriber("/Palletizer_action_rel", ActionRelation, palletizer.cb_palletizer_obj)

    # Pallet
    # rospy.Subscriber('/RackPallet_pose', TFMessage, pallet.cb_pose)
    rospy.Subscriber('/PalletRack01_pose', TFMessage, pallet.cb_pose)

    rospy.Subscriber('/PalletRacks_aabb', AABB, pallet.cb_aabb)
    rospy.Subscriber('/Pallet_location', Location, pallet.cb_location)

    # Carton
    rospy.Subscriber('/Object_pose', TFMessage, carton.cb_pose)
    rospy.Subscriber('/Object_aabb', AABB, carton.cb_aabb)
    rospy.Subscriber('/Object_location', Location, carton.cb_location)


    while not rospy.is_shutdown():
        rospy.sleep(SLEEP)
        get_last, data = get_last_timestamp(TIME_STAMP)

        if get_last :
            write_json(data, file_name)
            file_num += 1
        file_name = 'data_{}.json'.format(file_num)
        timestamp = "timestamp_{}".format(count)

        collect[timestamp] = {}
        robot_tmp = copy.deepcopy(palletizer.robot_info)
        object_tmp = copy.deepcopy(carton.object_info.update(pallet.object_info))


        # Init
        # Get Relations: Palletizer --> Carton/Pallet
        palletizer_obj_rel = palletizer.palleitzer_obj_rel
        palletizer_obj_rel = delete_overlap_rel(palletizer_obj_rel)

        # Select Valid Object Info
        valid_objects = select_valid_obj(object_tmp, palletizer.current_pallet_name, palletizer.current_carton_name)

        # Collect total info
        collect[timestamp]['robots'] = robot_tmp
        collect[timestamp]['objects'] = valid_objects
        collect[timestamp]['relationships'] = palletizer_obj_rel
        # print(valid_objects)

        for key, value in palletizer_obj_rel.items():
            # if rel[1] == rel_ClassNames.index('none'):
            #     continue
            for r in value:
                print('{} {} {} '.format(r[0],r[1], r[2]))

        print('{} stamp: {} count: {}'.format(file_name, TIME_STAMP, count))
        print()
        count += 1
        palletizer.init_relation()


if __name__ == '__main__':
    ## Model Realtime Test Input Dir

    data_dir = '/[~~~]/src/collect_data/dataset/palletizer_data'
    TIME_STAMP = 5
    SLEEP = 1

    removeAll(data_dir)
    try:
        listener(TIME_STAMP, SLEEP)
    except rospy.ROSInterruptException:
        pass

