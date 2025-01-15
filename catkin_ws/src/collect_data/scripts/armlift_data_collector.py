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

from Relation_Generation import Gen_Relation
from EnvInfo import Rack_Info, AMRInfo
collect = {}

map_vertext = dict()

data_dir = '[~~]/collect_data/dataset/armlift_data'





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

    lift = AMRInfo("AMR_LIFT01")
    rack = Rack_Info()

    # Lift
    rospy.Subscriber('/AMR_LIFT01_pose', TFMessage, lift.cb_pose, ("AMR_LIFT01"))
    rospy.Subscriber('/AMR_LIFT01_odom', Odometry, lift.cb_odometry, ("AMR_LIFT01"))
    rospy.Subscriber('/AMR_LIFT01_joint_states', JointState, lift.cb_jointstate, ("AMR_LIFT01"))
    rospy.Subscriber('/AMR_LIFT01_aabb', AABB, lift.cb_aabb, ("AMR_LIFT01"))

    #
    rospy.Subscriber('/RackPallet_pose', TFMessage, rack.cb_pose)

    robotNames = ['AMR_LIFT01']
    rackNames = ['Rack_02']

    while not rospy.is_shutdown():
        rospy.sleep(SLEEP)
        get_last, data = get_last_timestamp(TIME_STAMP)
        # if count == TIME_STAMP:
        #     print("WRITE")
        #     write_json(collect, file_name)
        #     file_num += 1
        #     count = 0
        #     print()
        if get_last :
            print("WRITE")
            write_json(data, file_name)
            file_num += 1
            print()
        file_name = 'temp_data_{}.json'.format(file_num)
        timestamp = "timestamp_{}".format(count)

        collect[timestamp] = {}
        robot_tmp = copy.deepcopy(lift.robot_info)
        object_tmp = copy.deepcopy(rack.object_info)
        # print(robot_tmp)


        # Init
        # Get Relations: Palletizer --> Carton/Pallet
        gen_relation = Gen_Relation(robotNames, rackNames)
        lift_obj_rel = gen_relation.get_lift_objects_rel(robot_tmp, object_tmp)

        # Collect total info
        collect[timestamp]['robots'] = robot_tmp
        collect[timestamp]['objects'] = object_tmp
        collect[timestamp]['relationships'] = lift_obj_rel
        # print(valid_objects)
        for key, value in lift_obj_rel.items():
            # if rel[1] == rel_ClassNames.index('none'):
            #     continue
            print(key, value)
            for r in value:
                print(r)

        print('{} stamp: {} count: {}'.format(file_name, TIME_STAMP, count))
        print()
        count += 1
        # palletizer.init_relation()


if __name__ == '__main__':
    ## Model Realtime Test Input Dir

    TIME_STAMP = 5
    SLEEP = 1

    removeAll(data_dir)
    try:
        listener(TIME_STAMP, SLEEP)
    except rospy.ROSInterruptException:
        pass

