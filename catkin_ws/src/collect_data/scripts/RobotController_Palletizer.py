#!/usr/bin/env python3
import random

import rospy
from collect_data.srv import PalletService_kgu
import sys, select, termios, tty
FLAG = False

robotNum = 2

Dummy_scenario = {
    'Palletizer': [('MoveArmToInitPose', 'Palletizer_01', '', ''), ('MoveArmToAttach', 'Palletizer_01', '', ''), ('CloseGripper', 'Palletizer_01', '', ''),
                   ('MoveArmToDetach', 'Palletizer_01', '', ''), ('OpenGripper', 'Palletizer_01', '', ''),
                   ] * 1000

}

robot_info = {}

def init():
    print('init start')
    global robot_info
    current_posX = 0.0
    current_posY = 0.0
    euler_theta = 0.0
    current_lift_joint = 0.0

    for robotIndex in range(robotNum):
        robotName = 'AMR_LIFT{0:0>2}'.format(robotIndex + 1)
        robotObj = RobotInfo(robotName=robotName, pos=[current_posX, current_posY], liftJoint=current_lift_joint,
                             euler_theta=euler_theta)
        robot_info[robotName] = robotObj


def getKey(key_timeout):
    print('START \t Input Any Key \t')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def robotLift_Plan(robotName, target_joint, order):
    status = 'NONE'
    result = False
    print('START {} ROBOT LIFT PLANNING \t Target Lift Joint = {}'.format(robotName, target_joint[0]))
    while not rospy.is_shutdown():

        if result:
            print('LIFT & UNLIFT FINISH')
            break
        response = liftReq(robotName, target_joint[0], order)
        result = response.success
        # rospy.sleep(0.2)

def robotMove_Plan(robotName, path_list, action,velocity):
    current_node_idx = 0
    status = 'NONE'
    print('START {} ROBOT MOVE PLANNING \t ROOT = {} '.format(robotName, path_list))
    # r = rospy.Rate(0.1)
    while not rospy.is_shutdown():

        if current_node_idx == len(path_list):
            break

        targetNode = path_list[current_node_idx]

        response = moveReq(robotName, targetNode, action,velocity)

        status = response.status
        result = response.success

        # if status in ['Turn Left', 'Turn Right']:
        #     # rospy.loginfo(f"=== {robotName} Status : {status} ===")

        if result:
            print(robotName,': node : ',targetNode)
            # rospy.loginfo(f"=== {robotName} Status : {status} ===")
            current_node_idx += 1
        # r.sleep
    return status

def robotMoveBackwardToNode(robotName,node_num):
    result = False
    print('START {} ROBOT POST MOVE PLANNING \t ROOT '.format(robotName))
    # r = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        response = postReq(robotName,node_num)
        result = response.success
        if result:
            print('{}--> POST_MOVE {} '.format(robotName, result))
            break
        # r.sleep
    return result


def robotPalletService(robotName,plan,wait_action):
    print('START {} ROBOT Pallet PLANNING \t ROOT '.format(robotName))
    result = False
    print(plan)

    print(plan)
    response = palletizerReq(plan[0],plan[1], plan[2], plan[3])
    result = response.result
    if plan[0] == wait_action:
        print('Wait....')
        rospy.sleep(8)
    print("action result:", result)
    FLAG = False

def temp_demo(robotName):
    for i, plan in enumerate(Dummy_scenario[robotName]):
        if i % 5 == 0:
            wait_action = random.choice(['MoveArmToInitPose', 'OpenGripper'])
        # Palletizer Robot Action
        if robotName == 'Palletizer':
            robotPalletService(robotName,plan,wait_action)


def cb_packingFinish(data):
    print("packing finish", data)
    if data.palletizer == "Palletizer1" and data.node == "1":
        FLAG = True
    # rospy.spin()

if __name__ == "__main__":

    rospy.init_node("kgu_Dummy_TM")
    rospy.loginfo("=== Robot Control Client Started ===")
    rospy.loginfo("=== Robot Control Waiting ===")

    palletizerReq = rospy.ServiceProxy('control_palletizer',PalletService_kgu)
    settings = termios.tcgetattr(sys.stdin)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None
    key = getKey(key_timeout)
    r = rospy.Rate(10)

    answer = temp_demo('Palletizer')
    rospy.spin()

