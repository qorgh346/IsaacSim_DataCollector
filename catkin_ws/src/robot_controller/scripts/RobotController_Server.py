#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from robot_controller.srv import MoveToNodeService, LiftService, MoveBackService, PalletService
from robotInfo import RobotInfo
from nav_msgs.msg import Odometry
from math import atan2
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import JointState
import json
import sys


robot_number = 2
robot_info = {}
node2pos = {}

def init():
    global robot_info
    global node2pos

    # 사전 정의된 Map 로드 <사진 첨부>
    # Map Info
        # 1. type 1 : 뭔 지 찾아서 적어야함
        # 2. 절대좌표, 상대좌표 포함
    json_dir = '[~~~]/catkin_ws/IsaacAMR_ws/utils/demo_vertex.json'
    node2pos = load_Map(json_dir)

    x = 0.0
    y = 0.0
    theta = 0.0
    current_lift_joint = 0.0

    # 로봇 객체 생성
    for robotIndex in range(robot_number):
        robotName = 'AMR_LIFT{0:0>2}'.format(robotIndex + 1)
        robotObj = RobotInfo(robotName=robotName, pos=[x, y], liftJoint=current_lift_joint, euler_theta=theta)

        #객체 생성 후, 딕셔너리 자료구조에 저장
        robot_info[robotName] = robotObj
    

def load_Map(json_dir):
    with open(json_dir, 'r') as f:
        json_data = json.load(f)
    return json_data


def newOdom(msg):
    #AMR_Lift01&02 Od SubScribe odometry_robot 
    rot_q = msg.pose.pose.orientation
    (roll, pitch, euler_theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    robot_name = msg.child_frame_id
    robot = robot_info[robot_name]
    robot.setTheta(euler_theta)
    robot.setPos(pos_x=msg.pose.pose.position.x, pos_y=msg.pose.pose.position.y)

def newJoint_1(msg):
    #AMR_Lift01 Lift Joint SubScribe
    joint_name = 'lift_joint'
    joint_idx = msg.name.index(joint_name)
    joint_pos = msg.position[joint_idx]
    robot = robot_info['AMR_LIFT01']
    robot.setJointPos(round(joint_pos * 100, 2))

def newJoint_2(msg):
    # AMR_Lift02 Lift Joint SubScribe
    joint_name = 'lift_joint'
    joint_idx = msg.name.index(joint_name)
    joint_pos = msg.position[joint_idx]
    robot = robot_info['AMR_LIFT02']
    robot.setJointPos(round(joint_pos * 100, 2))

def check_pos(goal_pos, current_pos, threshold):
    # 목표 노드의 좌표와 현재 AMR 로봇의 좌표 간의 상대거리를 비교함
    # 거리가 사전 정의된 임계값(threshold) 이하면 가까우니 목표 노드에 도착했다고 판단
    pos_x = current_pos[0]
    pos_y = current_pos[1]
    goal_x = goal_pos[0]
    goal_y = goal_pos[1]
    if abs(pos_x - goal_x) <= threshold and abs(pos_y - goal_y) <= threshold:
        return True
    return False

def handle_moveBackToNode_cb(req):
    #AMR 로봇이 팔렛을 Lift 또는 UnLift 한 후 뒤로 빠지는 동작에 대한 콜백함수
    #사용 이유 : AMR 로봇이 Lift & UnLift 동작 후 목표 노드로 회전할 때 장애물 고려
    robotName = req.robotName
    robot = robot_info[robotName]

    goal_x = robot.post_goal_x
    goal_y = robot.post_goal_y
    result = False
    theta = robot.getTheta()
    current_x, current_y = robot.getPos()

    if not robot.backward_flag:
        if theta > 0.75 and theta < 2.25:
            goal_x = current_x
            goal_y = current_y - robot.backward_degree
        elif theta < -0.75 and theta > -2.25:
            goal_x = current_x
            goal_y = current_y + robot.backward_degree
        if abs(theta) > 2.25:
            goal_x = current_x + robot.backward_degree
            goal_y = current_y
        elif abs(theta) < 0.75:
            goal_x = current_x - robot.backward_degree
            goal_y = current_y

    speed = Twist()
    speed.linear.x = -0.8
    speed.angular.z = 0.0
    if robot.check_pos(goal_pos=(goal_x, goal_y), current_pos=(current_x, current_y), threshold=0.3):
        rospy.loginfo(f"=== {robotName} Robot Move_Back_To_Node Success ===")
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        result = True
        robot.setBackward_flag(flag=False)

    else:
        robot.post_goal_x = goal_x
        robot.post_goal_y = goal_y
        robot.setBackward_flag(flag=True)

    # 후진하기 위해서 아이작 시뮬레이터의 Transport 로봇과 Ros 통신으로 /cmd_vel 주소로 후진 or 정지값 publish
    odom_pub_list[robotName].publish(speed)
    return result

def handle_lift_cb(req):
    # 팔렛을 Lift 또는 UnLift 동작에 대한 콜백함수
    # AMR 로봇의 Lift Joint 값을 천천히 조절
    # (너무 빨리 Lift & UnLift 하면, 적재된 상자가 떨어지거나 팔렛이 뒤틀림)

    joint_command = JointState()
    joint_command.name = ['lift_joint']

    robotName = req.robotName
    robot = robot_info[robotName]
    value = req.target_value
    order = req.order
    current_joint = robot.get_lift_joint()
    rospy.loginfo(f"Received Current Lift Joint : {current_joint}")

    if order == 'LiftUp':
        if current_joint > 0.1:
            return True
        else:
            current_joint += value
    elif order == 'PutDown':
        if current_joint < 0:
            return True
        else:
            current_joint -= value

    robot.set_lift_joint(current_joint)
    joint_command.position = np.array([current_joint])

    #/AMR_LIFT{0:0>2}_joint_command

    # 아이작 시뮬레이터의 Transport 로봇과 Ros 통신으로 /[RobotName]_joint_command 로봇 등에 달린 조인트값(???) publish
    # 0이면 내려가고, 1이면 올라감, 0 -->1 로 점차 증가하면 점점 천천히 올라감
    joint_pub_list[robotName].publish(joint_command)
    return False

def handle_moveToNode_cb(req):
    # AMR 로봇이 목표 노드까지 이동하기 위한 콜백함수
    # 현재 로봇이 바라보는 방향과 위치와 목표 노드까지의 사이 각도, 위치를 고려하여 이동

    robotName = req.robotName
    robot = robot_info[robotName]
    threshold = robot.threshold
    result = False
    robotFoward = robot.forward_flag
    action = req.action
    tNumber = req.node
    theta = robot.getTheta()
    current_posX, current_posY = robot.getPos()

    robot_index = int(robotName[-1]) # ex. AMR_LIFT01, AMR_LIFT02 ...

    goal = node2pos['Vertex'][str(tNumber)]['relative_pos'][robot_index-1]
    print('robot {} goal = {}'.format(robotName,goal))
    goal_x, goal_y = goal[0],goal[1]

    Rotation_section_L = 0.009#0.008
    Rotation_section_M = 0.17#0.15
    Rotation_section_H = 0.4#0.3

    if action == 'testMove':
        if not robot.prior_flag:
            robot.setStatus(status='READY')
            prior_x = robot.pos_x
            prior_y = robot.pos_y
            robot.setPrior_flag(flag=True)
        else:
            prior_x = robot.prior_x
            prior_y = robot.prior_y
        rGoal_x, rGoal_y = goal_x, goal_y

    elif action == 'MoveToNode':
        if not robot.prior_flag:
            robot.setStatus(status='READY')
            prior_x = robot.goal_x
            prior_y = robot.goal_y
            robot.setPrior_flag(flag=True)
        else:
            prior_x = robot.prior_x
            prior_y = robot.prior_y

        # 한번에 날라가는게 아니기 때문에 threshold 값 만큼 속도를 내가며 이동
        rGoal_x, rGoal_y = robot.getNewGoal(goal_x, goal_y, threshold)

    robot.setPrior_goal(prior_x, prior_y)
    robot.setGoalPos(goal_x, goal_y)

    speed = Twist()
    inc_x = goal_x - prior_x
    inc_y = goal_y - prior_y

    #목표까지 angle 값 계산
    angle_to_goal = atan2(inc_y, inc_x)

    #목표까지의 직선 거리 계산
    distance_to_goal = np.sqrt(inc_x * inc_x + inc_y * inc_y)

    if distance_to_goal >= 0.3:
        speed.linear.x = 9
        speed.angular.z = 0.0
        if not robot.theta_flag:
            prior_theta = robot.getTheta()
            robot.setTheta_flag(flag=True)
            robot.setPriorTheta(prior_theta)

        if not robotFoward:
            # robotFoward가 False이면 전진을 해야하는 조건에 만족하지 못했기 때문에
            # 로봇은 반드시 목표 노드를 향해 회전을 해야함.
            # 로봇이 전진을 해야하는 조건은 아래 있음 (뭔지 까먹음)

            prior_theta = robot.prior_theta
            if abs(angle_to_goal) > 1.5 and abs(angle_to_goal) < 3 and abs(theta) > abs(angle_to_goal):
                if angle_to_goal > 0:
                    if angle_to_goal * prior_theta > 0 and abs(angle_to_goal - theta) > Rotation_section_L:
                       #위 조건에 만족하면 Turn Left 해야함
                        speed.linear.x = 0.0
                        speed.angular.z = -0.08

                        if angle_to_goal - theta > Rotation_section_H:
                            speed.angular.z = -0.2
                        elif Rotation_section_H > angle_to_goal - theta > Rotation_section_M:
                            speed.angular.z = -0.1
                        else:
                            speed.angular.z = -0.08

                        robot.setForward_flag(flag=False)
                        robot.setStatus(status='Turn Left')
                    elif angle_to_goal * prior_theta < 0 and abs(angle_to_goal - theta) > Rotation_section_L:
                        # 위 조건에 만족하면 Turn Left 해야함
                        speed.linear.x = 0.0
                        speed.angular.z = -0.08

                        if abs(angle_to_goal - theta) > Rotation_section_H:
                            speed.angular.z = -0.2
                        elif Rotation_section_H > abs(angle_to_goal - theta) > Rotation_section_M:
                            speed.angular.z = -0.1
                        else:
                            speed.angular.z = -0.08

                        robot.setForward_flag(flag=False)
                        robot.setStatus(status='Turn Left')

                    else:
                        robot.setForward_flag(flag=True)
                        robot.setStatus(status='Forward')

                elif angle_to_goal < 0:
                    if angle_to_goal * prior_theta > 0 and abs(angle_to_goal - theta) > Rotation_section_L:
                        speed.linear.x = 0.0
                        speed.angular.z = 0.08

                        if angle_to_goal - theta < -Rotation_section_H:
                            speed.angular.z = 0.2
                        elif -Rotation_section_H < angle_to_goal - theta < -Rotation_section_M:
                            speed.angular.z = 0.1
                        else:
                            speed.angular.z = 0.08

                        robot.setForward_flag(flag=False)
                        robot.setStatus(status='Turn Left')
                    
                    elif angle_to_goal * prior_theta < 0 and abs(angle_to_goal - theta) > Rotation_section_L:
                        speed.linear.x = 0.0
                        speed.angular.z = -0.2

                        if angle_to_goal - theta < -Rotation_section_H:
                            speed.angular.z = -0.2
                        elif -Rotation_section_H < angle_to_goal - theta < -Rotation_section_M:
                            speed.angular.z = -0.1
                        else:
                            speed.angular.z = -0.08

                        robot.setForward_flag(flag=False)
                        robot.setStatus(status='Turn Left')
                    else:
                        robot.setForward_flag(flag=True)
                        robot.setStatus(status='Forward')
            elif angle_to_goal > 3:
                if theta < 0 and angle_to_goal - abs(theta) > Rotation_section_L:
                    speed.linear.x = 0.0
                    speed.angular.z = -0.08

                    if angle_to_goal - abs(theta) > Rotation_section_H:
                        speed.angular.z = -0.2
                    elif Rotation_section_H > angle_to_goal - abs(theta) > Rotation_section_M:
                        speed.angular.z = -0.1
                    else:
                        speed.angular.z = -0.08

                    robot.setForward_flag(flag=False)
                    robot.setStatus(status='Turn Left')
                elif theta > 0 and angle_to_goal - abs(theta) > Rotation_section_L:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.08

                    if angle_to_goal - abs(theta) > Rotation_section_H:
                        speed.angular.z = 0.2
                    elif Rotation_section_H > angle_to_goal - abs(theta) > Rotation_section_M:
                        speed.angular.z = 0.1
                    else:
                        speed.angular.z = 0.08

                    robot.setForward_flag(flag=False)
                    robot.setStatus(status='Turn Right')
                else:
                    robot.setForward_flag(flag=True)
            elif angle_to_goal - theta > Rotation_section_L:
                speed.linear.x = 0.0
                speed.angular.z = 0.08

                if angle_to_goal - theta > Rotation_section_H:
                    speed.angular.z = 0.2
                elif Rotation_section_H > angle_to_goal - theta > Rotation_section_M:
                    speed.angular.z = 0.1
                else:
                    speed.angular.z = 0.08

                robot.setForward_flag(flag=False)
                robot.setStatus(status='Turn Left')

            elif angle_to_goal - theta < - Rotation_section_L:
                speed.linear.x = 0.0
                speed.angular.z = -0.08

                if angle_to_goal - theta < -Rotation_section_H:
                    speed.angular.z = -0.2
                elif -Rotation_section_H < angle_to_goal - theta < -Rotation_section_M:
                    speed.angular.z = -0.1
                else:
                    speed.angular.z = -0.08

                robot.setForward_flag(flag=False)
                robot.setStatus(status='Turn Right')
            else:
                robot.setForward_flag(flag=True)
                robot.setStatus(status='Forward')

    if robot.check_pos(goal_pos=(rGoal_x, rGoal_y), current_pos=(current_posX, current_posY), threshold=threshold):
        rospy.loginfo(f"=== {robotName} Robot Move To Node  [{rGoal_x},{rGoal_y}] Success ===")
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        result = True
        robot.setPrior_flag(flag=False)
        robot.setBackward_flag(flag=False)
        robot.setTheta_flag(flag=False)
        robot.setForward_flag(flag=False)

    odom_pub_list[robotName].publish(speed)
    return (result, str(robot.getStatus()))


if __name__ == '__main__':
    init()

    rospy.init_node("test_rosbridge")
    rospy.loginfo("=== Robot Move Control Started ===")
    # odom_sub = (rospy.Subscriber("/AMR_LIFT{0:0>2}_odometry_robot".format(robotIndex+1), Odometry, newOdom) for robotIndex in range(robot_number) )
    odom_sub = rospy.Subscriber("/AMR_LIFT01_odometry_robot", Odometry, newOdom)
    odom_sub = rospy.Subscriber("/AMR_LIFT02_odometry_robot", Odometry, newOdom)
    joint_sub1 = rospy.Subscriber("/AMR_LIFT01_joint_states", JointState, newJoint_1)
    joint_sub2 = rospy.Subscriber("/AMR_LIFT02_joint_states", JointState, newJoint_2)
    joint_pub_list = {
        'AMR_LIFT{0:0>2}'.format(i + 1): rospy.Publisher("/AMR_LIFT{0:0>2}_joint_command".format(i + 1), JointState,
                                                         queue_size=1) for i in range(robot_number)}
    odom_pub_list = {
        'AMR_LIFT{0:0>2}'.format(i + 1): rospy.Publisher("/AMR_LIFT{0:0>2}_cmd_vel".format(i + 1), Twist, queue_size=1)
        for i in range(robot_number)}

    move_server = rospy.Service("/control_robot_move", MoveToNodeService, handle_moveToNode_cb)
    motion_server = rospy.Service("/control_robot_lift", LiftService, handle_lift_cb)
    move_post_server = rospy.Service("/control_robot_MoveBackwardToNode", MoveBackService, handle_moveBackToNode_cb)
    # palletizer_server = rospy.Service('control_palletizer', PalletService, handle_palletizer_cb)

    rospy.spin()
