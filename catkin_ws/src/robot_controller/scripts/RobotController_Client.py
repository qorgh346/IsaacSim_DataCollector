#!/usr/bin/env python3
import rospy
from robot_controller.srv import MoveToNodeService, MoveBackService, LiftService, PalletService
import sys, select, termios, tty
from robotInfo import RobotInfo
import time

robotNum = 2

scenario_list = {
    'AMR_LIFT01':[('MoveToNode',[100,101,142,141,151,152,1]),('LiftUp',[0.002]),('MoveBackwardToNode',[32]),
        ('MoveToNode',[152,1513,1512,1511,151,141,1403,1402,140,139,138,1380,26]),('PutDown',[0.002]),('MoveBackwardToNode',[54]),
                  ('MoveToNode',[138,139,140,1402,1403,141,142]),
        ('MoveToNode',[101,100,143])
                  ],

    'AMR_LIFT02':[('MoveToNode',[120,119,1191,118,117,116,1162,115,1151,20]),('LiftUp',[0.002]),('MoveBackwardToNode',[12]),
                  ('MoveToNode',[1151,115,1152,114,1142,113,1132,112,1122,111,1112,1113,109,1092,1093,107,1072,106,1062,105,1052,104,1042,103,1032,102,1021,1022,1023,1024,1025,1026,101,1421,142,146]),
                  ('PutDown',[0.002]),('MoveBackwardToNode',[11]),
                  ('MoveToNode',[1462,142,101,1025,1024,1023,1022,102,1032,103,1042,104,1052,105,1062,106,1072,107,1093,1092,109,1113,1112,111,112,113,1142,114,1152,115,1162,116,117,118,1191,119,120,157])
                  ],
    'Palletizer': [('MoveArmToInitPose', ''), ('MoveArmToAttach', 'Box01'), ('CloseGripper', ''),('MoveArmToDetach', 'Pallet01'), ('OpenGripper', ''),
                   ('MoveArmToInitPose', ''), ('MoveArmToAttach', 'Box02'), ('CloseGripper', ''),('MoveArmToDetach', 'Pallet01'), ('OpenGripper', '')
                   ]
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

def robotLift_Plan(robotName, velocity, order):
    status = 'NONE'
    result = False
    print('START {} ROBOT LIFT PLANNING \t Velocity = {}'.format(robotName, velocity))

    while not rospy.is_shutdown():
        if result:
            #로봇이 성공적으로 리프트 & 언리프트 수행했으면 while 탈출
            print('LIFT & UNLIFT FINISH')
            rospy.loginfo(f"{order} FINISH")
            break

        # /control_robot_lift  로봇 이름과 목표 노드, 행동, 속도값 전송
        response = liftReq(robotName, velocity, order)
        # RobotController_Server.py에서 처리 후, 결과값 반환 -> response 저장

        result = response.success
        # rospy.sleep(0.2)

def robotMove_Plan(robotName, path_list, action,velocity):
    current_node_idx = 0
    status = 'NONE'
    print('START {} ROBOT MOVE PLANNING \t ROOT = {} '.format(robotName, path_list))
    # r = rospy.Rate(0.1)
    while not rospy.is_shutdown():

        if current_node_idx == len(path_list):
            # 시나리오에 작성된 목표 노드들 모두 이동했으면 while문 탈출
            break

        targetNode = path_list[current_node_idx]

        # /control_robot_move Service  로봇 이름과 목표 노드, 행동, 속도값 전송
        response = moveReq(robotName, targetNode, action,velocity)
        # RobotController_Server.py에서 처리 후, 결과값 반환 -> response 저장

        status = response.status
        result = response.success

        if status in ['Turn Left', 'Turn Right']:
            rospy.loginfo(f"=== {robotName} Status : {status} ===")

        if result:
            #정상적으로 목표 노드까지 잘 도착했으면, result = True
            rospy.loginfo(f"=== {robotName} Status : {status} ===")

            # 다음 목표 노드로 이동하기 위해 Index 값 증가
            current_node_idx += 1

        # r.sleep
    return status

def robotMoveBackwardToNode(robotName,node_num):
    result = False
    print('START {} ROBOT POST MOVE PLANNING \t ROOT '.format(robotName))
    # r = rospy.Rate(0.1)
    while not rospy.is_shutdown():

        if result:
            # 로봇이 성공적으로 목표 노드까지 후진했으면 while 탈출
            print('{}--> POST_MOVE {} '.format(robotName, result))
            break

        # /control_robot_MoveBackwardToNode Service  로봇 이름과 후진할 목표 노드 번호 전송
        response = postReq(robotName,node_num)
        # RobotController_Server.py에서 처리 후, 결과값 반환 -> response 저장
        result = response.success
    return result


def robotPalletService(robotName,plan):
    # 현재 Ur10 로봇 제어 불가능 -> 오류 발생
    # 이유 : 아이작 시뮬레이터 Ur10 로봇의 각 행동 단위 구분 X
    # 해결 : 아이작 시뮬레이터가 제공하는 Box 팔로잉 기능을 이용해서 내부 코드에서 직접 제어
    print('START {} ROBOT Pallet PLANNING \t ROOT '.format(robotName))
    result = False
    print(plan)

    response = palletizerReq(robotName,plan[0], plan[1])
    result = response.result
    print("action result:", result)


def start_demo(robotName):
    for plan in scenario_list[robotName]:

        # Palletizer Robot Action
        if robotName == 'Palletizer':
            robotPalletService(robotName,plan)
            rospy.sleep(1)

        else:
            # AMR Robot Action
            if plan[0] == 'wait':
                sec = plan[1][0]
                print('waiting UR10...', sec)
                rospy.sleep(sec)

            velocity = 5.0 #로봇 이동 속도

            if plan[0] == 'testMove': #Test
                robotMove_Plan(robotName, plan[1], 'testMove',velocity)

            if plan[0] == 'MoveToNode': #목표 노드로 전진
                # plan[1] = 목표 노드 번호 리스트 (여러개)
                robotMove_Plan(robotName, plan[1], 'MoveToNode',velocity)

            if plan[0] in ['LiftUp', 'PutDown']: #AMR 리프트 업, 다운
                # plan[1] = 리프트 업, 다운 속도값 (velocity, 0~1)
                robotLift_Plan(robotName, plan[1], plan[0])

            if plan[0] == 'MoveBackwardToNode': #후진
                #plan[1] = 목표 노드 번호 1개 (후진)
                robotMoveBackwardToNode(robotName,plan[1])

if __name__ == "__main__":
    init()
    import multiprocessing

    rospy.init_node("Robot_Control_Client")
    rospy.loginfo("=== Robot Control Client Started ===")
    rospy.loginfo("=== Robot Control Waiting ===")

    rospy.wait_for_service("control_robot_move")
    moveReq = rospy.ServiceProxy('control_robot_move', MoveToNodeService)

    rospy.wait_for_service("control_robot_lift")
    liftReq = rospy.ServiceProxy('control_robot_lift', LiftService)

    rospy.wait_for_service("control_robot_MoveBackwardToNode")
    postReq = rospy.ServiceProxy('control_robot_MoveBackwardToNode', MoveBackService)

    # palletizerReq = rospy.ServiceProxy('control_ur10_paletting',PalletService)
    settings = termios.tcgetattr(sys.stdin)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None
    key = getKey(key_timeout)
    # r = rospy.Rate(10)
    p = multiprocessing.Pool(processes=4)

    # answer = p.map(temp_demo, ['AMR_LIFT01', 'AMR_LIFT02', 'Palletizer'])
    answer = p.map(start_demo, ['AMR_LIFT01','AMR_LIFT02'])
    p.close()
    p.join()

    rospy.spin()

