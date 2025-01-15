import math

class Gen_Relation():
    def __init__(self, robot_names, rack_names):

        node_info = {
            1: [132.90, 141.90],
            2: [132.90, 143.20],
            141: [141.45, 143.20],
            149: [138.90, 143.20],
            150: [134.40, 143.20],
            151: [138.90, 141.90],
            152: [134.40, 141.90]
        }


        self.node_info = node_info
        self.robot_names = robot_names
        self.rack_names = rack_names
        self.node_info = node_info
        self.enterExitThresholds = 1.6
        self.centerRadiusThresholds = 0.3
        self.goforward = 0.2
        self.gobackward = -0.2

    def delete_overlap_rel(self, rel_list):
        new_rel = dict()
        for key, value in rel_list.items():
            for r in value:
                if not key in new_rel.keys():
                    new_rel[key] = [r]
                elif r not in new_rel[key]:
                    new_rel.append(r)
        return new_rel


    def get_lift_objects_rel(self, robots, racks):
        armlift_obj_rel = dict()
        # armlift_obj_rel['L-Node'] = self.lift_node_rel(robots)
        armlift_obj_rel['L-Rack'] = self.lift_rack_rel(robots, racks)

        total_rel = self.delete_overlap_rel(armlift_obj_rel)

        # for k,v in armlift_obj_rel:
        #     print(v)
        return total_rel

    def lift_rack_rel(self, robots, racks):
        rel_list = list()
        for robot_id in self.robot_names:
            for rack_id in self.rack_names:
                robot = robots['ARMLIFT'][robot_id]
                rack = racks['Rack'][rack_id]

                enter_label = self.enter_exit(robot, rack)
                rel_list.append(enter_label)
                movingwith_label = self.movingWith(robot, rack)
                rel_list.append(movingwith_label)
        return rel_list


    def lift_node_rel(self, robots, node):
        return None

    def enter_exit(self, robot, rack):
        relation = [robot['name']]
        result = None
        robot_lifting = True if 'lift_up'in robot['lift_state'] else False

        sub_pose = self.get_pose(robot)
        obj_pose = self.get_pose(rack)
        x_diff = sub_pose['x'] - obj_pose['x']
        y_diff = sub_pose['y'] - obj_pose['y']
        x_diff_abs = abs(x_diff)
        y_diff_abs = abs(y_diff)
        eular_diff = sub_pose['euler'] - obj_pose['euler']
        linear_x = robot['odometry']['linear'][0]
        distance = self.cal_distance(robot, rack)


        if distance > self.enterExitThresholds and not robot_lifting:
            if 1.3 < sub_pose['euler'] < 1.7 and y_diff < 0 and x_diff_abs < 0.7:
                # print('3')
                if linear_x > self.goforward:
                    result = 'enteringInto'
                elif linear_x  < self.gobackward:
                    result = 'exitingFrom'
            elif -0.3 < sub_pose['euler'] < 0.3 and x_diff < 0 and y_diff_abs < 0.7:
                # print('2')
                if linear_x > self.goforward:
                    result = 'enteringInto'
                elif linear_x < self.gobackward:
                    result = 'exitingFrom'
            elif -1.7 < sub_pose['euler'] < -1.3 and y_diff > 0 and x_diff_abs < 0.7:
                # print('3')
                if linear_x > self.goforward:
                    result = 'enteringInto'
                elif linear_x < self.gobackward:
                    result = 'exitingFrom'
            elif -3.2 < sub_pose['euler'] < -2.7 and x_diff > 0 and y_diff_abs < 0.7:
                # print('4')
                if linear_x > self.goforward:
                    result = 'enteringInto'
                elif linear_x < self.gobackward:
                    result = 'exitingFrom'
        if result is not None:
            relation.append(rack['name'])
        else:
            relation.append('none')
        return relation


    def movingWith(self, robot, rack):
        relation = [robot['name']]
        robot_lifting = True if 'lift_up'in robot['lift_state'] else False
        sub_pose = self.get_pose(robot)
        obj_pose = self.get_pose(rack)
        x_diff_abs = abs(sub_pose['x'] - obj_pose['x'])
        y_diff_abs = abs(sub_pose['y'] - obj_pose['y'])
        distance = self.cal_distance(robot, rack)
        if self.isMoving(robot):
            if robot_lifting:
                if distance <= 0.3 and y_diff_abs < 1 and x_diff_abs < 1:
                    relation.append('movingWith')
                    relation.append(rack['name'])
            else:
                relation.append('movingWithOutRack')
        else:
            relation.append('none')
        return relation

    def cal_distance(self, sub, obj):
        sub_pose = self.get_pose(sub)
        obj_pose = self.get_pose(obj)
        return math.sqrt(math.pow(sub_pose['x'] - obj_pose['x'], 2)
                         + math.pow(sub_pose['y'] - obj_pose['y'], 2))

    def get_pose(self, item):
        pose_dict = dict()
        pose_dict['x'] = item['pose']['translation'][0]
        pose_dict['y'] = item['pose']['translation'][1]
        pose_dict['z'] = item['pose']['translation'][2]
        rot_x = item['pose']['rotation'][0]
        rot_y = item['pose']['rotation'][1]
        rot_z = item['pose']['rotation'][2]
        rot_w = item['pose']['rotation'][3]

        pose_dict['euler'] = self.euler_from_quaternion(rot_x, rot_y, rot_z, rot_w)[-1]
        return pose_dict

    def isMoving(self, robot):
        linear_x = robot['odometry']['linear'][0]
        angular_x = robot['odometry']['angular'][0]
        if linear_x > self.goforward or linear_x < self.gobackward:
            return True
        elif angular_x > 0.2 or angular_x < -0.2:
            return True
        return False

    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

