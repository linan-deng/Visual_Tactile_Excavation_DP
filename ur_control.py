import copy
import urx
import time
import math
import logging
import numpy as np

class RobotControl:
    def __init__(self):
        self.l = 0.05  # unit: m
        self.r = np.pi / 6
        # ~1000Hz
        theta_x = 0.0 / 180.0 * np.pi
        theta_y = 45.0 / 180.0 * np.pi
        theta_z = -135.0 / 180.0 * np.pi
        euler = np.array([theta_x, theta_y, theta_z]).reshape(-1, 1)
        self.R_b2w = self.Euler2R(euler)
        # print("self.R_b2w:", self.R_b2w)

        self.robot = urx.Robot('192.168.1.10')
        self.tool_z = 0.2
        self.robot.set_tcp((0, 0, self.tool_z, 0, 0, 0))
        self.robot.set_payload(1, (0, 0, 0))
        self.origin_pos_w = np.array([-0.25, 0.18+0.04, 0.235 + self.tool_z + 0.04]) #0.095, 0.045 tactile excavation # origin in world  # 0.28 is the table surface
        self.set_robot_pos_origin()
        self.tran_origin = self.robot.get_pose()
        print("self.tran_origin:", self.tran_origin)

    # euler angle euler: 3x1
    def Euler2R(self, euler=(0, 0, 0)):
        euler = np.array(euler)
        theta_x = euler[0]
        theta_y = euler[1]
        theta_z = euler[2]
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(theta_x), -np.sin(theta_x)],
                       [0, np.sin(theta_x), np.cos(theta_x)]], dtype=object)
        Ry = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],
                       [0, 1, 0],
                       [-np.sin(theta_y), 0, np.cos(theta_y)]], dtype=object)
        Rz = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                       [np.sin(theta_z), np.cos(theta_z), 0],
                       [0, 0, 1]], dtype=object)
        Rxyz = np.dot(Rx, np.dot(Ry, Rz))
        return np.array(Rxyz)

    # rotation matrix R: 3x3
    def R2Euler(self, R):
        sq = math.sqrt(R[0][0] * R[0][0] + R[0][1] * R[0][1])
        singular = sq < 1e-6
        if not singular:
            x = math.atan2(-R[1][2], R[2][2])
            y = math.atan2(R[0][2], sq)
            z = math.atan2(-R[0][1], R[0][0])
        return np.array([x, y, z]).squeeze()

    # input (3, 1), output (3, 1)
    def Pos_w2b(self, pos=(0, 0, 0)):
        pos = np.array(pos).reshape(-1, 1)
        pos_base = np.dot(self.R_b2w.T, pos).squeeze()
        return pos_base

    # input (3, 1), output (3, 1)
    def Pos_b2w(self, pos=(0, 0, 0)):
        pos = np.array(pos).reshape(-1, 1)
        pos_world = np.dot(self.R_b2w, pos)
        return pos_world

    # input (3, 3), output (3, 3)
    def R_t2b(self, R_t2w):
        R_t2b = np.dot(self.R_b2w.T, R_t2w)
        return R_t2b

    # input (3, 3), output (3, 3)
    def R_t2w(self, R_t2b):
        R_t2w = np.dot(self.R_b2w, R_t2b)
        return R_t2w

    # set pos relative to origin
    def set_robot_pos_origin(self, pos=(0.0, 0.0, 0.0), acc=0.1, vel=0.1):
        # print(self.robot.getl())
        # print(self.robot.get_pose())
        set_pose = [0, 0, 0, 0, 0, 0]
        # self.origin_pose = [0.9249702014077881, -0.4012653001207751, 2.1816249147513105]  # pose in base
        self.origin_pose = [0.9249702014077881, -0.4012653001207751, 2.1816249147513105]  # pose in base
        # self.origin_pos_w = np.array([-0.3, 0.18, 0.28+self.tool_z])  # origin in world
        set_pos = np.array([self.origin_pos_w[0]+pos[0], self.origin_pos_w[1]+pos[1],
                            self.origin_pos_w[2]+pos[2]]).reshape(-1, 1)
        origin_pos_b = self.Pos_w2b(set_pos)  # transform world to base
        set_pose[0] = origin_pos_b[0][0]
        set_pose[1] = origin_pos_b[1][0]
        set_pose[2] = origin_pos_b[2][0]
        set_pose[3] = self.origin_pose[0]
        set_pose[4] = self.origin_pose[1]
        set_pose[5] = self.origin_pose[2]
        self.robot.movel(set_pose, acc=acc, vel=vel)
        time.sleep(0.1)
        # print(self.robot.getl())
        # print(self.robot.get_pose())

    # input: unit: m, rad
    # set pos relative to origin
    def set_robot_pose_origin(self, pos=(0.0, 0.0, 0.0), theta=(0.0, 0.0, 0.0), acc=0.1, vel=0.1):
        trans = copy.copy(self.tran_origin)
        print("trans:", trans)
        # print("trans.orient:", trans.orient)
        pos_target = self.Pos_w2b(pos=(pos[0], pos[1], pos[2]))  # world to base
        trans.pos.x += pos_target[0][0]  # unit: m
        trans.pos.y += pos_target[1][0]
        trans.pos.z += pos_target[2][0]
        trans.orient.rotate_x(theta[0])  # unit: rad
        trans.orient.rotate_y(theta[1])
        trans.orient.rotate_z(theta[2])
        # print("trans.orient:", trans.orient)
        self.robot.movel(trans, acc=acc, vel=vel)  # apply the new pose

    # pose relative to current pose
    def set_robot_pose_relative(self, pos=(0.0, 0.0, 0.0), theta=(0.0, 0.0, 0.0), acc=0.1, vel=0.1):
        trans = self.robot.get_pose()  # = robot.getl() get current transformation matrix (tool to base)
        print("111111111111111robot.getl():", self.robot.getl())
        # print("111111111111111robot.get_pose():", self.robot.get_pose())
        # print("111111111111111robot.get_pose():", self.R2Euler(self.robot.get_pose().orient))
        pos_target = self.Pos_w2b(pos)  # world to base
        print(pos, pos_target)
        trans.pos.x += pos_target[0] # unit: m
        trans.pos.y += pos_target[1]
        trans.pos.z += pos_target[2]
        trans.orient.rotate_x(theta[0])  # unit: rad
        trans.orient.rotate_y(theta[1])
        trans.orient.rotate_z(theta[2])
        self.robot.movel(trans, acc=acc, vel=vel)  # apply the new pose
        print("22222222222222robot.getl():", self.robot.getl())
        # print("22222222222222robot.get_pose():", self.robot.get_pose())
        # print("22222222222222robot.self.Euler2R(self.origin_pos):", self.Euler2R(self.origin_pos))
        # self.robot.set_pose(trans, acc=acc, vel=vel)  # (default command="movel") apply the new pose

    # return 3x1
    def get_robot_pos_origin(self):
        self.pos_b = np.array(self.robot.getl()[0:3]).squeeze()
        # print("self.pos_b:", self.pos_b)
        self.pos_w = self.Pos_b2w(self.pos_b).squeeze()
        # print("self.pos_w:", self.pos_w)
        self.pos_relative = self.pos_w - self.origin_pos_w
        # print("self.pos_relative:", self.pos_relative)
        # print("self.pos_b.shape, self.pos_w, self.origin_pos_w, self.pos_relative:",
        #       self.pos_b.shape, self.pos_w, self.origin_pos_w, self.pos_relative)
        return self.pos_relative

if __name__ == "__main__":
    from gripper_control import GripperControl
    robot = RobotControl()
    # gripper = GripperControl()
    time.sleep(1)
    # print(robot.robot.getl())
    # print("robot.getl():", robot.robot.getl())
    # print("robot.get_robot_pos_origin():", robot.get_robot_pos_origin())
    # gripper.set_pos([20, 20, 0, 90])
    # gripper.set_pos([0, 0, 0, 0])
    # z_theta = np.array([90.0 / 180.0 * np.pi])
    robot.set_robot_pose_origin(pos=(0., 0., -0.06), theta=(0.0, 0.0, 0.0), acc=0.1, vel=0.1)  # 最大指尖力测试的机器人位置

    ################### stiffness test ####################
    # gripper.set_pos([20, 20, 0, 0])
    # time.sleep(1)
    # robot.set_robot_pose_origin((0, 0.01, 0), theta=(0.0, 0.0, 0.0), acc=0.01, vel=0.01)
    # robot.set_robot_pose_origin((0, 0, 0), theta=(0.0, 0.0, 0.0), acc=0.01, vel=0.01)
    # time.sleep(5)
    # robot.set_robot_pose_origin()
    # gripper.set_pos([0, 0, 0, 0])
    #######################################################

    # grasp_pose_path = "visualization_realtime/grasp_pose.txt"
    # data = []
    # with open(grasp_pose_path, "r") as f:
    #     for line in f.readlines():
    #         line1 = line.strip("\n")
    #         line1 = line1.split(',')
    #         data.append(line1)
    #     f.close()
    # data = np.array(data, dtype=np.float32)
    #
    # z_theta = np.pi/2
    # # Rz = np.array([[np.cos(z_theta), -np.sin(z_theta), 0],
    # #                [np.sin(z_theta), np.cos(z_theta), 0],
    # #                [0, 0, 1]])
    # R = data[:3, :]
    # pos_center = data[3, :]
    # eigenvalue = data[4, :]
    # thetas = robot.R2Euler(R)
    # thetas[0] = 0
    # thetas[1] = 0
    # print("thetas:", thetas)
    # thetas_grasp = copy.copy(thetas)
    # thetas_grasp[0] = 0
    # thetas_grasp[1] = 0
    # thetas_grasp[2] -= np.pi/2
    # print("thetas_grasp:", thetas_grasp)
    # gripper.set_pos([0, 50, 0, 50])
    # robot.set_robot_pose_origin((pos_center[0], pos_center[1], 0))
    # robot.set_robot_pose_origin((pos_center[0], pos_center[1], 0), theta=thetas)
    # robot.set_robot_pose_origin((pos_center[0], pos_center[1], pos_center[2]), theta=thetas)
    # time.sleep(1)
    # gripper.set_pos([90, 0, 90, 0])
    # time.sleep(1)
    # gripper.set_pos([90, 90, 90, 90])
    # robot.set_robot_pose_origin((pos_center[0], pos_center[1], 0), theta=thetas)
    # robot.set_robot_pose_origin((0, 0, 0))
    # time.sleep(1)
    # gripper.set_pos([0, 0, 0, 0])

    ###################### rake the granular materials ########################
    # p1_0 = pos_center - 4 * R[:, 1]*eigenvalue[1]
    # p1_1 = p1_0 + 4 * R[:, 0]*eigenvalue[0]
    # p1_2 = p1_0 - 4 * R[:, 0]*eigenvalue[0]
    # robot.set_robot_pose_origin((p1_1[0], p1_1[1], 0), thetas)
    # gripper.set_pos([0, 90, 20, 30])
    # robot.set_robot_pose_origin((p1_1[0], p1_1[1], p1_1[2]+0.01), thetas)
    # robot.set_robot_pose_origin((p1_2[0], p1_2[1], p1_2[2]+0.01), thetas)
    # gripper.set_pos([0, 90, 60, 80])
    # time.sleep(0.5)
    # robot.set_robot_pose_origin((p1_1[0], p1_1[1], 0), thetas)
    # gripper.set_pos([0, 90, 20, 30])
    # robot.set_robot_pose_origin((p1_1[0], p1_1[1], p1_1[2]+0.015), thetas)
    # robot.set_robot_pose_origin((p1_2[0], p1_2[1], p1_2[2]+0.015), thetas)
    # gripper.set_pos([0, 90, 60, 80])
    # time.sleep(0.5)
    # robot.set_robot_pose_origin((p1_1[0], p1_1[1], 0), thetas)
    # gripper.set_pos([0, 90, 20, 30])
    # robot.set_robot_pose_origin((p1_1[0], p1_1[1], p1_1[2]+0.015), thetas)
    # robot.set_robot_pose_origin((p1_2[0], p1_2[1], p1_2[2]+0.015), thetas)
    # gripper.set_pos([0, 90, 60, 80])
    # time.sleep(0.5)
    # # ######################## rake another side #########################
    # p2_0 = pos_center + 4 * R[:, 1]*eigenvalue[1]
    # p2_1 = p2_0 + 4 * R[:, 0]*eigenvalue[0]
    # p2_2 = p2_0 - 4 * R[:, 0]*eigenvalue[0]
    # robot.set_robot_pose_origin((p2_1[0], p2_1[1], 0), thetas)
    # gripper.set_pos([0, 90, 20, 30])
    # robot.set_robot_pose_origin((p2_1[0], p2_1[1], p2_1[2]+0.01), thetas)
    # robot.set_robot_pose_origin((p2_2[0], p2_2[1], p2_2[2]+0.01), thetas)
    # gripper.set_pos([0, 90, 60, 80])
    # time.sleep(0.5)
    # robot.set_robot_pose_origin((p2_1[0], p2_1[1], 0), thetas)
    # gripper.set_pos([0, 90, 20, 30])
    # robot.set_robot_pose_origin((p2_1[0], p2_1[1], p2_1[2]+0.015), thetas)
    # robot.set_robot_pose_origin((p2_2[0], p2_2[1], p2_2[2]+0.015), thetas)
    # gripper.set_pos([0, 90, 60, 80])
    # time.sleep(0.5)
    # robot.set_robot_pose_origin((p2_1[0], p2_1[1], 0), thetas)
    # gripper.set_pos([0, 90, 20, 30])
    # robot.set_robot_pose_origin((p2_1[0], p2_1[1], p2_1[2]+0.015), thetas)
    # robot.set_robot_pose_origin((p2_2[0], p2_2[1], p2_2[2]+0.015), thetas)
    # gripper.set_pos([0, 90, 60, 80])
    # time.sleep(0.5)

    # ####################### grasp object ######################
    # robot.set_robot_pose_origin((pos_center[0], pos_center[1], 0),
    #                             (thetas[0], thetas[1], thetas[2]+np.pi/2))
    # gripper.set_pos([0, 0, 0, 0])
    # robot.set_robot_pose_origin((pos_center[0], pos_center[1], pos_center[2]),
    #                             (thetas[0], thetas[1], thetas[2]+np.pi/2))
    # gripper.set_pos([50, 0, 50, 50])
    # time.sleep(0.5)
    # gripper.set_pos([90, 70, 90, 70])
    # time.sleep(0.5)
    # robot.set_robot_pose_origin((pos_center[0], pos_center[1], 0),
    #                             (thetas[0], thetas[1], thetas[2]+np.pi/2))
    # time.sleep(0.5)
    # robot.set_robot_pos_origin()
    # gripper.set_pos([0, 0, 0, 0])