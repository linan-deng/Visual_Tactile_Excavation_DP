import rtde_control
import rtde_receive
import numpy as np
import math
import time

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

        self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.10")
        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.10")

        self.target_pos = np.zeros(3)
        self.target_theta = np.zeros(3)
        self.pose_control_delta_time = time.time()
        self.pose_control_time = time.time()
        self.pose_control_count = 0
        self.pose_control_flag = False

        self.save_file_time = time.time()
        self.save_file_flag = False

        self.tool_z = 0.2
        self.rtde_c.setTcp((0, 0, self.tool_z, 0, 0, 0))
        self.rtde_c.setPayload(1, (0, 0, 0))
        ### pure tactile excavation
        # self.pose_origin = [0.20689411887647957, -0.5180431565860071, 0.1591425555508029, 0.9249873883908998, -0.4011170347437063, 2.181502354533378]
        ### diffusion policy, 上述位置+robot.set_robot_pose_origin(pos=(0.05, 0.05, -0.06), theta=(0.0, 0.0, 0.0), speed=0.1, acceleration=0.1, asynchronous=False)
        self.pose_origin = [0.11689931502783926, -0.4962838365769675, 0.15343150900049257, 0.9247482767307493, -0.4006590590093375, 2.181395461842642]
        self.set_robot_pose_origin(asynchronous=False)
        # self.joint_positions_origin = self.rtde_c.getInverseKinematics(self.rtde_r.getActualTCPPose())
        self.joint_positions_origin = self.rtde_c.getActualJointPositionsHistory(0)
        print("self.joint_positions_origin:", self.joint_positions_origin)

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

    # input (3, ), output (3, )
    def Pos_w2b(self, pos=(0, 0, 0)):
        pos = np.array(pos).reshape(-1, 1)
        pos_base = self.R_b2w.T @ pos
        return pos_base.squeeze()

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

    # input: unit: m, rad
    # set pos relative to origin
    def set_robot_pose_origin(self, pos=(0.0, 0.0, 0.0), theta=(0.0, 0.0, 0.0), speed=0.1, acceleration=0.1, asynchronous=True):
        origin_pose = np.copy(self.pose_origin)
        # 工具坐标系中的位姿变化量
        delta_pose = [pos[0], pos[1], pos[2], theta[0], theta[1], theta[2]]
        # p_from: starting pose (spatial vector)，起始位姿
        # p_from_to: pose change relative to starting pose (spatial vector)，位姿变化量（工具坐标系）
        set_pose = self.rtde_c.poseTrans(p_from=origin_pose, p_from_to=delta_pose)
        self.rtde_c.moveL(pose=set_pose, speed=speed, acceleration=acceleration,
                          asynchronous=asynchronous)  # apply the new pose
        # print("self.rtde_r.getActualTCPPose():", self.rtde_r.getActualTCPPose())

    # pose relative to current pose
    def set_robot_pose_relative(self, pos=(0.0, 0.0, 0.0), theta=(0.0, 0.0, 0.0), speed=0.1, acceleration=0.1, asynchronous=True):
        current_pose = self.rtde_r.getActualTCPPose()
        # 工具坐标系中的位姿变化量
        delta_pose = [pos[0], pos[1], pos[2], theta[0], theta[1], theta[2]]
        # p_from: starting pose (spatial vector)，起始位姿
        # p_from_to: pose change relative to starting pose (spatial vector)，位姿变化量（工具坐标系）
        set_pose = self.rtde_c.poseTrans(p_from=current_pose, p_from_to=delta_pose)
        self.rtde_c.moveL(pose=set_pose, speed=speed, acceleration=acceleration, asynchronous=asynchronous)  # apply the new pose
        # print("self.rtde_r.getActualTCPPose():", self.rtde_r.getActualTCPPose())

    # 将旋转矢量转为欧拉角,旋转矢量方向为轴,模为旋转角度
    # input: (3,), output: (3,)
    def rotvec2rpy(self, pose):
        rx, ry, rz = pose
        theta = np.sqrt(rx * rx + ry * ry + rz * rz)
        kx = rx / theta
        ky = ry / theta
        kz = rz / theta
        cth = np.cos(theta)
        sth = np.sin(theta)
        vth = 1 - np.cos(theta)

        r11 = kx * kx * vth + cth
        r12 = kx * ky * vth - kz * sth
        r13 = kx * kz * vth + ky * sth
        r21 = kx * ky * vth + kz * sth
        r22 = ky * ky * vth + cth
        r23 = ky * kz * vth - kx * sth
        r31 = kx * kz * vth - ky * sth
        r32 = ky * kz * vth + kx * sth
        r33 = kz * kz * vth + cth
        beta = np.arctan2(-r31, np.sqrt(r11 * r11 + r21 * r21))
        # 这里需要判断分母为cos(beta)=0的情况
        if beta > np.deg2rad(89.99):
            beta = np.deg2rad(89.99)
            alpha = 0
            gamma = np.arctan2(r12, r22)
        elif beta < -np.deg2rad(89.99):
            beta = -np.deg2rad(89.99)
            alpha = 0
            gamma = -np.arctan2(r12, r22)
        else:
            cb = np.cos(beta)
            alpha = np.arctan2(r21 / cb, r11 / cb)
            gamma = np.arctan2(r32 / cb, r33 / cb)
        rpy = np.zeros(3)
        rpy[0] = gamma
        rpy[1] = beta
        rpy[2] = alpha
        return rpy

    # 将欧拉角转为旋转矢量
    # input: (3,), output: (3,)
    def rpy2rotvec(self, pose):
        gamma, beta, alpha = pose
        ca = np.cos(alpha)
        cb = np.cos(beta)
        cg = np.cos(gamma)
        sa = np.sin(alpha)
        sb = np.sin(beta)
        sg = np.sin(gamma)
        rotation_matrix = np.zeros((3, 3))
        rotation_matrix[0, 0] = ca * cb
        rotation_matrix[0, 1] = ca * sb * sg - sa * cg
        rotation_matrix[0, 2] = ca * sb * cg + sa * sg
        rotation_matrix[1, 0] = sa * cb
        rotation_matrix[1, 1] = sa * sb * sg + ca * cg
        rotation_matrix[1, 2] = sa * sb * cg - ca * sg
        rotation_matrix[2, 0] = -sb
        rotation_matrix[2, 1] = cb * sg
        rotation_matrix[2, 2] = cb * cg
        theta = np.arccos((rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2] - 1) / 2)
        sth = np.sin(theta)
        if sth == 0:
            return np.zeros(3)
        kx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (2 * sth)
        ky = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (2 * sth)
        kz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (2 * sth)
        rovetc = np.zeros(3)
        rovetc[0] = theta * kx
        rovetc[1] = theta * ky
        rovetc[2] = theta * kz
        return rovetc

    def set_target(self, pos=(0.0, 0.0, 0.0), theta=(0.0, 0.0, 0.0)):
        self.target_pos = pos
        self.target_theta = theta

    def speed_limit(self, data, speed_limit=1.0):
        data_abs_max = np.max(np.abs(data))
        if data_abs_max > speed_limit:
            data = data/data_abs_max * speed_limit
        return data

    def pose_control(self):
        delta_pose_old = np.zeros(6)
        delta_theta_old = 0.0
        robot_data_file = "./end_pose_control/" + str(time.time()) + "_robot_data.txt"
        while self.pose_control_flag:
            target_pose_w = [self.target_pos[0], self.target_pos[1], self.target_pos[2], self.target_theta[0], self.target_theta[1], self.target_theta[2]]
            current_pose = np.array(self.rtde_r.getActualTCPPose())
            target_theta = self.target_theta
            target_pose_w[3:] = self.rpy2rotvec(target_theta)  # 欧拉角rpy转旋转矢量r
            target_pose = np.array(self.rtde_c.poseTrans(p_from=self.pose_origin, p_from_to=target_pose_w))
            delta_pose = target_pose - current_pose
            # print("target_theta", target_theta)
            # print("target_pose_w", target_pose_w)
            # print("target_pose", target_pose)
            # print("current_pose", current_pose)
            # print("delta_pose", delta_pose)
            ################## 位置PID控制 ################
            kp_pos = 5.0
            kd_pos = 0.8
            target_speed = np.zeros(6)
            if np.sum(delta_pose_old) == 0:
                target_speed[:3] = kp_pos * delta_pose[:3]
            else:
                target_speed[:3] = kp_pos * delta_pose[:3] + \
                                   kd_pos * (delta_pose[:3]-delta_pose_old[:3])
            target_speed[:3] = self.speed_limit(target_speed[:3], speed_limit=1.0)
            ################# 姿态PID控制 ################
            kp_theta = 2.0
            kd_theta = 0.2
            # if np.sum(delta_pose_old) == 0:
            #     target_speed[3:] = kp_theta * delta_pose[3:]
            # else:
            #     target_speed[3:] = kp_theta * delta_pose[3:] + \
            #                        kd_theta * (delta_pose[3:]-delta_pose_old[3:])
            # print("target_speed:", target_speed)
            # #############################################

            # rtde_c.poseTrans生成的旋转角,无法保证末端执行器绕着tool_z轴旋转
            # current_theta = self.rtde_c.getInverseKinematics(current_pose)[5]
            current_theta = self.rtde_c.getActualJointPositionsHistory(0)[5]
            target_theta = self.joint_positions_origin[5] + self.target_theta[2]
            delta_theta = target_theta - current_theta
            print("self.joint_positions_origin:", self.joint_positions_origin)
            print("target_theta:", target_theta)
            print("delta_theta:", delta_theta)
            theta_direction_w = np.array([0, 0, 1])  # np.array([0.5, -0.5, 0.7071])
            theta_direction_b = self.Pos_w2b(theta_direction_w)

            if delta_theta == 0:
                target_speed[3:] = kp_theta * delta_theta * theta_direction_b
            else:
                target_speed[3:] = kp_theta * delta_theta * theta_direction_b + \
                                   kd_theta * (delta_theta - delta_theta_old) * theta_direction_b  #kp_theta * delta_pose[3:]
            print("target_speed:", target_speed)
            ############ xd: tool speed in base coordinate [m/s] (spatial vector)
            delta_theta_old = delta_theta
            delta_pose_old = delta_pose
            self.rtde_c.speedL(xd=target_speed[:], acceleration=0.5, time=0.0)

            ################## save robot data ######################
            save_data = np.zeros(1+6+6+6+3)
            save_data[0] = time.time()
            save_data[1:7] = target_pose
            save_data[7:13] = current_pose
            save_data[13:19] = delta_pose
            # save_data[19:20] = target_theta
            # save_data[20:21] = current_theta
            # save_data[21:22] = delta_theta
            with open(robot_data_file, 'a') as f:
                np.savetxt(f, save_data.reshape(1, -1), delimiter=",")
                f.close()
            #########################################################

            delta_time = time.time() - self.pose_control_delta_time
            self.pose_control_delta_time = time.time()
            self.pose_control_count += 1
            #### pose control 100 Hz
            if delta_time < 0.01:
                time.sleep(0.01-delta_time)

            if self.pose_control_count % 100 == 0:
                print("pose_control_freq:", 100/(time.time()-self.pose_control_time))
                self.pose_control_time = time.time()

    # 推荐使用关节速度控制，更加稳定丝滑
    def pose_control_joints(self):
        delta_joint_positions_old = np.zeros(6)
        while self.pose_control_flag:

            current_pose = self.rtde_r.getActualTCPPose()
            target_pose_w = np.zeros(6)
            target_pose_w[:3] = self.target_pos[:]
            target_theta = self.target_theta
            target_pose_w[3:] = self.rpy2rotvec(target_theta)  # 欧拉角rpy转旋转矢量r
            target_pose = np.array(self.rtde_c.poseTrans(p_from=self.pose_origin, p_from_to=target_pose_w))

            save_current_pose = np.copy(current_pose)
            save_current_pose[3:] = self.rotvec2rpy(save_current_pose[3:])
            save_target_pose = np.copy(target_pose)
            save_target_pose[3:] = self.rotvec2rpy(save_target_pose[3:])
            save_delta_pose = save_target_pose - save_current_pose

            current_joint_positions = np.array(self.rtde_c.getInverseKinematics(current_pose))
            target_joint_positions = np.array(self.rtde_c.getInverseKinematics(target_pose))
            delta_joint_positions = target_joint_positions - current_joint_positions
            # print("current_joint_positions:", current_joint_positions)
            # print("target_joint_positions:", target_joint_positions)
            # current_joint_speed = np.array(self.rtde_c.getInverseKinematics(current_speed))
            # print("current_joint_speed:", current_joint_speed)
            kp_pos = 3.0
            kd_pos = 0.3
            if np.sum(delta_joint_positions_old) == 0:
                target_joint_speed = kp_pos * delta_joint_positions
            else:
                target_joint_speed = kp_pos * delta_joint_positions + \
                                   kd_pos * (delta_joint_positions - delta_joint_positions_old)
            # target_joint_speed = self.speed_limit(target_joint_speed, speed_limit=1.0)
            self.rtde_c.speedJ(qd=target_joint_speed, acceleration=1.0, time=1.0/100)
            delta_joint_positions_old = delta_joint_positions

            ################## save robot data ######################
            # print("self.save_file_flag:", self.save_file_flag)
            if self.save_file_flag == True:
                robot_data_file = "./end_pose_control/" + str(self.save_file_time) + "_robot_data.txt"
                save_data = np.zeros(1+6+6+6)
                save_data[0] = time.time()
                save_data[1:7] = save_target_pose  # 目标姿态
                save_data[7:13] = save_current_pose  # 当前姿态
                save_data[13:19] = save_delta_pose  # 姿态差值

                with open(robot_data_file, 'a') as f:
                    np.savetxt(f, save_data.reshape(1, -1), delimiter=",")
                    f.close()
            #########################################################

            delta_time = time.time() - self.pose_control_delta_time
            self.pose_control_delta_time = time.time()
            self.pose_control_count += 1
            #### pose control 100 Hz
            if delta_time < 0.01:
                time.sleep(0.01-delta_time)

            if self.pose_control_count % 10 == 0:
                print("pose_control_freq:", 10/(time.time()-self.pose_control_time))
                self.pose_control_time = time.time()

if __name__ == "__main__":
    from gripper_control import GripperControl
    robot = RobotControl()
    # gripper = GripperControl()
    time.sleep(1)
    print("current pose:", robot.rtde_r.getActualTCPPose())
    robot.set_robot_pose_origin(pos=(0.0, 0.0, 0.0), theta=(0.0, 0.0, 0.0), speed=0.1, acceleration=0.1, asynchronous=False)  # 最大指尖力测试的机器人位置
    print("current pose:", robot.rtde_r.getActualTCPPose())