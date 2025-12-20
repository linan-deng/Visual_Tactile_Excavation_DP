import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton, QLineEdit, QHBoxLayout, QVBoxLayout, QWidget, QFileDialog, QGridLayout
from PyQt5.QtGui import QPixmap, QImage, QColor, QMouseEvent
from PyQt5 import QtGui
from PyQt5.QtCore import Qt, QTimer
from camera_read_two import CameraRead
from skvideo.io import vwrite
import scipy.signal as signal
import threading
import time
import keyboard
import nidaqmx
from ur_control_rtde import RobotControl
from gripper_control import GripperControl
from imu_read import IMURead

class ImagePixelReader(QWidget):
    def __init__(self):
        super(ImagePixelReader, self).__init__()
        # 设置窗口的标题和初始大小
        self.setWindowTitle("Human-in-the-loop labeling software")
        self.setWindowIcon(QtGui.QIcon("logo/HUST.png"))
        self.setGeometry(20, 50, 1400, 800)
        self.camera = CameraRead()
        self.robot = RobotControl()
        self.gripper = GripperControl()
        self.imu = IMURead()

        self.delta_pos_camera = np.zeros(3)
        self.delta_theta_camera = np.zeros(3)  # unit: rad
        self.pos_world = np.zeros(3)
        self.theta_world = np.zeros(3)  # unit: rad
        self.grasp_pos = np.zeros(4, int)

        ############### imu sensor data read parameters ###########
        self.imuReadTime = 0
        self.imu_data = np.zeros(34)
        self.imu_data_old = np.zeros(34)
        self.imu_data_n = np.zeros((30, 34))  # first n imu data

        ##################### daq tactile data read parameters #################
        self.tactile_frame = np.zeros((16, 16))
        self.tactile_frame_temp = np.zeros((16, 17))
        self.tactile_frame_temp_10 = np.zeros((16, 17, 500))
        self.tactile_frame_10 = np.zeros((20, 16, 16)) # init
        self.tactile_frame_count = 0
        self.tactile_init = 0

        ### imu 频率
        self.imu_fps_time = time.time()
        self.imu_fps_count = 0

        ### tactile 频率
        self.tactile_fps_time = time.time()
        self.tactile_fps_count = 0

        ### 相机帧率
        self.camera_fps_time = time.time()
        self.camera_fps_count = 0

        #存储双目视频
        # self.frame1s = []
        # self.frame2s = []
        self.frames = []

        self.start_once_flag = True
        self.save_file_flag = False  # 是否存储所有数据
        self.imuFlag = True  # imu数据是否达到一定数目
        self.tactile_initialized_flag = False  # tactile初始化完成标志

        self.draw_points_count = 0
        self.draw_points_camera = np.zeros((2, 3))  # 3d points
        self.draw_points_xy_pixel = np.zeros((2, 2))  # pix coordinates

        # set UI
        self.set_widgets()
        self.set_layout()
        self.set_timer()


    def set_widgets(self):

        # 左摄像头显示
        self.frame1_label = QLabel(self)
        frame = np.zeros((640, 480, 3))
        qimg = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
        qpix = QPixmap.fromImage(qimg)
        self.frame1_label.setPixmap(qpix)
        self.frame1_label.setAlignment(Qt.AlignTop)

        # 右摄像头显示
        self.frame2_label = QLabel(self)
        qimg = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
        qpix = QPixmap.fromImage(qimg)
        self.frame2_label.setPixmap(qpix)
        self.frame2_label.setAlignment(Qt.AlignTop)

        # self.depth_label = QLabel(self)
        # qimg = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
        # qpix = QPixmap.fromImage(qimg)
        # self.depth_label.setPixmap(qpix)
        # self.depth_label.setAlignment(Qt.AlignTop)

        button_width = 100
        button_height = 100
        # 任务开始按钮
        self.startBtn = QPushButton('Start')
        self.startBtn.setFixedSize(int(button_width*3/2), button_height)
        self.startBtn.clicked.connect(self.start_timer)

        #任务结束按钮
        self.endBtn = QPushButton('End')
        self.endBtn.setFixedSize(int(button_width*3/2), button_height)
        self.endBtn.clicked.connect(self.end_timer)

        # 图像坐标系x轴正向移动
        self.move_x_positive_button = QPushButton('+x', self)
        self.move_x_positive_button.setFixedSize(button_width, button_height)
        self.move_x_positive_button.clicked.connect(self.move_x_positive_camera)

        # 图像坐标系x轴负向移动
        self.move_x_negative_button = QPushButton('-x', self)
        self.move_x_negative_button.setFixedSize(button_width, button_height)
        self.move_x_negative_button.clicked.connect(self.move_x_negative_camera)

        # 图像坐标系y轴正向移动
        self.move_y_positive_button = QPushButton('+y', self)
        self.move_y_positive_button.setFixedSize(button_width, button_height)
        self.move_y_positive_button.clicked.connect(self.move_y_positive_camera)

        # 图像坐标系y轴负向移动
        self.move_y_negative_button = QPushButton('-y', self)
        self.move_y_negative_button.setFixedSize(button_width, button_height)
        self.move_y_negative_button.clicked.connect(self.move_y_negative_camera)

        # 图像坐标系z轴正向移动
        self.move_z_positive_button = QPushButton('+z', self)
        self.move_z_positive_button.setFixedSize(button_width, button_height)
        self.move_z_positive_button.clicked.connect(self.move_z_positive_camera)

        # 图像坐标系z轴负向移动
        self.move_z_negative_button = QPushButton('-z', self)
        self.move_z_negative_button.setFixedSize(button_width, button_height)
        self.move_z_negative_button.clicked.connect(self.move_z_negative_camera)

        # 图像坐标系z轴正向旋转
        self.rotate_z_positive_button = QPushButton('+rz', self)
        self.rotate_z_positive_button.setFixedSize(button_width, button_height)
        self.rotate_z_positive_button.clicked.connect(self.rotate_z_positive_camera)

        # 图像坐标系z轴负向旋转
        self.rotate_z_negative_button = QPushButton('-rz', self)
        self.rotate_z_negative_button.setFixedSize(button_width, button_height)
        self.rotate_z_negative_button.clicked.connect(self.rotate_z_negative_camera)

        # 抓取
        self.grasp_button = QPushButton('grasp', self)
        self.grasp_button.setFixedSize(int(button_width*3/2), button_height)
        self.grasp_button.clicked.connect(self.grasp)

        # 释放
        self.release_button = QPushButton('release\ngrasp', self)
        self.release_button.setFixedSize(int(button_width*3/2), button_height)
        self.release_button.clicked.connect(self.release)

        # 耙扫
        self.rake_button = QPushButton('rake', self)
        self.rake_button.setFixedSize(int(button_width*3/2), button_height)
        self.rake_button.clicked.connect(self.rake)

        # 快速释放
        self.release_rake_button = QPushButton('release\nrake', self)
        self.release_rake_button.setFixedSize(int(button_width*3/2), button_height)
        self.release_rake_button.clicked.connect(self.release_rake)

        # control button
        font = QtGui.QFont()
        font.setFamily('Arial')
        font.setBold(True)
        font.setPointSize(20)
        font.setWeight(50)
        self.startBtn.setFont(font)
        self.endBtn.setFont(font)
        self.move_x_negative_button.setFont(font)
        self.move_x_positive_button.setFont(font)
        self.move_y_negative_button.setFont(font)
        self.move_y_positive_button.setFont(font)
        self.move_z_negative_button.setFont(font)
        self.move_z_positive_button.setFont(font)
        self.rotate_z_negative_button.setFont(font)
        self.rotate_z_positive_button.setFont(font)
        self.grasp_button.setFont(font)
        self.release_button.setFont(font)
        self.rake_button.setFont(font)
        self.release_rake_button.setFont(font)

    def set_layout(self):
        # 布局设置
        layoutCamera = QHBoxLayout()  # 水平布局
        layoutCamera.addWidget(self.frame1_label)
        layoutCamera.addWidget(self.frame2_label)
        # layoutCamera.addWidget(self.depth_label)
        layoutCamera.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        layoutTaskControl = QHBoxLayout()
        layoutTaskControl.addWidget(self.startBtn)
        layoutTaskControl.addWidget(self.endBtn)

        layoutXYControl1 = QHBoxLayout()
        layoutXYControl1.addWidget(self.move_y_negative_button)
        layoutXYControl2 = QHBoxLayout()
        layoutXYControl2.addWidget(self.move_x_negative_button)
        layoutXYControl2.addWidget(self.move_y_positive_button)
        layoutXYControl2.addWidget(self.move_x_positive_button)
        layoutXYControl = QVBoxLayout()
        layoutXYControl.addLayout(layoutXYControl1)
        layoutXYControl.addLayout(layoutXYControl2)

        layoutZRControl1 = QHBoxLayout()
        layoutZRControl1.addWidget(self.move_z_negative_button)
        layoutZRControl2 = QHBoxLayout()
        layoutZRControl2.addWidget(self.rotate_z_negative_button)
        layoutZRControl2.addWidget(self.move_z_positive_button)
        layoutZRControl2.addWidget(self.rotate_z_positive_button)
        layoutZRControl = QVBoxLayout()
        layoutZRControl.addLayout(layoutZRControl1)
        layoutZRControl.addLayout(layoutZRControl2)

        layoutGripperControl1 = QHBoxLayout()
        layoutGripperControl1.addWidget(self.grasp_button)
        layoutGripperControl1.addWidget(self.release_button)
        layoutGripperControl2 = QHBoxLayout()
        layoutGripperControl2.addWidget(self.rake_button)
        layoutGripperControl2.addWidget(self.release_rake_button)
        layoutGripperControl = QVBoxLayout()
        layoutGripperControl.addLayout(layoutGripperControl1)
        layoutGripperControl.addLayout(layoutGripperControl2)

        layoutControl = QVBoxLayout()
        layoutControl.addLayout(layoutXYControl)
        layoutControl.addLayout(layoutZRControl)
        layoutControl.addLayout(layoutGripperControl)
        layoutControl.addLayout(layoutTaskControl)
        layoutControl.setAlignment(Qt.AlignCenter | Qt.AlignTop)

        self.layout = QHBoxLayout()
        self.layout.addLayout(layoutCamera)
        self.layout.addLayout(layoutControl)
        self.setLayout(self.layout)

    def set_timer(self):

        self.read_camera_Timer = QTimer(self)
        self.read_camera_Timer.timeout.connect(self.read_and_show_image)

        self.keyboard_listening_Timer = QTimer(self)
        self.keyboard_listening_Timer.timeout.connect(self.listen_key)

        self.imu_read_data_Timer = QTimer(self)
        self.imu_read_data_Timer.timeout.connect(self.imu.read_data_single)

        self.imu_read_Timer = QTimer(self)
        self.imu_read_Timer.timeout.connect(self.imu_read)

        self.daq_read_Timer = QTimer(self)
        self.daq_read_Timer.timeout.connect(self.daq_read)

    def start_timer(self):
        # 设置开始按钮不可点击，结束按钮可点击
        self.startBtn.setEnabled(False)
        self.endBtn.setEnabled(True)
        self.grasp_pos[:] = 0
        self.gripper.set_pos(self.grasp_pos, step_time=0.07)
        # self.startBtn.setStyleSheet("background-color: color:rgb(0,0,0,0)")

        if self.start_once_flag:

            # 双目摄像头
            self.read_camera_Timer.start(30)
            print("camera start !!!")

            # self.keyboard_listening_Timer.start(100)
            # 创建一个新的线程，并启动它来监听特定的按键
            self.thread_keyboard = threading.Thread(target=self.listen_key)
            self.thread_keyboard.start()
            print("thread_keyboard start !!!")

            # 创建一个新的线程，用于机械臂末端位姿控制
            self.robot.pose_control_flag = True
            self.thread_pose_control = threading.Thread(target=self.robot.pose_control_joints)
            self.thread_pose_control.start()
            print("thread_pose_control start !!!")

            # imu
            self.imu_read_data_Timer.start(1)
            self.imu_read_Timer.start(1)
            print("imu start !!!")

            # daq不宜重复声明
            self.set_daq()
            print("daq set !!!")
            self.daq_read_Timer.start(1)
            # self.thread_daq = threading.Thread(target=self.daq_read)
            # self.thread_daq.start()
            print("daq start !!!")

            self.start_once_flag = False  # 任务只开启一次
            self.save_file_flag = False  # 第一次等tactile初始化后再存储数据
        else:
            self.save_file_flag = True

        # 保存数据
        self.file_time = time.time()
        self.robot.save_file_time = self.file_time
        self.robot.save_file_flag = True
        self.gripper_data_file = "./end_pose_control/" + str(self.file_time) + "_gripper_data.txt"
        self.imu_data_file = "./end_pose_control/" + str(self.file_time) + "_imu_data.txt"
        self.tactile_data_file = "./end_pose_control/" + str(self.file_time) + "_tactile_data.txt"

        print("all start !!!")

    def end_timer(self):
        self.save_file_flag = False
        self.robot.save_file_flag = self.save_file_flag
        self.startBtn.setEnabled(True)
        self.endBtn.setEnabled(False)

        self.grasp_pos[:] = 0
        self.gripper.set_pos(self.grasp_pos, step_time=0.07)
        # self.endBtn.setStyleSheet("background-color: color:rgb(0,0,0,0)")

        # visualize
        # 可视化
        from IPython.display import Video
        # vwrite("./end_pose_control/" + str(self.file_time)+'_frame1.mp4', self.frame1s)
        # vwrite("./end_pose_control/" + str(self.file_time)+'_frame2.mp4', self.frame2s)
        vwrite("./end_pose_control/" + str(self.file_time)+'_frame_merge.mp4', self.frames)
        print("video save finished !!!")
        # self.frame1s = []
        # self.frame2s = []
        self.frames = []

    def imu_read(self):
        # print("imu reading ...")
        if self.imuFlag:  # daq初始化完毕
            # self.imuCount += 1
            self.imu_data = self.imu.imu_data
            if self.imu_data[0] != self.imu_data_old[0]:
                if self.save_file_flag:
                    ################## save imu data #################
                    with open(self.imu_data_file, "a") as f:
                        imu_data_save = np.concatenate((np.array(time.time()).reshape(1, -1), np.array(self.imu_data).reshape(1, -1)), axis=1)
                        np.savetxt(f, imu_data_save)
                        f.close()

                self.imu_fps_count += 1
                if self.imu_fps_count % 100 == 0:
                    fps = 100 / (time.time() - self.imu_fps_time)
                    print("imu fps:", fps)
                    self.imu_fps_time = time.time()

            self.imu_data_old = self.imu_data

    def set_daq(self):
        # The task is automatically closed when exiting the 'with' block
        print("11111111111111111")
        self.daqTask = nidaqmx.Task()
        print("2222222222222222")
        # Configure and start your task here
        maxVoltage = 10
        minVoltage = -10
        sampleRate = 5000
        samplesPerChannel = 100000
        dev_name = "PXI2Slot4/"
        print("dev_name:", dev_name)
        for i in range(21):
            self.daqTask.ai_channels.add_ai_voltage_chan(dev_name + 'ai' + str(i), min_val=minVoltage, max_val=maxVoltage,
                                                  terminal_config=nidaqmx.constants.TerminalConfiguration.DIFF)
        # 每个通道读取多组，提高读取速度
        self.daqTask.timing.cfg_samp_clk_timing(sampleRate, sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS, samps_per_chan=samplesPerChannel)
        # spin off call to check
        self.daqTask.start()

    def daq_read(self):
        # print("daq reading ...")
        # Check if task needs to update the graph
        samplesAvailable = self.daqTask._in_stream.avail_samp_per_chan
        vals = self.daqTask.read(samplesAvailable)
        vals = np.array(vals)
        samples_num = vals.shape[1]  # 每次daq读取的样本数，vals的形状为：20x样本数
        vals = vals.T  # 样本数x20
        # print("vals.shape:", vals.shape)
        if samples_num > 0:
            for i in range(samples_num):
                if not ((0.15 < vals[i, 0:4]) & (vals[i, 0:4] < 3.15)).any():
                    arr_num = np.int64(vals[i, 0:4] > 3.1)  # 前四个通道为二进制的行号
                    row_num = arr_num[0] + arr_num[1]*2 + arr_num[2]*4 + arr_num[3]*8  # 二进制转化为十进制
                    # print("vals[i, 20]:", vals[i, 20])
                    row_flag = np.int64(vals[i, 20] > 3.1) | arr_num[3]  # 判断是选择第一行or开关关闭状态
                    if row_flag:
                        if int(self.tactile_frame_temp_10[row_num, 0, 0] < self.tactile_frame_temp_10.shape[2]):
                            self.tactile_frame_temp_10[row_num, 1:, int(self.tactile_frame_temp_10[row_num, 0, 0])] = vals[i, 4:20].copy()
                            self.tactile_frame_temp_10[row_num, 0, 0] = self.tactile_frame_temp_10[row_num, 0, 0] + 1  # 多出的第一列记录当前行读取数目

                ### 读满一帧触觉图像
                if not ((self.tactile_frame_temp_10[:, 0, 0] < 10).any()):
                    for i in range(16):
                        samples_num = int(self.tactile_frame_temp_10[i, 0, 0])
                        for j in range(16):
                            index = np.argsort(self.tactile_frame_temp_10[i, j+1, :samples_num])  # 从小到大
                            half_num = int(samples_num * 1/2)
                            if len(index) > 0:
                                self.tactile_frame[i, j] = np.mean(self.tactile_frame_temp_10[i, j+1, half_num:half_num+4])
                            else:
                                self.tactile_frame[i, j] = self.tactile_frame_temp_10[i, j+1, 0].copy()
                    self.tactile_frame_temp_10[:, :, :] = 0

                    ################# 用前面10组数据初始化 ####################
                    if self.tactile_init < self.tactile_frame_10.shape[0]:
                        self.tactile_frame_10[self.tactile_init, :, :] = self.tactile_frame
                        self.tactile_init = self.tactile_init + 1
                        print("self.tactile_init:", self.tactile_init)
                    else:
                        # self.frame = abs(self.frame - np.average(self.frame_10, axis=0))
                        self.tactile_frame = self.tactile_frame - np.average(self.tactile_frame_10, axis=0) # reduce the initial value
                        self.tactile_frame = signal.medfilt(self.tactile_frame, (3, 3))  # median filtering
                        self.tactile_frame[self.tactile_frame < 0.0000001] = 0.0000001
                        self.tactile_frame_count = self.tactile_frame_count + 1

                        self.imuFlag = True
                        if not self.tactile_initialized_flag:
                            self.tactile_initialized_flag = True
                            self.save_file_flag = True

                        if self.save_file_flag:
                            t = time.time()
                            ################## save tactile data #################
                            with open(self.tactile_data_file, "a") as f:
                                ### time data
                                tactile_frame_time = np.ones((self.tactile_frame.shape[1], 1))*t
                                tactile_frame_save = np.concatenate((tactile_frame_time, self.tactile_frame), axis=1)
                                np.savetxt(f, tactile_frame_save)
                                f.close()

                        self.tactile_fps_count += 1
                        if self.tactile_fps_count % 10 == 0:
                            fps = 10 / (time.time() - self.tactile_fps_time)
                            print("tactile fps:", fps)
                            self.tactile_fps_time = time.time()

    def listen_key(self):
        keyboard.hook(self.on_key_event)

    def on_key_event(self, event):
        self.pos_step = 0.005
        self.theta_step = np.round(5/180.0*np.pi, 3)
        self.grasp_step = 5
        if self.save_file_flag:
            if event.name == 'w':
                if event.event_type == 'down':
                    self.move_y_negative_button.setStyleSheet("background-color: yellow")
                    self.delta_pos_camera[1] = -self.pos_step
                    print(f'Key {event.name} {event.event_type}')
                else:
                    self.move_y_negative_button.setStyleSheet("background-color: color:rgb(0,0,0,0)")
                    self.delta_pos_camera[1] = 0.0
                    print(f'Key {event.name} {event.event_type}')

            if event.name == 's':
                print(f'Key {event.name} {event.event_type}')
                if event.event_type == 'down':
                    self.move_y_positive_button.setStyleSheet("background-color: yellow")
                    self.delta_pos_camera[1] = self.pos_step
                else:
                    self.delta_pos_camera[1] = 0.0
                    self.move_y_positive_button.setStyleSheet("background-color: color:rgb(0,0,0,0)")

            if event.name == 'a':
                print(f'Key {event.name} {event.event_type}')
                if event.event_type == 'down':
                    self.delta_pos_camera[0] = -self.pos_step
                    self.move_x_negative_button.setStyleSheet("background-color: yellow")
                else:
                    self.delta_pos_camera[0] = 0.0
                    self.move_x_negative_button.setStyleSheet("background-color: color:rgb(0,0,0,0)")

            if event.name == 'd':
                print(f'Key {event.name} {event.event_type}')
                if event.event_type == 'down':
                    self.delta_pos_camera[0] = self.pos_step
                    self.move_x_positive_button.setStyleSheet("background-color: yellow")
                else:
                    self.delta_pos_camera[0] = 0.0
                    self.move_x_positive_button.setStyleSheet("background-color: color:rgb(0,0,0,0)")

            if event.name == 'up':
                print(f'Key {event.name} {event.event_type}')
                if event.event_type == 'down':
                    self.delta_pos_camera[2] = -self.pos_step
                    self.move_z_negative_button.setStyleSheet("background-color: yellow")
                else:
                    self.delta_pos_camera[2] = 0.0
                    self.move_z_negative_button.setStyleSheet("background-color: color:rgb(0,0,0,0)")

            if event.name == 'down':
                print(f'Key {event.name} {event.event_type}')
                if event.event_type == 'down':
                    self.delta_pos_camera[2] = self.pos_step
                    self.move_z_positive_button.setStyleSheet("background-color: yellow")
                else:
                    self.delta_pos_camera[2] = 0.0
                    self.move_z_positive_button.setStyleSheet("background-color: color:rgb(0,0,0,0)")

            if event.name == 'left':
                print(f'Key {event.name} {event.event_type}')
                if event.event_type == 'down':
                    self.delta_theta_camera[2] = self.theta_step
                    self.rotate_z_negative_button.setStyleSheet("background-color: yellow")
                else:
                    self.delta_theta_camera[2] = 0.0
                    self.rotate_z_negative_button.setStyleSheet("background-color: color:rgb(0,0,0,0)")

            if event.name == 'right':
                print(f'Key {event.name} {event.event_type}')
                if event.event_type == 'down':
                    self.delta_theta_camera[2] = -self.theta_step
                    self.rotate_z_positive_button.setStyleSheet("background-color: yellow")
                else:
                    self.delta_theta_camera[2] = 0.0
                    self.rotate_z_positive_button.setStyleSheet("background-color: color:rgb(0,0,0,0)")

            if event.name == 'e':
                print(f'Key {event.name} {event.event_type}')
                if event.event_type == 'down':
                    self.grasp_button.setStyleSheet("background-color: yellow")
                    self.grasp_pos += self.grasp_step
                    print("self.grasp_pos:", self.grasp_pos)
                    self.gripper.set_pos(self.grasp_pos, step_time=0.07)
                else:
                    self.grasp_button.setStyleSheet("background-color: color:rgb(0,0,0,0)")

            if event.name == 'q':
                print(f'Key {event.name} {event.event_type}')
                if event.event_type == 'down':
                    self.release_button.setStyleSheet("background-color: yellow")
                    if np.max(self.grasp_pos) >= self.grasp_step:
                        self.grasp_pos -= self.grasp_step
                        print("self.grasp_pos:", self.grasp_pos)
                        self.gripper.set_pos(self.grasp_pos, step_time=0.07)
                else:#if event.event_type == 'up':
                    self.release_button.setStyleSheet("background-color: color:rgb(0,0,0,0)")

            if event.name == 'c':
                print(f'Key {event.name} {event.event_type}')
                if event.event_type == 'down':
                    self.rake_button.setStyleSheet("background-color: yellow")
                    self.grasp_pos[:2] += self.grasp_step
                    self.grasp_pos[3] = 80
                    print("self.grasp_pos:", self.grasp_pos)
                    self.gripper.set_pos(self.grasp_pos, step_time=0.07)
                else:#if event.event_type == 'up':
                    self.rake_button.setStyleSheet("background-color: color:rgb(0,0,0,0)")

            if event.name == 'z':
                print(f'Key {event.name} {event.event_type}')
                if event.event_type == 'down':
                    self.release_rake_button.setStyleSheet("background-color: yellow")
                    if np.max(self.grasp_pos[:2]) >= self.grasp_step:
                        self.grasp_pos[:2] -= self.grasp_step
                        self.grasp_pos[3] = 80
                        print("self.grasp_pos:", self.grasp_pos)
                        self.gripper.set_pos(self.grasp_pos, step_time=0.07)
                else:  # if event.event_type == 'up':
                    self.release_rake_button.setStyleSheet("background-color: color:rgb(0,0,0,0)")

            if event.name == '1':
                print(f'Key {event.name} {event.event_type}')
                if event.event_type == 'down':
                    # self.startBtn.setStyleSheet("background-color: yellow")
                    self.start_timer()

            if event.name == '2':
                print(f'Key {event.name} {event.event_type}')
                if event.event_type == 'down':
                    # self.endBtn.setStyleSheet("background-color: yellow")
                    self.end_timer()

            print("self.delta_pos_camera:", self.delta_pos_camera)
            self.delta_pos_world = self.Pos_c2w(self.delta_pos_camera)
            self.delta_theta_world = self.delta_theta_camera

            self.pos_world += self.delta_pos_world
            self.theta_world += self.delta_theta_world
            self.robot.set_target(pos=self.pos_world, theta=self.theta_world)
            time.sleep(0.1)

            # self.robot.set_robot_pose_origin(
            #     pos=self.pos_world, theta=self.theta_world, asynchronous=True)
            # self.robot.set_robot_pose_relative(
            #     pos=self.delta_pos_world, theta=self.delta_theta_world, asynchronous=True)



    # input (3,), output (3,)
    # remote control, pos in camera -> pos in world, then send to robot
    def Pos_c2w(self, pos=np.zeros(3)):
        # ~1000Hz
        theta_x = 0.0 / 180.0 * np.pi
        theta_y = 0.0 / 180.0 * np.pi
        theta_z = -np.pi/2+self.theta_world[2]
        # euler = np.array([theta_x, theta_y, theta_z]).reshape(-1, 1)
        # theta_x = euler[0]
        # theta_y = euler[1]
        # theta_z = euler[2]
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(theta_x), -np.sin(theta_x)],
                       [0, np.sin(theta_x), np.cos(theta_x)]])
        Ry = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],
                       [0, 1, 0],
                       [-np.sin(theta_y), 0, np.cos(theta_y)]])
        Rz = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                       [np.sin(theta_z), np.cos(theta_z), 0],
                       [0, 0, 1]])
        R_c2w = Rx @ (Ry @ Rz)
        pos = np.array(pos).reshape(-1, 1)
        pos_w = R_c2w @ pos
        return pos_w.squeeze()

    def move_x_positive_camera(self):
        pass

    def move_x_negative_camera(self):
        pass

    def move_y_positive_camera(self):
        pass

    def move_y_negative_camera(self):
        pass

    def move_z_positive_camera(self):
        pass

    def move_z_negative_camera(self):
        pass

    def rotate_z_positive_camera(self):
        pass

    def rotate_z_negative_camera(self):
        pass

    def grasp(self):
        pass

    def release(self):
        pass

    def rake(self):
        pass

    def release_rake(self):
        pass

    def read_and_show_image(self):
        self.frame1, self.frame2 = self.camera.readRawFrame()
        if self.save_file_flag:
            # self.frame1s.append(self.frame1)
            # self.frame2s.append(self.frame2)
            frame = np.concatenate([self.frame1, self.frame2], axis=1)
            self.frames.append(frame)

            with open(self.gripper_data_file, 'a') as f:
                gripper_data_save = np.concatenate((np.array(time.time()).reshape(1, -1), self.grasp_pos.reshape(1, -1)), axis=1)
                np.savetxt(f, gripper_data_save)
                f.close()

        self.camera_fps_count += 1
        if self.camera_fps_count % 10 == 0:
            fps = 10/(time.time()-self.camera_fps_time)
            print("camera fps:", fps)
            self.camera_fps_time = time.time()

        frame = self.frame1
        qimg = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
        qpix = QPixmap.fromImage(qimg)
        self.frame1_label.setPixmap(qpix)

        frame = self.frame2
        qimg = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
        qpix = QPixmap.fromImage(qimg)
        self.frame2_label.setPixmap(qpix)



    def generate_action(self):
        pass

    def execute_action(self):
        pass

    # input: nx3, object pos in camera (cm)
    # output: nx3, object pos in world (cm)
    def pos_camera2world(self, data):
        pos_top_world = np.array([10, 10, 0])
        camera_pos = np.array([10, 10, 20])
        data = np.copy(data).reshape(-1, 3)
        data = data.T
        theta_z = np.pi/2
        Rz = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                       [np.sin(theta_z), np.cos(theta_z), 0],
                       [0, 0, 1]])
        camera_origin_world = pos_top_world.reshape(3, 1)
        camera_origin_world[2, 0] = camera_pos[2]-13  # cm
        print("pos_top_world:", pos_top_world)
        data = Rz.T @ data + camera_origin_world
        return data.T

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = ImagePixelReader()
    ex.show()
    try:
        sys.exit(app.exec_())
    except Exception as e:
        print("An unexpected error occurred:", e)