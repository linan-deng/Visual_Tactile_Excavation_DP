import serial
import numpy as np
import time
import threading
import serial.tools.list_ports


record_flag = 0
record_data = np.zeros((1, 2), dtype=np.float64)
frame_data = np.zeros((1, 2), dtype=np.float64)


class MotorControl:
    def __init__(self):
        # self.motor_ser = serial.Serial("COM6", 9600)
        self.motor_ser = serial.Serial("COM19", 9600,
                parity=serial.PARITY_NONE,
                timeout=0.2,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS)

        self.time_now = 0
        self.pos_now = 0
        self.force_now = 0
        self.start_flag = False
        self.times = 0

    def int2float(self, int32):
        sign = (int32 >> 31) & 0x01
        exponent = ((int32 >> 23) & 0xff) - 127
        decimal = int32 & 0x7fffff
        f = 2**exponent
        for exp in range(23):
            exponent -= 1
            f += ((decimal >> (22 - exp)) & 0x01) * 2**exponent
        return f * (-1)**sign

    def record_start(self, opt):
        global record_flag
        if opt:
            self.button_record['bg'] = 'SpringGreen'
            record_flag = 1
        else:
            self.button_record['bg'] = 'White'
            record_flag = 2

    # 串口号改变函数
    def combo1_handler(self, var):
        global port_serial
        port_serial = var

    # 串口波特率改变回掉函数
    def combo2_handler(self, var):
        global bitrate_serial
        bitrate_serial = var

    def motor_setPos(self, direction_x = 0, distance_x = 0.0, time_sleep_x = 10,
                     direction_y = 0, distance_y = 0.0, time_sleep_y = 10,
                     direction_z = 0, distance_z = 0.0, time_sleep_z = 10):
        # 1:远离开关, 2:靠近开关
        # 10,000个脉冲一圈，滑块走1mm
        # 脉冲高低电平保持时间均为100ms
        send_data = str(direction_x) + ',' + str(int(float(distance_x)*10000)) + ',' + str(time_sleep_x) + ',' + \
                    str(direction_y) + ',' + str(int(float(distance_y)*10000)) + ',' + str(time_sleep_y) + ',' + \
                    str(direction_z) + ',' + str(int(float(distance_z)*10000)) + ',' + str(time_sleep_z)
        print("send data:", send_data)
        self.motor_ser.write(send_data.encode("utf-8"))

    def motor_loop(self):
        print("motor loop")
        pos_end = 5
        delay_time = 50 #微秒, 速度v = 1/(10000*2*delay_time*(10^-6)) = 50/delay_time (mm/s)
        sleep_time = pos_end * 10000 * (2 * delay_time) * 0.000001 + 3
        print("sleep time:", sleep_time)
        print("start_flag:", self.start_flag)
        while True:
            if self.start_flag:
                print("--------------------", self.times, "--------------------")
                self.motor_setPos(0,0,10,0,0,10,0,pos_end,delay_time)
                time.sleep(sleep_time)
                self.motor_setPos(0,0,10,0,0,10,1,pos_end,delay_time)
                time.sleep(sleep_time)
                self.times = self.times + 1
                self.start_flag = False
            else:
                self.times = 0

    # cycle test
    # def motor_loop(self):
    #     print("motor loop")
    #     # cycle load: 0.7
    #     pos_end = 0.7
    #     # cycle load: 50
    #     delay_time = 50 #微秒, 速度v = 1/(10000*2*delay_time*(10^-6)) = 50/delay_time (mm/s)
    #     # cycle load: +0.1
    #     sleep_time = pos_end * 10000 * (2 * delay_time) * 0.000001 + 0.3
    #     print("sleep time:", sleep_time)
    #     print("start_flag:", self.start_flag)
    #     while True:
    #         if self.start_flag:
    #             print("--------------------", self.times, "--------------------")
    #             self.motor_setPos(0,0,10,0,0,10,0,pos_end,delay_time)
    #             time.sleep(sleep_time)
    #             self.times = self.times + 1
    #             self.motor_setPos(0,0,10,0,0,10,1,pos_end,delay_time)
    #             time.sleep(sleep_time)
    #         else:
    #             self.times = 0

    # # vibration test
    # def motor_loop(self):
    #     print("motor loop")
    #     # cycle load: 0.7
    #     pos_end = 0.3
    #     # cycle load: 50
    #     delay_time = [150, 100, 50] #微秒, 速度v = 1/(10000*2*delay_time*(10^-6)) = 50/delay_time (mm/s)
    #     # cycle load: +0.1
    #     time.sleep(5)
    #     print("start_flag:", self.start_flag)
    #     while True:
    #         if self.start_flag:
    #             print("--------------------", self.times, "--------------------")
    #             for j in range(len(delay_time)):
    #                 for i in range(5):
    #                     sleep_time = pos_end * 10000 * (2 * delay_time[j]) * 0.000001+delay_time[j]/250
    #                     self.motor_setPos(0,0,10,0,0,10,0,pos_end, delay_time[j])
    #                     time.sleep(sleep_time)
    #                     self.motor_setPos(0,0,10,0,0,10,1,pos_end, delay_time[j])
    #                     time.sleep(sleep_time)
    #                     self.times = self.times + 1
    #         else:
    #             self.times = 0

    # # increment load
    # def motor_loop(self):
    #     print("motor loop")
    #     # cycle load: 0.7
    #     pos_end = [0.25, 0.3, 0.35, 0.4, 0.45]
    #     # cycle load: 50
    #     delay_time = 100 #微秒, 速度v = 1/(10000*2*delay_time*(10^-6)) = 50/delay_time (mm/s)
    #     # cycle load: +0.1
    #     time.sleep(5)
    #     print("start_flag:", self.start_flag)
    #     while True:
    #         if self.start_flag:
    #             print("--------------------", self.times, "--------------------")
    #             for j in range(len(pos_end)):
    #                 for i in range(10):
    #                     self.motor_setPos(0,0,10,0,0,10,0,pos_end[j],delay_time)
    #                     sleep_time = pos_end[j] * 10000 * (2 * delay_time) * 0.000001 + 1
    #                     # print("sleep time:", sleep_time)
    #                     time.sleep(sleep_time)
    #                     self.times = self.times + 1
    #                     self.motor_setPos(0,0,10,0,0,10,1,pos_end[j],delay_time)
    #                     time.sleep(sleep_time)
    #         else:
    #             self.times = 0

    # # step test
    # def motor_loop(self):
    #     print("motor loop")
    #     # cycle load: 0.7
    #     pos_end = 0.3
    #     # cycle load: 50
    #     delay_time = 10 #微秒, 速度v = 1/(10000*2*delay_time*(10^-6)) = 50/delay_time (mm/s)
    #     # cycle load: +0.1
    #     time.sleep(5)
    #     print("start_flag:", self.start_flag)
    #     while True:
    #         if self.start_flag:
    #             print("--------------------", self.times, "--------------------")
    #             for j in range(3):
    #                 sleep_time = pos_end * 10000 * (2 * delay_time) * 0.000001 + 3
    #                 self.motor_setPos(0,0,10,0,0,10,0,pos_end,delay_time)
    #                 time.sleep(sleep_time)
    #                 # self.motor_setPos(0,0,10,0,0,10,0,0.1,delay_time)
    #                 # time.sleep(sleep_time)
    #                 # self.motor_setPos(0,0,10,0,0,10,0,0.1,delay_time)
    #                 # time.sleep(sleep_time)
    #                 self.motor_setPos(0,0,10,0,0,10,1,pos_end,delay_time)
    #                 time.sleep(sleep_time*2)
    #                 self.times = self.times + 1
    #         else:
    #             self.times = 0

if __name__ == '__main__':
    loadTest = LoadTest()
    motorLoop = threading.Thread(target=loadTest.motor_loop)
    motorLoop.start()
