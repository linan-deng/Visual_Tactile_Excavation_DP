import serial
import serial.tools.list_ports
import time

import numpy as np

'''
moto 的位置统一用np.matrix表示
'''
RATIOL2M = 26.526
RATIOM2L = 0.0377

class motoControl:
    def __init__(self):
        self.motor_num = 4
        self.ser = self.linkport()
        if self.ser == None:
            print("error construct")
            return
        self.init_pos = [1473, 1614, 1311, 1397]
        self.pos = self.get_pos()
        print("pos = {}".format(self.pos))

       # self.set_release()

    # 底层
    def linkport(self):
        port_list = list(serial.tools.list_ports.comports())
        print(port_list)
        if len(port_list) == 0:
            print("No useful port !!!")
            return None
        else:
            print("------------port list--------------")
            for port in port_list:
                print(port[0])
            print("-------try to link moto---------")
            try:
                bps = 115200
                timex = 0
                for port in port_list:
                    serialName = port[0]
                    if "COM" in serialName:
                        print("try %s " % serialName)
                        # os.system("sudo chmod 777 " + serialName)
                        ser = serial.Serial(serialName, bps, timeout=timex)
                        #ser.write("#001PVER!".encode())
                        version = ser.write("#001PVER!".encode())
                        print("version: ", version)
                        time.sleep(0.2)
                        if len(ser.read(ser.in_waiting).decode()):
                            print("gripper link successfully !!!")
                            return ser
                        else:
                            print("fail")


            except Exception as e:
                print("exception: ", e)
                return None
        return None

    # 写指令
    def write(self, string):
        self.ser.write(str(string).encode())

    def configMode(self):
        '''配置成电机模式'''
        cmd = "#001PMOD7!"
        self.write(cmd)

    def getID(self):
        cmd = "#001PID!"
        self.write(cmd)
        time.sleep(0.02)
        ID = self.read_pos()
        print("ID", ID)

    # 读位置
    def read_pos(self):
        pos = self.ser.read(self.ser.in_waiting).decode()
        return pos
    def get_pos_ID(self, ID):
        self.ser.flushInput()
        cmd = "#00{}PRAD!".format(ID)
        #while not self.ser.in_waiting:
        self.write(cmd)
        time.sleep(0.02)
        rec = self.read_pos()
        print("rec: ", rec)
        pos = rec.split('P')[1].split('!')[0]
        return pos
    def get_pos(self):
        pos_sum = []
        for i in range(1, self.motor_num + 1):
            pos_sum.append(int(self.get_pos_ID(i)))
        return pos_sum

    # 写四个电机 位置
    def write_pos_ID(self, pos, ID):
        '''配置成电机模式'''
        cmd = "#00{}P{}T0000!".format(ID, pos)
        self.write(cmd)
        time.sleep(0.02)
    def write_pos(self, poses):
        '''
        poses 为普通数字
        '''
        for i in range(1, self.motor_num + 1):
            self.write_pos_ID(poses[i - 1], i)


    # 上层算法 将电机位置与线长mm直接联系
    # 线程250 mm
    # 舵机模式的活动范围 0-2500
    def get_line_ID(self, ID):
        pos = int(self.get_pos_ID(ID))
        line = 250.0 - (pos - self.init_pos[ID - 1]) * RATIOM2L
        return line

    def get_line(self):
        line_sum = []
        for i in range(1, self.motor_num + 1):
            line_sum.append(int(self.get_line_ID(i)))
        return line_sum

    def write_line_ID(self, line, ID):
        '''配置成电机模式'''
        pos = int((250.0 - line) * RATIOL2M + self.init_pos[ID - 1])
        cmd = "#00{}P{}T0000!".format(ID, pos)
        self.write(cmd)
        time.sleep(0.02)

    def write_line(self, lines):
        '''
        lines 为普通数组
        '''
        for i in range(1, self.motor_num + 1):
            self.write_line_ID(lines[i - 1], i)

    def reset_pos(self):
        self.write_pos(self.init_pos)



if __name__ == "__main__":
    Moto = motoControl()
    if Moto.ser == None:
        exit(0)
   # print(Moto.init_pos.T)
    Moto.write_pos(Moto.init_pos)
    time.sleep(1)
    poses = Moto.get_pos()
    print("2", poses)

    Moto.write_line([245, 245, 245, 245])
    time.sleep(1)
    lines = Moto.get_line()
    print("3", lines)
    poses = Moto.get_pos()
    print("4", poses)
    time.sleep(5)
    Moto.reset_pos()
    Moto.ser.close()
