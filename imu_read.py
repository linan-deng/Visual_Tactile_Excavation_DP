import serial
import numpy as np
import time
import threading
import serial.tools.list_ports
from array import array

record_flag = 0
record_data = np.zeros((1, 2), dtype=np.float64)
frame_data = np.zeros((1, 2), dtype=np.float64)

class IMURead:
    def __init__(self):
        self.ser = serial.Serial("COM23", 115200,
                parity=serial.PARITY_NONE,
                timeout=10,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS)

        self.CmdPacket_Begin = 0x49  # 起始码
        self.CmdPacket_End = 0x4D  # 结束码
        self.CmdPacketMaxDatSizeRx = 73  # 模块发来的数据包的数据体最大长度

        self.CS = 0  # 校验和
        self.i = 0
        self.RxIndex = 0
        self.buf = bytearray(5 + self.CmdPacketMaxDatSizeRx)  # 接收包缓存
        self.cmdLen = 0  # 长度
        self.t1 = time.time()
        self.imu_data = np.zeros(34)
        self.stopFlag = False
        print(self.imu_data)
        self.dataFlag = False
        self.startTime = time.time()
        self.count = 0
        self.imu_file_name = "tactile_dataset/" + str(self.startTime) + "_imu_data.txt"
        ################### set imu parameter #####################
        # self.ser.write(bytes.fromhex(
        #     "000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000FF00FF49FF0B1205FF00041E010305FF0F594D"))  # 1.发送配置参数
        self.set_parameter()
        time.sleep(0.2)
        # self.ser.write(bytes.fromhex(
        #     "0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000FF00FF49FF0103034D"))  # 2.唤醒传感器
        self.start_sensor()
        time.sleep(0.2)
        # self.ser.write(bytes.fromhex(
        #     "0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000FF00FF49FF0119194D"))  # 3.开启主动上报
        self.set_active_send()

        # self.data =
        # * 加速度0-2: aX, aY, aZ,
        # (含重力)加速度3-5: AX, AY, AZ,
        # * 角速度6-8: GX, GY, GZ,
        # * 磁场9-11: CX, CY, CZ,
        # * 温度/气压/高度12-14: temperature, airPressure, height,
        # 四元数15-18: w, x, y, z
        # * 角度19-21: angleX, angleY, angleZ,
        # * 坐标22-24: offsetX, offsetY, offsetZ,
        # 走/跑/骑/车25-28: 25, 26, 27, 28,
        # 导航系加速度29-31: asX, asY, asZ,
        # ADC电压/GPIO: 32-33

    def Cmd_RxUnpack(self, buf, DLen):
        scaleAccel = 0.00478515625  # 加速度 [-16g~+16g]    9.8*16/32768
        scaleQuat = 0.000030517578125  # 四元数 [-1~+1]         1/32768
        scaleAngle = 0.0054931640625  # 角度   [-180~+180]     180/32768
        scaleAngleSpeed = 0.06103515625  # 角速度 [-2000~+2000]    2000/32768
        scaleMag = 0.15106201171875  # 磁场 [-4950~+4950]   4950/32768
        scaleTemperature = 0.01  # 温度
        scaleAirPressure = 0.0002384185791  # 气压 [-2000~+2000]    2000/8388608
        scaleHeight = 0.0010728836  # 高度 [-9000~+9000]    9000/8388608

        imu_dat = array('f', [0.0 for i in range(0, 34)])
        # print("rev data:",buf)
        if buf[0] == 0x11:
            ctl = (buf[2] << 8) | buf[1]
            # print("\n subscribe tag: 0x%04x" % ctl)
            # print("ms: ", ((buf[6] << 24) | (buf[5] << 16) | (buf[4] << 8) | (buf[3] << 0)))

            L = 7  # 从第7字节开始根据 订阅标识tag来解析剩下的数据
            if ((ctl & 0x0001) != 0):
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel
                L += 2
                tmpY = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel
                L += 2
                tmpZ = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel
                L += 2
                imu_dat[0] = float(tmpX)
                imu_dat[1] = float(tmpY)
                imu_dat[2] = float(tmpZ)
                # print("\taX: %.3f" % tmpX)  # x加速度aX
                # print("\taY: %.3f" % tmpY)  # y加速度aY
                # print("\taZ: %.3f" % tmpZ)  # z加速度aZ

            if ((ctl & 0x0002) != 0):
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel
                L += 2
                tmpY = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel
                L += 2
                tmpZ = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel
                L += 2
                imu_dat[3] = float(tmpX)
                imu_dat[4] = float(tmpY)
                imu_dat[5] = float(tmpZ)
                # print("\tAX: %.3f" % tmpX)  # x加速度AX
                # print("\tAY: %.3f" % tmpY)  # y加速度AY
                # print("\tAZ: %.3f" % tmpZ)  # z加速度AZ

            if ((ctl & 0x0004) != 0):
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAngleSpeed
                L += 2
                tmpY = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAngleSpeed
                L += 2
                tmpZ = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAngleSpeed
                L += 2
                imu_dat[6] = float(tmpX)
                imu_dat[7] = float(tmpY)
                imu_dat[8] = float(tmpZ)
                # print("\tGX: %.3f" % tmpX)  # x角速度GX
                # print("\tGY: %.3f" % tmpY)  # y角速度GY
                # print("\tGZ: %.3f" % tmpZ)  # z角速度GZ

            if ((ctl & 0x0008) != 0):
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleMag
                L += 2
                tmpY = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleMag
                L += 2
                tmpZ = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleMag
                L += 2
                imu_dat[9] = float(tmpX)
                imu_dat[10] = float(tmpY)
                imu_dat[11] = float(tmpZ)
                # print("\tCX: %.3f" % tmpX)  # x磁场CX
                # print("\tCY: %.3f" % tmpY)  # y磁场CY
                # print("\tCZ: %.3f" % tmpZ)  # z磁场CZ

            if ((ctl & 0x0010) != 0):
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleTemperature
                L += 2

                tmpU32 = np.uint32(((np.uint32(buf[L + 2]) << 16) | (np.uint32(buf[L + 1]) << 8) | np.uint32(buf[L])))
                if ((tmpU32 & 0x800000) == 0x800000):  # 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
                    tmpU32 = (tmpU32 | 0xff000000)
                tmpY = np.int32(tmpU32) * scaleAirPressure
                L += 3

                tmpU32 = np.uint32((np.uint32(buf[L + 2]) << 16) | (np.uint32(buf[L + 1]) << 8) | np.uint32(buf[L]))
                if ((tmpU32 & 0x800000) == 0x800000):  # 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
                    tmpU32 = (tmpU32 | 0xff000000)
                tmpZ = np.int32(tmpU32) * scaleHeight
                L += 3

                imu_dat[12] = float(tmpX)
                imu_dat[13] = float(tmpY)
                imu_dat[14] = float(tmpZ)
                # print("\ttemperature: %.2f" % tmpX)  # 温度
                # print("\tairPressure: %.3f" % tmpY)  # 气压
                # print("\theight: %.3f" % tmpZ)  # 高度

            if ((ctl & 0x0020) != 0):
                tmpAbs = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleQuat
                L += 2
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleQuat
                L += 2
                tmpY = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleQuat
                L += 2
                tmpZ = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleQuat
                L += 2
                imu_dat[15] = float(tmpAbs)
                imu_dat[16] = float(tmpX)
                imu_dat[17] = float(tmpY)
                imu_dat[18] = float(tmpZ)
                # print("\tw: %.3f" % tmpAbs)  # w
                # print("\tx: %.3f" % tmpX)  # x
                # print("\ty: %.3f" % tmpY)  # y
                # print("\tz: %.3f" % tmpZ)  # z

            if ((ctl & 0x0040) != 0):
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAngle
                L += 2
                tmpY = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAngle
                L += 2
                tmpZ = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAngle
                L += 2
                imu_dat[19] = float(tmpX)
                imu_dat[20] = float(tmpY)
                imu_dat[21] = float(tmpZ)
                # print("\tangleX: %.3f" % tmpX)  # x角度
                # print("\tangleY: %.3f" % tmpY)  # y角度
                # print("\tangleZ: %.3f" % tmpZ)  # z角度

            if ((ctl & 0x0080) != 0):
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) / 1000.0
                L += 2
                tmpY = np.short((np.short(buf[L + 1]) << 8) | buf[L]) / 1000.0
                L += 2
                tmpZ = np.short((np.short(buf[L + 1]) << 8) | buf[L]) / 1000.0
                L += 2
                imu_dat[22] = float(tmpX)
                imu_dat[23] = float(tmpY)
                imu_dat[24] = float(tmpZ)
                # print("\toffsetX: %.3f" % tmpX)  # x坐标
                # print("\toffsetY: %.3f" % tmpY)  # y坐标
                # print("\toffsetZ: %.3f" % tmpZ)  # z坐标

            if ((ctl & 0x0100) != 0):
                tmpU32 = ((buf[L + 3] << 24) | (buf[L + 2] << 16) | (buf[L + 1] << 8) | (buf[L] << 0))
                L += 4
                # print("\tsteps: %u" % tmpU32)  # 计步数
                tmpU8 = buf[L]
                L += 1
                if (tmpU8 & 0x01):  # 是否在走路
                    # print("\t walking yes")
                    imu_dat[25] = 100
                else:
                    # print("\t walking no")
                    imu_dat[25] = 0
                if (tmpU8 & 0x02):  # 是否在跑步
                    # print("\t running yes")
                    imu_dat[26] = 100
                else:
                    # print("\t running no")
                    imu_dat[26] = 0
                if (tmpU8 & 0x04):  # 是否在骑车
                    # print("\t biking yes")
                    imu_dat[27] = 100
                else:
                    # print("\t biking no")
                    imu_dat[27] = 0
                if (tmpU8 & 0x08):  # 是否在开车
                    # print("\t driving yes")
                    imu_dat[28] = 100
                else:
                    # print("\t driving no")
                    imu_dat[28] = 0

            if ((ctl & 0x0200) != 0):
                tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel
                L += 2
                tmpY = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel
                L += 2
                tmpZ = np.short((np.short(buf[L + 1]) << 8) | buf[L]) * scaleAccel
                L += 2
                imu_dat[29] = float(tmpX)
                imu_dat[30] = float(tmpY)
                imu_dat[31] = float(tmpZ)
                # print("\tasX: %.3f" % tmpX)  # x加速度asX
                # print("\tasY: %.3f" % tmpY)  # y加速度asY
                # print("\tasZ: %.3f" % tmpZ)  # z加速度asZ

            if ((ctl & 0x0400) != 0):
                tmpU16 = ((buf[L + 1] << 8) | (buf[L] << 0))
                L += 2
                # print("\tadc: %u" % tmpU16)  # adc测量到的电压值，单位为mv
                imu_dat[32] = float(tmpU16)

            if ((ctl & 0x0800) != 0):
                tmpU8 = buf[L]
                L += 1
                # print("\t GPIO1  M:%X, N:%X" % ((tmpU8 >> 4) & 0x0f, (tmpU8) & 0x0f))
                imu_dat[33] = float(tmpU8)
            self.imu_data = imu_dat
            self.dataFlag = True

            # with open(self.imu_file_name, "ab") as g:
            #     np.savetxt(g, np.array(self.imu_data).reshape(1, -1))
            # self.count += 1
            # print("self.count:", self.count)

            self.t2 = time.time()
            if self.t2 - self.t1 > 0:
                # print("IMU fps:", 1/(self.t2-self.t1))
                self.t1 = self.t2
        else:
            print("[error] data head not define")

    def Cmd_GetPkt(self, byte):
        self.CS += byte  # 边收数据边计算校验码，校验码为地址码开始(包含地址码)到校验码之前的数据的和
        if self.RxIndex == 0:  # 起始码
            if byte == self.CmdPacket_Begin:
                self.i = 0
                self.buf[self.i] = self.CmdPacket_Begin
                self.i += 1
                self.CS = 0  # 下个字节开始计算校验码
                self.RxIndex = 1
        elif self.RxIndex == 1:  # 数据体的地址码
            self.buf[self.i] = byte
            self.i += 1
            if byte == 255:  # 255是广播地址，模块作为从机，它的地址不可会出现255
                self.RxIndex = 0
            else:
                self.RxIndex += 1
        elif self.RxIndex == 2:  # 数据体的长度
            self.buf[self.i] = byte
            self.i += 1
            if byte > self.CmdPacketMaxDatSizeRx or byte == 0:  # 长度无效
                self.RxIndex = 0
            else:
                self.RxIndex += 1
                self.cmdLen = byte
        elif self.RxIndex == 3:  # 获取数据体的数据
            self.buf[self.i] = byte
            self.i += 1
            if self.i >= self.cmdLen + 3:  # 已收完数据体
                self.RxIndex += 1
        elif self.RxIndex == 4:  # 对比 效验码
            self.CS -= byte
            if (self.CS & 0xFF) == byte:  # 校验正确
                self.buf[self.i] = byte
                self.i += 1
                self.RxIndex += 1
            else:  # 校验失败
                self.RxIndex = 0
        elif self.RxIndex == 5:  # 结束码
            self.RxIndex = 0
            if byte == self.CmdPacket_End:  # 捕获到完整包
                self.buf[self.i] = byte
                self.i += 1
                hex_string = " ".join(f"{b:02X}" for b in self.buf[0:self.i])
                # print(f"U-Rx[Len={self.i}]:{hex_string}")
                self.Cmd_RxUnpack(self.buf[3:self.i - 2], self.i - 5)  # 处理数据包的数据体
                return 1
        else:
            self.RxIndex = 0
        return 0

    def set_zero_pos(self):
        send_data = [0x00]*46 # 固定前导码
        send_data.append(0x00)
        send_data.append(0xFF)
        send_data.append(0x00)
        send_data.append(0xFF) # 固定前导码
        send_data.append(0x49) # 起始码，固定为0x49
        send_data.append(0xFF) # 串口通信地址，广播地址255
        ################ user modify ################
        send_data.append(0x01)  # 长度：数据体的字节数
        send_data.append(0x13)  # 数据体：具体内容的负载
        send_data.append(0x13)  # 校验码：从地址到数据体(包含地址、长度和数据体)每字节数据的和校验
        #############################################
        send_data.append(0x4D)  # 结束码：固定为0x4D
        # print(send_data)
        send_data = bytes(send_data)
        # print(send_data)
        self.ser.write(send_data)
        time.sleep(0.01)
        data = self.ser.read_all()
        time.sleep(0.1)
        # print(data)
        print("set set zero pos !!!")

    def set_parameter(self):
        send_data = [0x00]*46 # 固定前导码
        send_data.append(0x00)
        send_data.append(0xFF)
        send_data.append(0x00)
        send_data.append(0xFF) # 固定前导码
        send_data.append(0x49) # 起始码，固定为0x49
        send_data.append(0xFF) # 串口通信地址，广播地址255
        ################ user modify ################
        # user_data = [0x12, 0x05, 0xFF, 0x00, 0x04, 0x01, 0x01, 0x03, 0x05, 0xFF, 0x0F] # 用户自定义数据体
        # user_data = [0x12, 0x05, 0xFF, 0x00, 0x04, 0xC8, 0x01, 0x03, 0x05, 0xDD, 0x00] # 静止时位置归零
        user_data = [0x12, 0x05, 0x00, 0x00, 0x05, 0xC8, 0x01, 0x03, 0x05, 0xDD, 0x00] # 静止时位置不归零,融合磁场
        send_data.append(len(user_data))  # 长度：数据体的字节数
        # print(len(user_data))
        send_data = np.concatenate((send_data, user_data), axis=0) # 数据体：具体内容的负载
        send_data = send_data.tolist()
        crc = np.sum([0xFF]) + len(user_data) + np.sum(user_data)
        crc = crc & 0xff
        if crc > 255:
            crc = 256 - crc
        # print(crc)
        send_data.append(crc) # 校验码：从地址到数据体(包含地址、长度和数据体)每字节数据的和校验
        #############################################
        send_data.append(0x4D)  # 结束码：固定为0x4D
        # print(send_data)
        send_data = bytes(send_data)
        # print(send_data)
        self.ser.write(send_data)
        time.sleep(0.01)
        data = self.ser.read_all()
        time.sleep(0.1)
        # print(data)
        print("set imu parameter !!!")

    def start_sensor(self):
        send_data = [0x00]*46 # 固定前导码
        send_data.append(0x00)
        send_data.append(0xFF)
        send_data.append(0x00)
        send_data.append(0xFF) # 固定前导码
        send_data.append(0x49) # 起始码，固定为0x49
        send_data.append(0xFF) # 串口通信地址，广播地址255
        ################ user modify ################
        send_data.append(0x01)  # 长度：数据体的字节数
        send_data.append(0x03)  # 数据体：具体内容的负载
        send_data.append(0x03)  # 校验码：从地址到数据体(包含地址、长度和数据体)每字节数据的和校验
        #############################################
        send_data.append(0x4D)  # 结束码：固定为0x4D
        # print(send_data)
        send_data = bytes(send_data)
        # print(send_data)
        self.ser.write(send_data)
        time.sleep(0.01)
        data = self.ser.read_all()
        time.sleep(0.01)
        # print("data:", data)
        print("start imu !!!")

    def set_active_send(self):
        send_data = [0x00]*46 # 固定前导码
        send_data.append(0x00)
        send_data.append(0xFF)
        send_data.append(0x00)
        send_data.append(0xFF)  # 固定前导码
        send_data.append(0x49)  # 起始码，固定为0x49
        send_data.append(0xFF)  # 串口通信地址，广播地址255
        ################ user modify ################
        # user_data = [0x12, 0x05, 0xFF, 0x00, 0x04, 0x01, 0x01, 0x03, 0x05, 0xFF, 0x0F] # 用户自定义数据体
        user_data = [0x19]
        send_data.append(len(user_data))  # 长度：数据体的字节数
        # print(len(user_data))
        send_data = np.concatenate((send_data, user_data), axis=0) # 数据体：具体内容的负载
        send_data = send_data.tolist()
        crc = np.sum([0xFF]) + len(user_data) + np.sum(user_data)
        crc = crc & 0xff
        if crc > 255:
            crc = 256 - crc
        # print(crc)
        send_data.append(crc) # 校验码：从地址到数据体(包含地址、长度和数据体)每字节数据的和校验
        #############################################
        send_data.append(0x4D)  # 结束码：固定为0x4D
        # print(send_data)
        send_data = bytes(send_data)
        # print(send_data)
        self.ser.write(send_data)
        time.sleep(0.01)
        data = self.ser.read_all()
        time.sleep(0.1)
        # print(data)
        print("set imu active send !!!")

    def set_active_send_close(self):
        send_data = [0x00]*46 # 固定前导码
        send_data.append(0x00)
        send_data.append(0xFF)
        send_data.append(0x00)
        send_data.append(0xFF) # 固定前导码
        send_data.append(0x49) # 起始码，固定为0x49
        send_data.append(0xFF) # 串口通信地址，广播地址255
        ################ user modify ################
        # user_data = [0x12, 0x05, 0xFF, 0x00, 0x04, 0x01, 0x01, 0x03, 0x05, 0xFF, 0x0F] # 用户自定义数据体
        user_data = [0x18]
        send_data.append(len(user_data))  # 长度：数据体的字节数
        # print(len(user_data))
        send_data = np.concatenate((send_data, user_data), axis=0) # 数据体：具体内容的负载
        send_data = send_data.tolist()
        crc = np.sum([0xFF]) + len(user_data) + np.sum(user_data)
        crc = crc & 0xff
        if crc > 255:
            crc = 256 - crc
        # print(crc)
        send_data.append(crc) # 校验码：从地址到数据体(包含地址、长度和数据体)每字节数据的和校验
        #############################################
        send_data.append(0x4D)  # 结束码：固定为0x4D
        # print(send_data)
        send_data = bytes(send_data)
        # print(send_data)
        self.ser.write(send_data)
        time.sleep(0.01)
        data = self.ser.read_all()
        time.sleep(0.1)
        # print(data)
        print("set active send !!!")

    ######################### cycle read #######################
    def read_data(self):
        print("IMU read task start !!!")
        # self.ser.write(bytes.fromhex(
        #     "000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000FF00FF49FF0B1205FF00041E010305FF0F594D"))  # 1.发送配置参数
        self.set_parameter()
        time.sleep(0.2)
        # self.ser.write(bytes.fromhex(
        #     "0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000FF00FF49FF0103034D"))  # 2.唤醒传感器
        self.start_sensor()
        time.sleep(0.2)
        # self.ser.write(bytes.fromhex(
        #     "0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000FF00FF49FF0119194D"))  # 3.开启主动上报
        self.set_active_send()
        time.sleep(0.2)
        while True:
            self.dataFlag = False
            data = self.ser.read(1)  # read 1 bytes
            if len(data) > 0:  # if data is not empty
                self.Cmd_GetPkt(data[0])
                # print("theta x:", self.imu_data[19])
            if self.stopFlag:
                break

    def read_data_single(self):
        while True:
            self.dataFlag = False
            data = self.ser.read(1)  # read 1 bytes
            if len(data) > 0:  # if data is not empty
                self.Cmd_GetPkt(data[0])
                # print("theta x:", self.imu_data[19])
            if self.dataFlag:
                break
        return self.imu_data

if __name__ == '__main__':
    imuRead = IMURead()
    imuRead.read_data()
    # motorLoop = threading.Thread(target=imuRead.imu_read)
    # motorLoop.start()