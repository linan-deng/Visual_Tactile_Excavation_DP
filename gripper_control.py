import serial
import serial.tools.list_ports
import time
import numpy as np
import tkinter
from tkinter import *
import threading

class GripperControl:
    
    def __init__(self):
        self.motor_num = 4
        self.ser = serial.Serial('COM34', 9600, timeout=1)
        self.pos = [0, 0, 0, 0]

    def posLimit(self, pos):
        min_pos = 0
        max_pos1 = 80
        max_pos2 = 80
        if pos[0]<min_pos: pos[0] = min_pos
        if pos[1]<min_pos: pos[1] = min_pos
        if pos[2]<min_pos: pos[2] = min_pos
        if pos[3]<min_pos: pos[3] = min_pos
        if pos[0]>max_pos1: pos[0] = max_pos1
        if pos[1]>max_pos2: pos[1] = max_pos2
        if pos[2]>max_pos1: pos[2] = max_pos1
        if pos[3]>max_pos2: pos[3] = max_pos2
        return pos

    def write(self, pos):
        pos = self.posLimit(pos)
        send_data = ",".join(str(i) for i in pos)
        send_data = "#" + send_data
        print(send_data)
        self.ser.write(send_data.encode())

    def set_value(self, value, key):
        paramVar.set(f"change [{key}] to {value}")
        send_pos = [int(f1_1.get()), int(f1_2.get()), int(f2_1.get()), int(f2_2.get())]
        self.write(send_pos)
        time.sleep(0.07)

    def set_pos(self, pos, step_time=0.07):
        pos = self.posLimit(pos)
        self.write(pos)
        time.sleep(step_time)

    def vel_control(self, start_pos, end_pos, step_time=0.07):
        start_pos = np.array(start_pos)
        end_pos = np.array(end_pos)
        # distance = np.sqrt(np.sum(np.square(end_pos-start_pos)))
        step_num = int(max(abs(end_pos-start_pos)))
        if step_num>0:
            step = (end_pos - start_pos) / step_num
            for i in range(step_num):
                start_pos = start_pos + step
                send_pos = np.asarray(start_pos, dtype=int)
                # print(i, send_pos)
                self.write(send_pos)
                time.sleep(step_time)

if __name__ == "__main__":
    gripper = GripperControl()
    root = Tk()
    root.title("Motor Control")
    root.geometry("600x300")
    root.wm_resizable(False, False)
    height = 480
    width = 640
    f1_1 = IntVar()
    f1_2 = IntVar()
    f2_1 = IntVar()
    f2_2 = IntVar()
    scales = {'Finger1-MCP': {'range': (0, 90), 'value': f1_1, 'pos': 0, 'describe': "Finger1/MCP"},
              'Finger1-PIP': {'range': (0, 90), 'value': f1_2, 'pos': 1, 'describe': "Finger1/PIP"},
              'Finger2-MCP': {'range': (0, 90), 'value': f2_1, 'pos': 2, 'describe': "Finger2/MCP"},
              'Finger2-PIP': {'range': (0, 90), 'value': f2_2, 'pos': 3, 'describe': "Finger2/PIP"}}
    for k, v in scales.items():
        # Label(root, text=v['describe']).grid(row=v['pos'], column=0, padx=5 )
        scales[k]['target'] = Scale(root,
                    label=v['describe'],  # 标签
                    variable=v['value'],  # 值
                    from_=v['range'][0],  # 最小值， 记住是from_， from是关键字
                    to=v['range'][1],  # 最大值
                    resolution= 1,#v['range'][1]/60,  # 步进值
                    show=1,  # 是否在上面显示值
                    orient=HORIZONTAL,  # 水平显示
                    length=450,  # 滑块长度
                    # tickinterval=10,
                    command=lambda value, key=k: gripper.set_value(value, key))
        v['value'].set(0) # 设为默认值
        scales[k]['target'].grid(row=v['pos'], column=0, columnspan=3)

    paramVar = StringVar()
    Label(root, text="参数信息:").grid(row=5, pady=18)
    Label(root, textvariable=paramVar).grid(row=5, column=1)

    root.mainloop()

    # if Motor.ser == None:
    #     exit(0)
    # times = 0
    # while True:
    #     times = times + 1
    #     print("############", times, "############")
    #     pos1 = [0, 0, 0, 0]
    #     pos2 = [0, 40, 0, 40]
    #     pos3 = [50, 0, 50, 0]
    #     pos4 = [50, 10, 50, 10]
    #     t = 0.1
    #     num = 30
    #     vel_control(pos1, pos2, t, num)
    #     vel_control(pos2, pos1, t, num)
    #     vel_control(pos1, pos3, t, num)
    #     vel_control(pos3, pos4, t, num)
    #     vel_control(pos4, pos3, t, num)
    #     vel_control(pos3, pos1, t, num)
