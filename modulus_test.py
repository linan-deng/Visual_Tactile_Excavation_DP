import serial
import numpy as np
import time
import threading
import serial.tools.list_ports
import tkinter as tk
from tkinter import *
from force_read import ForceRead
from platform_motion_control import MotorControl

class ModulusTest(tk.Frame):

    def __init__(self, master):
        tk.Frame.__init__(self, master)
        self.master.title("Modulus Test")
        self.master.geometry("400x200")
        self.continueRunning = True
        self.time_start = 0
        self.freq = 0
        self.count = 0
        self.loadingFlag = True
        self.times = 0
        self.force_0 = 0


        self.posLable = Label(self.master, text="Motor position")
        self.posLable.grid(row=0, column=0, sticky='e')

        self.posInput = tk.StringVar()
        self.posInput.set('0')
        self.posEntry = Entry(self.master, textvariable = self.posInput, justify='right')
        self.posEntry.grid(row=0, column=1, sticky='e')


        self.forceLable = Label(self.master, text="Force")
        self.forceLable.grid(row=1, column=0, sticky='e')

        self.forceShow = StringVar()
        self.forceShow.set("0")
        self.forceShowLable = Label(self.master, textvariable=self.forceShow)
        self.forceShowLable.grid(row=1, column=1, sticky='e')


        self.forceFreqLable = Label(self.master, text="Force frquency")
        self.forceFreqLable.grid(row=2, column=0, sticky='e')

        self.forceFreqShow = StringVar()
        self.forceFreqShow.set("0")
        self.forceFreqShowLable = Label(self.master, textvariable=self.forceFreqShow)
        self.forceFreqShowLable.grid(row=2, column=1, sticky='e')

        self.loadingButton = Button(self.master, text="Loading", command=self.loading)
        self.loadingButton.grid(row=3, column=0, sticky='ew')

        self.unloadingButton = Button(self.master, text="Unloading", command=self.unloading)
        self.unloadingButton.grid(row=3, column=1, sticky='ew')

        self.startButton = Button(self.master, text="Start Testing", command=self.startTask)
        self.startButton.grid(row=4, column=0, sticky='ew')

        self.stopButton = Button(self.master, text="Stop Testing", command=self.stopTask)
        self.stopButton.grid(row=4, column=1, sticky='ew')

    def loading(self):
        pos = self.posEntry.get()
        MC.motor_setPos(0, 0, 0, 0, 0, 0, 0, pos, 10)
        print("Target pos: +", pos)

    def unloading(self):
        pos = self.posEntry.get()
        MC.motor_setPos(0, 0, 0, 0, 0, 0, 1, pos, 10)
        print("Target pos: -", pos)

    def set_pos(self, pos):
        MC.motor_setPos(0, 0, 0, 0, 0, 0, 0, pos, 10)
        print("Target pos: +", pos)

    def release_pos(self, pos):
        MC.motor_setPos(0, 0, 0, 0, 0, 0, 1, pos, 10)
        print("Target pos: -", pos)

    def startTask(self):
        self.continueRunning = True
        self.loadingFlag = True
        self.count = 0
        self.times = 0
        self.force_0 = 0
        if (self.continueRunning):
            self.force_data_filename = 'data/' + str(time.time()) + '_force_data.txt'
            self.pos_data_filename = 'data/' + str(time.time()) + '_pos_data.txt'
            # MC.start_flag = True
            self.master.after(1, self.runTask)
        # else:
        #     self.startButton['state'] = 'enabled'

    def runTask(self):

        force = FR.force_now
        print("Force:", force)
        self.forceShow.set(force)
        self.forceFreqShow.set(self.freq)
        end_pos = 15
        counts = 100
        step_pos = end_pos/counts

        if self.times < 1:
            if self.loadingFlag:
                self.set_pos(step_pos)
                self.count = self.count + 1
                print("self.count:", self.count)
                time.sleep(0.01)
                if self.count > counts-0.1:
                    self.loadingFlag = False

                # save force data
                force = np.expand_dims(force, 0)
                with open(self.force_data_filename, "ab") as f:
                    np.savetxt(f, force, delimiter=',')

                # save times data
                pos = np.expand_dims(end_pos*self.count/counts, 0)
                with open(self.pos_data_filename, "ab") as f:
                    np.savetxt(f, pos, delimiter=',')
            else:
                self.release_pos(step_pos)
                self.count = self.count - 1
                print("self.count:", self.count)
                time.sleep(0.02)
                if self.count < 0.1:
                    self.times = self.times + 1

        if time.time()-self.time_start != 0:
            self.freq = 1/(time.time()-self.time_start)
            print("Force read frequency:", self.freq)
        time.sleep(0.001)
        self.time_start = time.time()
        # check if the task should sleep or stop
        if (self.continueRunning):
            self.master.after(1, self.runTask)
        # else:
        #     self.startButton['state'] = 'enabled'

    def stopTask(self):
        self.continueRunning = False
        self.loadingFlag = False
        MC.start_flag = False


if __name__ == '__main__':
    root = tk.Tk()
    app = ModulusTest(root)

    FR = ForceRead()
    task_FR = threading.Thread(target=FR.force_read)
    task_FR.start()
    MC = MotorControl()
    task_MC = threading.Thread(target=MC.motor_loop)
    task_MC.start()

    # 进入消息循环
    app.mainloop()




