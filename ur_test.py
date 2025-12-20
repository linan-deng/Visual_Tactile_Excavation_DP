import time

import rtde_control
import rtde_receive
import numpy as np

rtde_c = rtde_control.RTDEControlInterface("192.168.1.10")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.10")

tool_z = 0.2
theta_x = 0.0 / 180.0 * np.pi
theta_y = 45.0 / 180.0 * np.pi
theta_z = -135.0 / 180.0 * np.pi
rtde_c.setTcp((0, 0, tool_z, 0, 0, 0))
rtde_c.setPayload(1, (0, 0, 0))
pose_origin = [0.20689411887647957, -0.5180431565860071, 0.1591425555508029, 0.9249873883908998, -0.4011170347437063, 2.181502354533378]
rtde_c.moveL(pose=pose_origin, speed=0.1, acceleration=0.1, asynchronous=False)  # apply the new pose
theta_direction_b = np.array([0.5, -0.5, 0.7071])/10
while True:
    time.sleep(2)
    target_speed = np.zeros(6)
    target_speed[5] = 0.2
    rtde_c.speedJ(qd=target_speed, acceleration=0.5, time=1.0/500)
    time.sleep(2)
    target_speed[5] = -0.2
    rtde_c.speedJ(qd=target_speed, acceleration=0.5, time=1.0/500)
