import skvideo.io
import os
import time
import numpy as np
import cv2

path = 'data/robot_excavation/'

file_name_all = os.listdir(path)

file_name_image = []
file_name_imu = []
for item in file_name_all:
    if 'frame_merge.mp4' in item:
        file_name_image.append(item)
    if 'imu' in item:
        file_name_imu.append(item)

for item in file_name_image:
    print('item:', item)
    t1 = time.time()
    # video_data = skvideo.io.vread(path+item)  # (4325, 640, 960, 3)
    # len_frame = data.shape[0]

    imu_file_name = item.split('_')[0] + '_imu_data.txt'
    imu_data = np.loadtxt(path+imu_file_name, delimiter=' ')
    len_imu = imu_data.shape[0]

    # 视屏获取
    videoCapture = cv2.VideoCapture(path+item)
    # 帧率(frames per second)
    fps = videoCapture.get(cv2.CAP_PROP_FPS)
    # 总帧数(frames)
    len_frame = int(videoCapture.get(cv2.CAP_PROP_FRAME_COUNT))
    time.sleep(0.5)

    print('fps:', fps)
    print('len_imu:', len_imu)
    print('len_frame:', len_frame)
    frame_time = []
    for i in range(len_frame):
        time_temp = imu_data[int(len_imu/len_frame*i), 0]
        frame_time.append(time_temp)
    frame_time = np.array(frame_time).reshape(-1, 1)
    np.savetxt(path+item.split('_')[0]+'_frame_merge_time.txt', frame_time)