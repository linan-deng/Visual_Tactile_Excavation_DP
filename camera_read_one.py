import copy
import cv2
import numpy as np
import time

capture_usb1 = cv2.VideoCapture(0)
### 打开自带的摄像头
if capture_usb1.isOpened():
    # 以下设置显示屏的宽高
    capture_usb1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture_usb1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

frame_i = 0
fps = 0
start_time = time.time()
# 持续读取摄像头数据
while True:

    frame_i += 1
    read_code1, frame1 = capture_usb1.read()
    frame1 = cv2.flip(frame1, -1)
    b, g, r = cv2.split(frame1)
    if not read_code1:
        break
    print("frame1.shape:", frame1.shape)
    print("b.shape:", b.shape)
    time_stamp = time.time()
    # imgStack = np.hstack((frame1, b))  # 相同大小图像水平拼接
    frame1_rgb = np.hstack((r, g, b))  # 相同大小图像水平拼接
    # if frame_i % 5 == 0:
    #     cv2.imwrite("./img/l_" + str(time_stamp) + ".jpg", frame1)
    #     cv2.imwrite("./img/r_" + str(time_stamp) + ".jpg", frame2)
    #     cv2.imwrite("./img/c_" + str(time_stamp) + ".jpg", imgStack)

    if frame_i % 10 == 0:
        delta_time = time.time() - start_time
        if delta_time > 0:
            fps = int(10/delta_time)
        else:
            fps = 0
        print("fps:", fps)
        start_time = time.time()
    cv2.putText(frame1, str(fps), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0,0,255), thickness=2)

    cv2.imshow("Camera", frame1)  # 在窗口 "Demo4" 显示图像 imgStack
    # cv2.moveWindow("Camera", 0, 0)
    cv2.imshow("frame1_rgb", frame1_rgb)  # 在窗口 "Demo4" 显示图像 imgStack

    # 输入 s 键，保存当前画面为图片
    if cv2.waitKey(1) == ord('s'):
        # 设置图片分辨率
        # frame = cv2.resize(frame1, (1920, 1080))
        cv2.imwrite('camera_img/'+str(frame_i)+'_frame1.jpg', frame1)


# 释放资源
capture_usb1.release()
cv2.destroyWindow("left")