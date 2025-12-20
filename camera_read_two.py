import copy
import os.path
import numpy as np
import cv2
import time
from stereo_camera import StereoCamera
import open3d as o3d
import matplotlib.pyplot as plt
# np.set_printoptions(threshold=np.inf)

#统一单位为cm
class CameraRead:
    def __init__(self, WIDTH=480, HEIGHT=640):
        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT
        self.capture_usb1 = cv2.VideoCapture(0)
        self.capture_usb2 = cv2.VideoCapture(1)
        read_code1, frame1 = self.capture_usb1.read()
        read_code2, frame2 = self.capture_usb2.read()
        print("read_code1, read_code2:", read_code1, read_code2)
        # # 打开自带的摄像头
        # if capture_usb1.isOpened():
        #     if capture_usb1.isOpened():
        #         # 以下设置显示屏的宽高
        #         capture_usb1.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        #         capture_usb1.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        #         capture_usb2.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        #         capture_usb2.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.save_flag = False
        self.frame_index = 0
        self.save_i = 100
        self.fps = 0
        self.fps_time = time.time()
        self.start_time = time.time()
        # 持续读取摄像头数据
        self.SC = StereoCamera()
        self.frame1_seq = np.zeros((10, self.HEIGHT, self.WIDTH, 3))
        self.frame2_seq = np.zeros((10, self.HEIGHT, self.WIDTH, 3))
        self.x = np.zeros((self.HEIGHT, self.WIDTH))
        self.y = np.zeros((self.HEIGHT, self.WIDTH))
        self.depth = np.zeros((self.HEIGHT, self.WIDTH))
        self.points = np.zeros((self.HEIGHT, self.WIDTH, 3))
        self.frame2points = np.zeros((3, self.HEIGHT, self.WIDTH, 3))
        self.frame1s = np.zeros((3, self.HEIGHT, self.WIDTH, 3))
        self.frame2s = np.zeros((3, self.HEIGHT, self.WIDTH, 3))
        self.frame2point = np.zeros((self.HEIGHT, self.WIDTH, 3))
        self.frame1 = np.zeros((self.HEIGHT, self.WIDTH, 3))
        self.frame2 = np.zeros((self.HEIGHT, self.WIDTH, 3))

    ### 图像序列滤波
    # input: n x height x width x 3
    def filterFrames(self, frames):
        print("frames.shape:", frames.shape)
        filtered_frame = np.zeros((self.HEIGHT, self.WIDTH, frames.shape[3]))
        y = frames[:, :, :, 0]
        x = frames[:, :, :, 1]
        depth = frames[:, :, :, 2]

        # filtered_frame[:, :, 0] = y[-1, :, :]
        # filtered_frame[:, :, 1] = x[-1, :, :]
        # filtered_frame[:, :, 2] = depth[-1, :, :]
        filtered_frame[:, :, 0] = np.mean(y, axis=0)  # bias=3 cm
        filtered_frame[:, :, 1] = np.mean(x, axis=0)
        filtered_frame[:, :, 2] = np.max(depth, axis=0)
        return filtered_frame

        ######################## o3d 点云显示 #########################
        # points_o3d = self.points.reshape(-1, 3)
        # points_o3d = np.delete(points_o3d, np.where(points_o3d[:, 2] < 0.1), axis=0)
        # if points_o3d.shape[0] > 2000:  # 降采样
        #     random_list = np.random.randint(0, points_o3d.shape[0], 2000)
        #     points_o3d = points_o3d[random_list]
        # # print(points.shape, points_o3d.shape)
        # self.pcd.points = o3d.utility.Vector3dVector(points_o3d)

    ### DBSCAN 无监督聚类
    # output: nx3, nx1, (数量从大到小的类, 标签从0开始)
    def clusterPoints(self, points):
        cut_width = 100
        points[:cut_width, :, :] = 0
        points[-cut_width:, :, :] = 0
        points[:cut_width, :, :] = 0
        points[-cut_width:, :, :] = 0

        points_o3d = points.reshape(-1, 3)
        points_o3d = np.delete(points_o3d, np.where(points_o3d[:, 2] < 0.1), axis=0)
        # 降采样
        if points_o3d.shape[0] > 2000:
            random_list = np.random.randint(0, points_o3d.shape[0], 2000)
            points_o3d = points_o3d[random_list]
        # print(points.shape, points_o3d.shape)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_o3d)
        labels = np.array(pcd.cluster_dbscan(eps=0.5, min_points=15))
        print("points_o3d.shape:", points_o3d.shape)
        print("labels.shape:", labels.shape)
        if len(labels) > 0:
            max_label = labels.max()
            print("max_label:", max_label)
            centers = np.zeros((max_label, 3))
            ######################## 从多到少排序 ##########################
            labels_number = np.zeros(max_label)
            for i in range(max_label):
                labels_number[i] = np.sum(labels == i)
            labels_number_sorts_index = np.argsort(labels_number)[::-1]
            print("labels_number:", labels_number)
            print("labels_number_sorts_index:", labels_number_sorts_index)
            labels_copy = np.copy(labels)
            for i in range(max_label):
                temp_index_list = np.where(labels_copy == labels_number_sorts_index[i])  # 最多点的簇内点的索引
                labels[temp_index_list] = i
                # print("temp_index_list:", temp_index_list)
                # print("mean:", np.mean(points_o3d[temp_index_list], axis=0))
                # print(centers[i, :].shape)
                # print(np.mean(points_o3d[temp_index_list], axis=0).shape)
                centers[i, :] = np.mean(points_o3d[temp_index_list], axis=0).reshape(1, 3)
            print(f"point cloud has {max_label + 1} clusters")
        else:
            centers = np.zeros((1, 3))

        # ######################## o3d 点云显示 #########################
        # print("labels_number:", labels_number)
        # print(f"point cloud has {max_label + 1} clusters")
        # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        # colors[labels < 0] = 0
        # self.pcd = o3d.geometry.PointCloud()
        # self.pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        # o3d.visualization.draw_geometries([self.pcd.points])
        return points_o3d, labels, centers

    def readRawFrame(self):
        read_code1, frame1 = self.capture_usb1.read()
        read_code2, frame2 = self.capture_usb2.read()
        # print("read_code1, read_code2:", read_code1, read_code2)
        if read_code1 and read_code2:
            # print("self.frame_index:", self.frame_index)
            frame1 = cv2.rotate(frame1, cv2.ROTATE_90_COUNTERCLOCKWISE)
            frame2 = cv2.rotate(frame2, cv2.ROTATE_90_COUNTERCLOCKWISE)
            self.frame1, self.frame2 = self.SC.epipolarRectification(frame1, frame2)  # 极线校正
            return self.frame1, self.frame2

    def readFrame(self):
        ##############################################################
        self.capture_usb1 = cv2.VideoCapture(1)
        self.capture_usb2 = cv2.VideoCapture(0)
        while True:
            read_code1, frame1 = self.capture_usb1.read()
            read_code2, frame2 = self.capture_usb2.read()
            print("read_code1, read_code2:", read_code1, read_code2)
            if read_code1 and read_code2:
                print("self.frame_index:", self.frame_index)
                frame1 = cv2.rotate(frame1, cv2.ROTATE_90_COUNTERCLOCKWISE)
                frame2 = cv2.rotate(frame2, cv2.ROTATE_90_COUNTERCLOCKWISE)
                self.frame1, self.frame2 = self.SC.epipolarRectification(frame1, frame2)  # 极线校正
                ######################## 计算帧率 #########################
                if self.frame_index % 1 == 0:
                    delta_time = time.time() - self.fps_time
                    self.fps_time = time.time()
                    if delta_time > 0:
                        self.fps = 1 / delta_time
                    else:
                        self.fps = 0
                    print("self.fps:", self.fps)
                ##########################################################
                frame1_filtered = self.filterSingleImage(self.frame1)  # Canny边缘提取与原图像掩膜后
                frame2_filtered = self.filterSingleImage(self.frame2)
                disparity1, disparity2 = self.SC.stereoMatchSGBM(frame1_filtered, frame2_filtered)  # 视差计算
                self.frame2point = self.SC.getDepthMapWithQ(disparity1)  # 深度计算
                self.frame_index += 1

            if self.frame_index >= 1:
                self.frame_index = 0
                self.capture_usb1.release()
                self.capture_usb2.release()
                break
        print("return frame")
        return self.frame2point, self.frame1, self.frame2  # return nxhxwx3

    def readFrames(self):
        ##############################################################
        # self.capture_usb1 = cv2.VideoCapture(0)
        # self.capture_usb2 = cv2.VideoCapture(1)
        while True:
            read_code1, frame1 = self.capture_usb1.read()
            read_code2, frame2 = self.capture_usb2.read()
            print("read_code1, read_code2:", read_code1, read_code2)
            if read_code1 and read_code2:
                print("self.frame_index:", self.frame_index)
                frame1 = cv2.rotate(frame1, cv2.ROTATE_90_COUNTERCLOCKWISE)
                frame2 = cv2.rotate(frame2, cv2.ROTATE_90_COUNTERCLOCKWISE)
                frame1, frame2 = self.SC.epipolarRectification(frame1, frame2)  # 极线校正
                ######################## 显示极线校正后的图像 #########################
                if self.frame_index % 1 == 0:
                    delta_time = time.time() - self.fps_time
                    self.fps_time = time.time()
                    if delta_time > 0:
                        self.fps = 1 / delta_time
                    else:
                        self.fps = 0
                    print("self.fps:", self.fps)
                # imgStack = np.hstack((frame1, frame2))  # 相同大小图像水平拼接
                # imgStack = imgStack  # : cv2颜色是 0-255
                # cv2.putText(imgStack, str(self.fps), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255),
                #             thickness=2)
                # cv2.imshow("Camera", imgStack)  # 在窗口 "Demo4" 显示图像 imgStack
                # cv2.moveWindow("Camera", 0, 0)
                ###################################################################
                frame1_filtered = self.filterSingleImage(frame1)  # Canny边缘提取与原图像掩膜后
                frame2_filtered = self.filterSingleImage(frame2)
                # disparity1, disparity2 = self.SC.stereoMatchSGBM(frame1, frame2)  # 视差计算
                disparity1, disparity2 = self.SC.stereoMatchSGBM(frame1_filtered, frame2_filtered)  # 视差计算
                points = self.SC.getDepthMapWithQ(disparity1)  # 深度计算
                # points[:, :, 0] = cv2.medianBlur(points[:, :, 0], 5)  # 滤波效果比较好
                # points[:, :, 1] = cv2.medianBlur(points[:, :, 1], 5)  # 滤波效果比较好
                # points[:, :, 2] = cv2.medianBlur(points[:, :, 2], 5)  # 滤波效果比较好

                self.frame2points[self.frame_index, :, :, :] = points
                self.frame1s[self.frame_index, :, :, :] = frame1
                self.frame2s[self.frame_index, :, :, :] = frame2
                self.frame_index += 1

            if self.frame_index == 3:
                self.frame_index = 0
                break

        return self.frame2points, self.frame1s, self.frame2s  # return nxhxwx3

    def stopCamera(self):
        # 释放资源
        self.capture_usb1.release()
        cv2.destroyWindow("left")
        self.capture_usb2.release()
        cv2.destroyWindow("right")

    def fft(self, img):
        print("img.shape:", img.shape)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        print("img_gray.shape:", img_gray.shape)
        f = np.fft.fft2(img_gray)
        # shift 将低频移到中间
        fshift = np.fft.fftshift(f)
        rows, cols = fshift.shape
        mid_x, mid_y = int((rows) / 2), (int((cols) / 2))
        # 高通
        width = 5
        mask1 = np.ones((rows, cols), dtype=np.uint8)
        mask1[mid_x - width:mid_x + width, :] = 0
        mask1[:, mid_y - width:mid_y + width] = 0
        fshift1 = mask1 * fshift
        isshift1 = np.fft.ifftshift(fshift1)

        # 低通蒙板
        mask2 = np.zeros((rows, cols), dtype=np.uint8)
        width = 10
        # mask2[mid_x - width:mid_x + width, :] = 1
        # mask2[:, mid_y - width:mid_y + width] = 1
        mask2[mid_x - width:mid_x + width, mid_y - width:mid_y + width] = 1
        fshift2 = mask2 * fshift
        isshift2 = np.fft.ifftshift(fshift2)

        high = np.fft.ifft2(isshift1)
        low = np.fft.ifft2(isshift2)

        img_high = np.abs(high).astype(np.uint8)
        img_low = np.abs(low).astype(np.uint8)

        # 展示
        fshift1[fshift1 == 0j] = 1
        fshift1 = np.log(np.abs(fshift1))
        fshift2[fshift2 == 0j] = 1
        fshift2 = np.log(np.abs(fshift2))

        # plt.subplot(321)
        # plt.imshow(img_gray, 'gray')
        # plt.title('Raw')
        # plt.subplot(322)
        # # plt.imshow(np.log(np.abs(f)), 'gray')
        # # plt.title('Raw fft')
        # plt.imshow(np.log(np.abs(fshift)), 'gray')
        # plt.title('Fshift raw fft')
        #
        # plt.subplot(323)
        # plt.imshow(img_high, 'gray')
        # plt.title('HPF')
        # plt.subplot(324)
        # plt.imshow(fshift1, 'gray')
        # plt.title('HPF fft')
        #
        # plt.subplot(325)
        # plt.imshow(img_low, 'gray')
        # plt.title('LPF')
        # plt.subplot(326)
        # plt.imshow(fshift2, 'gray')
        # plt.title('LPF fft')
        # plt.show()
        return img_low

    def filterSingleImage(self, image):
        r = 3
        threshold1 = 20  # 用于边缘链接
        threshold2 = 70  # 用于强边缘的检测
        blurred = cv2.GaussianBlur(image, (r, r), 0)  # 高斯矩阵的长与宽都是r，标准差为0
        # image = image - blurred
        mask1 = cv2.Canny(blurred, threshold1, threshold2)
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        # mask1 = cv2.erode(mask1, kernel)  # 腐蚀图像
        # mask1 = cv2.erode(mask1, kernel)  # 腐蚀图像
        # mask1 = cv2.dilate(mask1, kernel)  # 膨胀图像
        mask1[np.where(mask1 > 1)] = 1
        mask1[np.where(mask1 < 1)] = 0
        mask1 = np.expand_dims(mask1, 2).repeat(3, axis=2)
        image = np.multiply(image, mask1)
        return image


    def test(self):
        ##############################################################
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name='3D points', width=480, height=640, left=1000, top=100)
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0, 0, 0])
        render_option = vis.get_render_option()  # 设置点云渲染参数
        render_option.point_size = 2
        pcd = o3d.geometry.PointCloud()
        pcd_center = o3d.geometry.PointCloud()
        vis.add_geometry(pcd)
        axis_pcd = o3d.geometry.TriangleMesh().create_coordinate_frame(size=5, origin=[0, 0, 0])
        vis.add_geometry(axis_pcd)
        reset_view_point_flag = True
        ##############################################################

        while True:
            self.frame_index += 1
            print("self.frame_index:", self.frame_index)
            read_code1, frame1 = self.capture_usb1.read()
            read_code2, frame2 = self.capture_usb2.read()
            print("read_code1, read_code2:", read_code1, read_code2)
            if read_code1 and read_code2:
                frame1 = cv2.rotate(frame1, cv2.ROTATE_90_COUNTERCLOCKWISE)
                frame2 = cv2.rotate(frame2, cv2.ROTATE_90_COUNTERCLOCKWISE)
                frame1, frame2 = self.SC.epipolarRectification(frame1, frame2)  # 极线校正

                print("frame1.shape:", frame1.shape)
                print("frame1.max():", frame1.max())

                frame1 = self.filterSingleImage(frame1)
                frame2 = self.filterSingleImage(frame2)
                # # frame1_fft = self.fft(frame1)
                # # frame2_fft = self.fft(frame2)

                self.disparity1, self.disparity2 = self.SC.stereoMatchSGBM(frame1, frame2)  # 视差计算
                self.points = self.SC.getDepthMapWithQ(self.disparity1)  # 深度计算
                # self.points[:, :, 0] = cv2.medianBlur(self.points[:, :, 0], 5)
                # self.points[:, :, 1] = cv2.medianBlur(self.points[:, :, 1], 5)
                # self.points[:, :, 2] = cv2.medianBlur(self.points[:, :, 2], 5)  # 滤波效果比较好
                self.x = self.points[:, :, 0]
                self.y = self.points[:, :, 1]
                self.depth = self.points[:, :, 2]

                if self.frame_index <= self.frame2points.shape[0]:
                    self.frame2points[self.frame_index-1, :, :] = self.points
                else:
                    ########################## 图像序列滤波 ########################
                    self.frame2points[:-1, :, :, :] = self.frame2points[1:, :, :, :]
                    self.frame2points[-1, :, :, :] = self.points
                    # self.points = self.filterFrames(self.frame2points)
                    points = self.points

                    ########################## 双目图像显示 ########################
                    if self.frame_index % 1 == 0:
                        delta_time = time.time() - self.fps_time
                        self.fps_time = time.time()
                        if delta_time > 0:
                            self.fps = 1/delta_time
                        else:
                            self.fps = 0
                        print("self.fps:", self.fps)
                    # print(frame1.shape, frame2.shape, depth.shape)
                    imgStack = np.hstack((frame1, frame2))  # 相同大小图像水平拼接
                    # 显示帧率fps
                    # cv2.putText(imgStack, str(self.fps), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255), thickness=2)
                    cv2.imshow("Camera", imgStack)  # 在窗口 "Demo4" 显示图像 imgStack
                    # cv2.moveWindow("Camera", 0, 0)

                    # imgStack = np.hstack((frame1_fft, frame2_fft))  # 相同大小图像水平拼接
                    # cv2.imshow("Camera FFT", imgStack)  # 在窗口 "Demo4" 显示图像 imgStack
                    # # cv2.moveWindow("Camera FFT", 100, 100)

                    disparity1_show = self.disparity1/np.max(self.disparity1)  # : cv2颜色是 0-255
                    depth_show = self.points[:, :, 2]/np.max(self.points[:, :, 2])
                    self.disparity_and_depth = np.hstack((disparity1_show, depth_show))  # 相同大小图像水平拼接
                    cv2.imshow("Disparity1 and Depth", self.disparity_and_depth)  # 在窗口 "Demo4" 显示图像 imgStack
                    cv2.setMouseCallback('Disparity1 and Depth', self.mouseEvent)
                    # print("disparity1_show.shape:", disparity1_show.shape)
                    # cv2.imshow("Disparity1", disparity1_show)  # 在窗口 "Demo4" 显示图像 imgStack
                    # print("depth_show.shape:", depth_show.shape)
                    # cv2.imshow("Depth", depth_show)  # 在窗口 "Demo4" 显示图像 imgStack

                    ########################## o3d 点云聚类 ############################
                    points_o3d, labels, centers = self.clusterPoints(points)  # DBSCAN 无监督聚类
                    print("labels:", labels)
                    pcd.points = o3d.utility.Vector3dVector(points_o3d)
                    ### 簇中心
                    pcd_center.points = o3d.utility.Vector3dVector(centers)
                    center_colors = np.ones((centers.shape[0], 3))
                    # center_colors[:, 0] = 1  # 簇中心渲染为红色
                    pcd_center.colors = o3d.utility.Vector3dVector(center_colors)
                    ###
                    if labels.shape[0] > 0:
                        max_label = np.max(labels)
                        ######################## o3d 点云显示 #########################
                        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
                        colors[labels < 0] = 0
                        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

                    ########################## o3d 点云显示 ############################
                    vis.update_geometry(pcd)
                    vis.update_geometry(axis_pcd)
                    if reset_view_point_flag == True:
                        vis.reset_view_point(True)
                        ctr = vis.get_view_control()
                        # ctr.set_lookat(np.array([0.0, 0.0, 0.0]))
                        ctr.set_up((0, -1, 0))  # set the positive direction of the x-axis as the up direction
                        ctr.set_front((0, 0, -1))  # set the positive direction of the x-axis toward you
                        reset_view_point_flag = False
                    vis.poll_events()
                    vis.update_renderer()

                    ######################### o3d 点云形状分割 ############################
                    # # 3D Shape Detection with RANSAC
                    # plane_model, inliers = pcd.segment_plane(distance_threshold=500, ransac_n=3, num_iterations=100)
                    # [a, b, c, d] = plane_model
                    # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
                    # inlier_cloud = pcd.select_by_index(inliers)
                    # outlier_cloud = pcd.select_by_index(inliers, invert=True)
                    # inlier_cloud.paint_uniform_color([1.0, 0, 0])
                    # outlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])
                    ################ RANSAC loop for Multiple plane shapes detection #################
                    # segment_models = {}
                    # segments = {}
                    # max_plane_idx = 10
                    # rest = pcd
                    # for i in range(max_plane_idx):
                    #     colors = plt.get_cmap("tab20")(i)
                    #     segment_models[i], inliers = rest.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
                    #     segments[i] = rest.select_by_index(inliers)
                    #     segments[i].paint_uniform_color(list(colors[:3]))
                    #     rest = rest.select_by_index(inliers, invert=True)
                    #     print("pass", i, "/", max_plane_idx, "done.")

                    ##############################################################

                    # 输入 s 键，保存当前画面为图片
                    if cv2.waitKey(1) == ord('s'):
                        self.save_flag = True
                        # 设置图片分辨率
                        # frame = cv2.resize(frame1, (1920, 1080))
                        self.save_i += 1
                    elif cv2.waitKey(1) == ord('q'):
                        self.start_time = time.time()
                        self.save_flag = False
                        self.save_i = 0

                    if self.save_flag:
                        self.save_flag = False
                        path1 = './camera_data/' + str(self.start_time) + '_left/'
                        path2 = './camera_data/' + str(self.start_time) + '_right/'
                        if not os.path.exists(path1):
                            os.mkdir(path1)
                        if not os.path.exists(path2):
                            os.mkdir(path2)
                        cv2.imwrite(path1+str(self.save_i)+'.jpg', frame1)  # frame1 is left
                        cv2.imwrite(path2+str(self.save_i)+'.jpg', frame2)  # frame2 is right
                    # cv2.imwrite('camera_calibration/images/left/'+str(save_i)+'.jpg', frame1)  # frame1 is left
                        # cv2.imwrite('camera_calibration/images/right/'+str(save_i)+'.jpg', frame2)  # frame2 is right
                        # cv2.imwrite('images/'+str(save_i)+'_left.jpg', frame1)  # frame1 is left
                        # cv2.imwrite('images/'+str(save_i)+'_right.jpg', frame2)  # frame2 is right

        # 释放资源
        self.capture_usb1.release()
        self.capture_usb2.release()

    def showFrame(self):
        while True:
            self.frame_index += 1
            print("self.frame_index:", self.frame_index)
            read_code1, frame1 = self.capture_usb1.read()
            read_code2, frame2 = self.capture_usb2.read()
            frame1 = cv2.rotate(frame1, cv2.ROTATE_90_COUNTERCLOCKWISE)
            frame2 = cv2.rotate(frame2, cv2.ROTATE_90_COUNTERCLOCKWISE)

            if self.frame_index % 10 == 0:
                delta_time = time.time() - self.fps_time
                self.fps_time = time.time()
                if delta_time > 0:
                    self.fps = int(10 / delta_time)
                else:
                    self.fps = 0
                print("self.fps:", self.fps)
            # print(frame1.shape, frame2.shape, depth.shape)
            imgStack = np.hstack((frame1, frame2))  # 相同大小图像水平拼接
            imgStack = imgStack  # : cv2颜色是 0-255
            cv2.putText(imgStack, str(self.fps), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255),
                        thickness=2)
            cv2.imshow("Camera", imgStack)  # 在窗口 "Demo4" 显示图像 imgStack
            # cv2.moveWindow("Camera", 0, 0)
            cv2.waitKey(1)

        # 释放资源
        self.capture_usb1.release()
        cv2.destroyWindow("left")
        self.capture_usb2.release()
        cv2.destroyWindow("right")

    def mouseEvent(self, event, x, y, flags, param=None):
        y = y - self.depth.shape[0]
        x = x - self.depth.shape[1]
        if event == cv2.EVENT_LBUTTONDBLCLK:
            # print x,y,disp[y,x],filteredImg[y,x]
            average = 0
            for u in range(-1, 2):
                for v in range(-1, 2):
                    average += self.depth[y + u, x + v]
            pos_x = self.x[y, x]
            pos_y = self.y[y, x]
            Distance = average / 9
            Distance = np.around(Distance, decimals=2)
            print("pos:", str(pos_x)+', '+str(pos_y)+', '+str(Distance) + 'mm')
            print("disparity1:", self.disparity1[y, x])
            gbr = (1, 1, 0)
            cv2.circle(self.disparity_and_depth, (x, y), radius=3, color=gbr, thickness=-1)  # 绘制实心圆：thickness=-1
            cv2.putText(self.disparity_and_depth, "xyz: " + str(pos_x) + ', ' + str(pos_y) + ', ' + str(Distance), (self.depth.shape[1]+20, 40), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=gbr, thickness=2)
            cv2.putText(self.disparity_and_depth, "disparity: " + str(self.disparity1[y, x]), (self.depth.shape[1]+20, 60), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=gbr, thickness=2)
            cv2.imshow("Disparity1 and Depth", self.disparity_and_depth)  # 在窗口 "Demo4" 显示图像 imgStack
            cv2.waitKey(1)

if __name__ == '__main__':
    camera = CameraRead()
    # camera.showFrame()
    camera.test()