import copy
import time
import cv2
import numpy as np
import os
# import open3d
# np.set_printoptions(threshold=np.inf)

class StereoCamera:
    def __init__(self, WIDTH=480, HEIGHT=640):
        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT
        self.capture_usb1 = cv2.VideoCapture(0)
        self.capture_usb2 = cv2.VideoCapture(1)
        self.initCameraParameters()

        self.save_flag = False
        self.save_i = 0
        self.fps = 0
        self.start_time = time.time()

    def initCameraParameters(self):
        # 相机内参, 即上文标定得到的 cameraMatrix1
        self.cameraMatrix1 = np.array([[652.89818638,   0.        , 249.35767636],
       [  0.        , 654.15483976, 326.97452409],
       [  0.        ,   0.        ,   1.        ]])
        # 相机畸变系数, 即上文标定得到的 distCoeffs1
        self.distCoeffs1 = np.array([[-7.76145992e-02,  1.00900508e+00, -2.52057793e-03,
         9.94254550e-04, -3.90896779e+00]])

        # 即上文标定得到的 cameraMatrix2
        self.cameraMatrix2 = np.array([[647.89883112,   0.        , 259.67804566],
       [  0.        , 649.92013826, 301.27518992],
       [  0.        ,   0.        ,   1.        ]])
        # 即上文标定得到的 distCoeffs2
        self.distCoeffs2 = np.array([[-1.07921674e-01,  8.71540226e-01, -4.18208170e-03,
         1.80484959e-03, -2.23093519e+00]])

        # 旋转矩阵,即上文标定得到的R
        self.R = np.array([[ 0.99966948,  0.00833614, -0.02431934],
       [-0.0080791 ,  0.99991066,  0.01064862],
       [ 0.02440594, -0.01044862,  0.99964753]])
        # 平移矩阵,第一个值为基线距离,即上文标定得到的T
        self.T = np.array([[-4.99984205],
       [ 0.14406776],
       [-0.87300846]])

        # 焦距为Q[2, 3]
        self.R_l, self.R_r, self.P_l, self.P_r, self.Q, self.validPixROI1, self.validPixROI2 = \
            cv2.stereoRectify(self.cameraMatrix1, self.distCoeffs1, self.cameraMatrix2, self.distCoeffs2, (self.WIDTH, self.HEIGHT), self.R, self.T)  # 计算旋转矩阵和投影矩阵
        print("self.Q", repr(self.Q))

        self.Q = np.array([[ 1.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        -1.33436422e+02],
       [ 0.00000000e+00,  1.00000000e+00,  0.00000000e+00,
        -3.10969093e+02],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         6.52037489e+02],
       [ 0.00000000e+00,  0.00000000e+00,  1.96946119e-01,
        -0.00000000e+00]])

        # 左右图需要分别计算校正查找映射表以及重映射
        self.map11, self.map12 = cv2.initUndistortRectifyMap(self.cameraMatrix1, self.distCoeffs1, self.R_l, self.P_l, (self.WIDTH, self.HEIGHT), cv2.CV_32FC1)  # 计算校正查找映射表
        self.map21, self.map22 = cv2.initUndistortRectifyMap(self.cameraMatrix2, self.distCoeffs2, self.R_r, self.P_r, (self.WIDTH, self.HEIGHT), cv2.CV_32FC1)


    def saveImage(self):
        self.save_i += 1
        read_code1, frame1 = self.capture_usb1.read()
        read_code2, frame2 = self.capture_usb2.read()
        # print("read_code1, read_code2:", read_code1, read_code2)
        frame1 = cv2.rotate(frame1, cv2.ROTATE_90_COUNTERCLOCKWISE)
        frame2 = cv2.rotate(frame2, cv2.ROTATE_90_COUNTERCLOCKWISE)
        # imgStack_raw = np.hstack((frame2, frame1))  # 相同大小图像水平拼接
        # cv2.imshow("Camera_raw", imgStack_raw)  # 在窗口 "Demo4" 显示图像 imgStack_raw
        frame1, frame2 = self.epipolarRectification(frame1, frame2)  # 极线校正
        # disparity1, disparity2 = self.stereoMatchSGBM(frame1, frame2)  # 视差计算
        # depth1 = self.getDepthMapWithQ(disparity1)  # 深度计算
        # depth2 = self.getDepthMapWithQ(disparity2)  # 深度计算
        # print(depth1)
        # frame1 = cv2.flip(frame1, -1)
        # frame2 = cv2.flip(frame2, -1)
        # frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        # frame2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
        # print("frame1.shape:", frame1.shape)
        if not read_code1 or not read_code2:
            print("No image read!!!")
            pass
        else:
            path1 = './camera_data/' + str(self.start_time) + '_left/'
            path2 = './camera_data/' + str(self.start_time) + '_right/'
            if not os.path.exists(path1):
                os.mkdir(path1)
            if not os.path.exists(path2):
                os.mkdir(path2)
            cv2.imwrite(path1 + str(self.save_i) + '.jpg', frame1)  # frame1 is left
            cv2.imwrite(path2 + str(self.save_i) + '.jpg', frame2)  # frame2 is right

    # 双目相机参数标定
    def stereoCalibration(self):
        leftpath = 'camera_calibration/images/left'
        rightpath = 'camera_calibration/images/right'
        CHECKERBOARD = (8, 5)  # 棋盘格内角点数
        square_size = (2.7, 2.7)  # 棋盘格大小，单位cm
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        imgpoints_l = []  # 存放左图像坐标系下角点位置
        imgpoints_r = []  # 存放左图像坐标系下角点位置
        objpoints = []  # 存放世界坐标系下角点位置
        objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
        objp[0, :, 0] *= square_size[0]
        objp[0, :, 1] *= square_size[1]

        # 双目标定
        for ii in os.listdir(leftpath):
            img_l = cv2.imread(os.path.join(leftpath, ii))
            gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
            img_r = cv2.imread(os.path.join(rightpath, ii))
            gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
            # cv2.imshow('img_l', img_l)
            # cv2.waitKey(1)
            ret_l, corners_l = cv2.findChessboardCorners(gray_l, CHECKERBOARD)  # 检测棋盘格内角点
            print(ret_l, corners_l)
            ret_r, corners_r = cv2.findChessboardCorners(gray_r, CHECKERBOARD)
            if ret_l and ret_r:
                objpoints.append(objp)
                corners2_l = cv2.cornerSubPix(gray_l,corners_l,(11,11),(-1,-1),criteria)
                imgpoints_l.append(corners2_l)
                corners2_r = cv2.cornerSubPix(gray_r,corners_r,(11,11),(-1,-1),criteria)
                imgpoints_r.append(corners2_r)
                # img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2,ret)
                # cv2.imwrite('./ChessboardCornersimg.jpg', img)

        ret, mtx_l, dist_l, rvecs_l, tvecs_l = cv2.calibrateCamera(objpoints, imgpoints_l, gray_l.shape[::-1],None,None)  #先分别做单目标定
        ret, mtx_r, dist_r, rvecs_r, tvecs_r = cv2.calibrateCamera(objpoints, imgpoints_r, gray_r.shape[::-1],None,None)

        retval, self.cameraMatrix1, self.distCoeffs1, self.cameraMatrix2, self.distCoeffs2, self.R, self.T, self.E, self.F = \
            cv2.stereoCalibrate(objpoints, imgpoints_l, imgpoints_r, mtx_l, dist_l, mtx_r, dist_r, gray_l.shape[::-1])   #再做双目标定

        print("stereoCalibrate:")
        print("self.cameraMatrix1:")
        print(repr(self.cameraMatrix1))
        print("self.distCoeffs1:")
        print(repr(self.distCoeffs1))
        print("self.cameraMatrix2:")
        print(repr(self.cameraMatrix2))
        print("self.distCoeffs2:")
        print(repr(self.distCoeffs2))
        print("self.R:")
        print(repr(self.R))
        print("self.T:")
        print(repr(self.T))
        print("self.E:")
        print(repr(self.E))
        print("self.F:")
        print(repr(self.F))

    # 极限校正
    def epipolarRectification(self, left_image, right_image):
        rect_left_image = cv2.remap(left_image, self.map11, self.map12, cv2.INTER_CUBIC)  # 重映射
        rect_right_image = cv2.remap(right_image, self.map21, self.map22, cv2.INTER_CUBIC)
        return rect_left_image, rect_right_image

    # 拼接图片
    def cat2images(self, left_image, right_image):
        HEIGHT = left_image.shape[0]
        WIDTH = right_image.shape[1]
        if len(left_image.shape) == 3:
            imgcat = np.zeros((HEIGHT, WIDTH * 2 + 20, 3))
            imgcat[:, :WIDTH, :] = left_image
            imgcat[:, -WIDTH:, :] = right_image
            for i in range(int(HEIGHT / 32)):
                imgcat[i * 32, :, :] = 255
        if len(left_image.shape) == 2:
            imgcat = np.zeros((HEIGHT, WIDTH * 2 + 20))
            imgcat[:, :WIDTH] = left_image
            imgcat[:, -WIDTH:] = right_image
            for i in range(int(HEIGHT / 32)):
                imgcat[i * 32, :] = 255
        return imgcat

    # 视差计算
    def stereoMatchSGBM(self, left_image, right_image, down_scale=False):
        ########################### 滤波 ########################### 这里滤波结果不好，图像会变得稠密
        # left_image = self.addPoissonNoise(left_image)
        # right_image = self.addPoissonNoise(right_image)
        ###########################################################
        # left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        # right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
        # SGBM匹配参数设置
        if left_image.ndim == 2:
            img_channels = 1
        else:
            img_channels = 3
        print("img_channels:", img_channels)
        min_disp = 0
        num_disp = 144 - min_disp
        blockSize = 5
        paraml = {'minDisparity': min_disp,  # 最小视差值。通常设置为0，这意味着算法将从0开始搜索视差。
                  'numDisparities': num_disp,  # 视差搜索范围。必须是16的整数倍。算法将搜索从minDisparity到minDisparity + numDisparities - 1的视差值
                  'blockSize': blockSize,  # 块的线性大小。大小应该是奇数（因为块位于当前像素的中心）。更大的块大小意味着更平滑，但不太准确的视差图。较小的块大小会给出更详细的视差图，但算法找到错误对应的几率更高。一般在3到11之间。
                  'P1': 8 * img_channels * blockSize ** 2,  # 控制视差平滑度的第一个参数，是相邻像素之间视差变化为1的惩罚。值越大，视差越平滑。
                  'P2': 32 * img_channels * blockSize ** 2,  # 控制视差平滑度的第二个参数，是相邻像素之间视差变化超过1的惩罚。值越大，视差越平滑。该算法要求P2>P1。
                  # 'disp12MaxDiff': 5,  # 左右视差检查中允许的最大差异（以整数像素为单位）。将其设置为非正值以禁用检查。
                  # 'preFilterCap': 63,  # 预滤波图像像素的截断值。该算法首先计算每个像素的x方向的导数，并按[-preFilterCap，preFilterCap]间隔剪裁其值。结果值被传递到Birchfield-Tomasi像素代价函数。
                  # 'uniquenessRatio': 5,  # 最佳（最小）计算成本函数值应超过第二最佳值的百分比，满足此百分比的条件下才认为找到的匹配是正确的。通常，5-15范围内的值就足够好了。
                  # 'speckleWindowSize': 100,  # 考虑其噪声斑点的平滑差距区域的最大尺寸，并使之无效。把它设置为0以禁用斑点过滤。否则，将它设置在50-200范围内的某个地方。
                  # 'speckleRange': 0,  # 每个连接组件内的最大视差变化。如果进行斑点过滤，将参数设置为正值，它将被隐式地乘以16。通常，1或2就足够了。
                  # 'mode': False  # 默认情况下，它设置为false。若将其设置为MODE_HH，将运行完整的双过程动态规划算法。
                  }
        # 构建SGBM对象
        left_matcher = cv2.StereoSGBM_create(**paraml)
        paramr = paraml
        paramr['minDisparity'] = -paraml['numDisparities']
        # right_matcher = cv2.StereoSGBM_create(**paramr)

        # Used for the filtered image
        right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)  # Create another stereo for right this time

        # 计算视差图
        size = (left_image.shape[1], left_image.shape[0])
        if down_scale == False:
            print("left_image.shape, right_image.shape:", left_image.shape, right_image.shape)
            disparity_left = left_matcher.compute(left_image, right_image)
            disparity_right = right_matcher.compute(right_image, left_image)

        else:
            left_image_down = cv2.pyrDown(left_image)
            right_image_down = cv2.pyrDown(right_image)
            factor = left_image.shape[1] / left_image_down.shape[1]

            disparity_left_half = left_matcher.compute(left_image_down, right_image_down)
            disparity_right_half = right_matcher.compute(right_image_down, left_image_down)
            disparity_left = cv2.resize(disparity_left_half, size, interpolation=cv2.INTER_AREA)
            disparity_right = cv2.resize(disparity_right_half, size, interpolation=cv2.INTER_AREA)
            disparity_left = factor * disparity_left
            disparity_right = factor * disparity_right

        ###################################### WLS FILTER ####################################
        # WLS FILTER Parameters
        lmbda = 100  # 权重参数，用于平衡数据项和平滑项。较大的lambda_值会导致更平滑的视差图，但可能会降低视差图的精度。
        sigma = 0.3  # 颜色差异的标准差，用于计算权重。较大的sigma_color值意味着颜色差异较大的像素将具有较小的权重，从而减少它们对最终视差图的影响。
        wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
        wls_filter.setLambda(lmbda)
        wls_filter.setSigmaColor(sigma)
        # Using the WLS filter
        disparity_left = wls_filter.filter(disparity_left, left_image, None, disparity_right)
        disparity_left = disparity_left.astype(np.float32) / 16.
        ######################################################################################

        ################## Filtering the Results with a closing filter #####################
        # kernel = np.ones((5, 5), np.uint8)
        # disparity_left = cv2.morphologyEx(disparity_left, cv2.MORPH_CLOSE, kernel)  # Apply an morphological filter for closing little "black" holes in the picture(Remove noise)
        ####################################################################################

        # # Colors map
        # dispc = (closing - closing.min()) * 255
        # dispC = dispc.astype(
        #     np.uint8)  # Convert the type of the matrix from float32 to uint8, this way you can show the results with the function cv2.imshow()
        # disp_Color = cv2.applyColorMap(dispC,
        #                                cv2.COLORMAP_OCEAN)  # Change the Color of the Picture into an Ocean Color_Map
        # filt_Color = cv2.applyColorMap(disparity_filtered, cv2.COLORMAP_OCEAN)

        reset_index = np.where(disparity_left < 0.0)
        disparity_left[reset_index] = 0
        trueDisp_left = disparity_left

        trueDisp_right = disparity_right.astype(np.float32)

        # # 真实视差（因为SGBM算法得到的视差是×16的）
        # trueDisp_left = disparity_left.astype(np.float32) / 16.
        # trueDisp_right = disparity_right.astype(np.float32) / 16.
        # # print("trueDisp_left:", trueDisp_left)
        # # print("trueDisp_right:", trueDisp_right)
        # print("trueDisp_left:", trueDisp_left)
        # print("np.max(trueDisp_left):", np.max(trueDisp_left))
        return trueDisp_left, trueDisp_right

    # 利用opencv函数计算深度图
    def getDepthMapWithQ(self, disparityMap):
        points_3d = cv2.reprojectImageTo3D(disparityMap, self.Q)  # 单位与内参矩阵标定单位一样，为 cm
        x = points_3d[:, :, 0]-4.5
        y = points_3d[:, :, 1]-1
        depthMap = points_3d[:, :, 2]
        reset_index = np.where(np.abs(depthMap) > 30.0)
        reset_index = np.logical_or(np.abs(depthMap) > 30.0, np.abs(depthMap) < 0.0)
        depthMap[reset_index] = 0
        x[reset_index] = 0
        y[reset_index] = 0
        # depthMap = self.filterMedianBlur(depthMap)  # 去噪
        points_3d[:, :, 0] = x
        points_3d[:, :, 1] = y
        points_3d[:, :, 2] = depthMap
        # print("depthMap:", depthMap)
        # print("np.max(depthMap):", np.max(depthMap))
        return points_3d

    def filterBlur(self, image):
        """
        均值模糊 : 去随机噪声有很好的去噪效果
        （1, 15）是垂直方向模糊，（15， 1）是水平方向模糊
        """
        dst = cv2.blur(image, (3, 3))
        return dst

    def filterMedianBlur(self, image):  # 中值模糊  对椒盐噪声有很好的去燥效果
        dst = cv2.medianBlur(image, 5)
        return dst

    def filterCustomBlur(self, image):
        """
        用户自定义模糊
        下面除以25是防止数值溢出
        """
        kernel = np.ones([5, 5], np.float32) / 25
        dst = cv2.filter2D(image, -1, kernel)
        return dst

    def addGaussNoise(self, image):
        gauss = np.random.normal(0, 3, (image.shape[0], image.shape[1], 3)).astype(np.uint8)
        noisy_img = image + gauss
        return noisy_img

    def addPoissonNoise(self, image):
        # 计算图像像素的分布范围
        vals = len(np.unique(image))
        vals = 1 ** np.ceil(np.log2(vals))
        # 给图片添加泊松噪声
        noisy_img = np.random.poisson(image * vals) / float(vals)
        noisy_img = noisy_img.astype(np.uint8)
        return noisy_img

    def addSaltNoise(self, image):
        s_vs_p = 0.5
        # 设置添加噪声图像像素的数目
        amount = 0.04
        noisy_img = np.copy(image)
        # 添加salt噪声
        num_salt = np.ceil(amount * image.size * s_vs_p)
        # 设置添加噪声的坐标位置
        coords = [np.random.randint(0, i - 1, int(num_salt)) for i in image.shape]
        noisy_img[coords[0], coords[1], :] = [255, 255, 255]
        # 添加pepper噪声
        num_pepper = np.ceil(amount * image.size * (1. - s_vs_p))
        # 设置添加噪声的坐标位置
        coords = [np.random.randint(0, i - 1, int(num_pepper)) for i in image.shape]
        noisy_img[coords[0], coords[1], :] = [0, 0, 0]
        return noisy_img

    def readSave(self):
        capture_usb1 = cv2.VideoCapture(0)
        capture_usb2 = cv2.VideoCapture(1)
        width = 640  # 640
        height = 480  # 480
        # # 打开自带的摄像头
        # if capture_usb1.isOpened():
        #     if capture_usb1.isOpened():
        #         # 以下设置显示屏的宽高
        #         capture_usb1.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        #         capture_usb1.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        #         capture_usb2.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        #         capture_usb2.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        frame_i = 0
        save_i = 0
        fps = 0
        start_time = time.time()
        # 持续读取摄像头数据
        while True:
            frame_i += 1
            read_code1, frame1 = capture_usb1.read()
            read_code2, frame2 = capture_usb2.read()
            frame1 = cv2.rotate(frame1, cv2.ROTATE_90_COUNTERCLOCKWISE)
            frame2 = cv2.rotate(frame2, cv2.ROTATE_90_COUNTERCLOCKWISE)
            print("frame1.shape:", frame1.shape)
            print("read_code1, read_code2:", read_code1, read_code2)
            if not read_code1 or not read_code2:
                break
            if frame_i % 10 == 0:
                delta_time = time.time() - start_time
                if delta_time > 0:
                    fps = int(10 / delta_time)
                else:
                    fps = 0
                print("fps:", fps)
                start_time = time.time()
            imgStack = np.hstack((frame1, frame2))  # 相同大小图像水平拼接
            cv2.putText(imgStack, str(fps), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255),
                        thickness=2)
            cv2.imshow("Camera", imgStack)  # 在窗口 "Demo4" 显示图像 imgStack
            # cv2.moveWindow("Camera", 0, 0)

            # 输入 s 键，保存当前画面为图片
            if cv2.waitKey(1) == ord('s'):
                # 设置图片分辨率
                # frame = cv2.resize(frame1, (1920, 1080))
                save_i += 1
                cv2.imwrite('./camera_calibration/images/left/' + str(save_i) + '.jpg', frame1)  # frame1 is left
                cv2.imwrite('./camera_calibration/images/right/' + str(save_i) + '.jpg', frame2)  # frame2 is right
    # # 根据公式计算深度图
    # def getDepthMapWithConfig(self, disparityMap: np.ndarray, config: stereoconfig.stereoCamera) -> np.ndarray:
    #     fb = config.cam_matrix_left[0, 0] * (-config.T[0])
    #     doffs = config.doffs
    #     depthMap = np.divide(fb, disparityMap + doffs)
    #     reset_index = np.where(np.logical_or(depthMap < 0.0, depthMap > 65535.0))
    #     depthMap[reset_index] = 0
    #     reset_index2 = np.where(disparityMap < 0.0)
    #     depthMap[reset_index2] = 0
    #     return depthMap.astype(np.float32)
    #
    # # 将h×w×3数组转换为N×3的数组
    # def hw3ToN3(self, points):
    #     height, width = points.shape[0:2]
    #
    #     points_1 = points[:, :, 0].reshape(height * width, 1)
    #     points_2 = points[:, :, 1].reshape(height * width, 1)
    #     points_3 = points[:, :, 2].reshape(height * width, 1)
    #
    #     points_ = np.hstack((points_1, points_2, points_3))
    #
    #     return points_
    #
    # # 深度、颜色转换为点云
    # def DepthColor2Cloud(self, points_3d, colors):
    #     rows, cols = points_3d.shape[0:2]
    #     size = rows * cols
    #
    #     points_ = self.hw3ToN3(points_3d)
    #     colors_ = self.hw3ToN3(colors).astype(np.int64)
    #
    #     # 颜色信息
    #     blue = colors_[:, 0].reshape(size, 1)
    #     green = colors_[:, 1].reshape(size, 1)
    #     red = colors_[:, 2].reshape(size, 1)
    #
    #     rgb = np.left_shift(blue, 0) + np.left_shift(green, 8) + np.left_shift(red, 16)
    #
    #     # 将坐标+颜色叠加为点云数组
    #     pointcloud = np.hstack((points_, rgb)).astype(np.float32)
    #
    #     # 删掉一些不合适的点
    #     X = pointcloud[:, 0]
    #     Y = pointcloud[:, 1]
    #     Z = pointcloud[:, 2]
    #
    #     # 下面参数是经验性取值，需要根据实际情况调整
    #     remove_idx1 = np.where(Z <= 0)
    #     remove_idx2 = np.where(Z > 15000) # 注意单位是mm
    #     remove_idx3 = np.where(X > 10000)
    #     remove_idx4 = np.where(X < -10000)
    #     remove_idx5 = np.where(Y > 10000)
    #     remove_idx6 = np.where(Y < -10000)
    #     remove_idx = np.hstack(
    #         (remove_idx1[0], remove_idx2[0], remove_idx3[0], remove_idx4[0], remove_idx5[0], remove_idx6[0]))
    #
    #     pointcloud_1 = np.delete(pointcloud, remove_idx, 0)
    #
    #     return pointcloud_1
    #
    # # 点云显示
    # def view_cloud(self, pointcloud):
    #     cloud = pcl.PointCloud_PointXYZRGBA()
    #     cloud.from_array(pointcloud)
    #
    #     try:
    #         visual = pcl.pcl_visualization.CloudViewing()
    #         visual.ShowColorACloud(cloud)
    #         v = True
    #         while v:
    #             v = not (visual.WasStopped())
    #     except:
    #         pass

if __name__ == '__main__':
    SC = StereoCamera()
    # SC.readSave()
    SC.stereoCalibration()
    # for i in range(21, 22):
    #     number = i
    #     left_image = cv2.imread("camera_data/1726061495.30695_left/" + str(number) + ".jpg")
    #     right_image = cv2.imread("camera_data/1726061495.30695_right/" + str(number) + ".jpg")
    #
    #     imgcat_source = SC.cat2images(left_image, right_image)
    #     HEIGHT = left_image.shape[0]
    #     WIDTH = left_image.shape[1]
    #     cv2.imwrite('camera_data/1726061495.30695_left/imgcat_source_'+str(i)+'.jpg', imgcat_source)
    #
    #     # # SC.stereoCalibration()
    #     # rect_left_image, rect_right_image = SC.epipolarRectification(left_image, right_image)
    #     # imgcat_out = SC.cat2images(rect_left_image, rect_right_image)
    #     # cv2.imwrite('camera_data/1726061495.30695_left/imgcat_out_'+str(i)+'.jpg', imgcat_out)
    #
    #     # SC.stereoMatchSGBM()
    #     trueDisp_left, trueDisp_right = SC.stereoMatchSGBM(left_image, right_image)
    #     imgcat_out = SC.cat2images(trueDisp_left, trueDisp_right)
    #     cv2.imwrite('camera_data/1726061495.30695_left/disparity_'+str(i)+'.jpg', imgcat_out)
    #
    #     # SC.stereoMatchSGBM()
    #     depth_left = SC.getDepthMapWithQ(trueDisp_left)
    #     depth_right = SC.getDepthMapWithQ(trueDisp_right)
    #     imgcat_out = SC.cat2images(depth_left, depth_right)
    #     cv2.imwrite('camera_data/1726061495.30695_left/depth_'+str(i)+'.jpg', imgcat_out)
