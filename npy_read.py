import numpy as np
from camera_read_two import CameraRead
import open3d as o3d
import copy
import cv2
import matplotlib.pyplot as plt
import math
from stereo_camera import StereoCamera

camera = CameraRead()

# input: nx3
# output: data in real world
def calculate_principal_axis(arr):
    if arr.shape[0] == 0:
        arr = np.ones((1, 3))
    arr_mean = np.mean(arr, axis=0)  # 1x3
    arr_norm = arr - arr_mean
    cov = 1 / arr.shape[0] * (arr_norm.T @ arr_norm)
    eigenvalue, featurevector = np.linalg.eig(cov)
    eigenvalue = np.sqrt(eigenvalue)
    eigenvalue_sort_index = np.argsort(eigenvalue)[::-1]
    eigenvalue_sort = eigenvalue[eigenvalue_sort_index]
    featurevector_sort = featurevector[:, eigenvalue_sort_index]
    featurevector_sort[:, 1] = -featurevector_sort[:, 1]
    featurevector_sort[:, 2] = np.cross(featurevector_sort[:, 0], featurevector_sort[:, 1])
    print("eigenvalue_sort:", eigenvalue_sort)
    print("featurevector_sort:", featurevector_sort)
    return np.array(arr_mean).squeeze(), np.array(featurevector_sort), np.array(eigenvalue_sort).squeeze()

# input: nx3, object pos in world (cm)
# output: nx3, object pos in camera (cm)
def pos_world2camera(data, pos_top_world):
    print("data.shape:", data.shape)
    data = copy.copy(data)
    data = data.T
    theta_z = np.pi/2
    Rz = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                   [np.sin(theta_z), np.cos(theta_z), 0],
                   [0, 0, 1]])
    pos_top = pos_top_world
    camera_origin_world = copy.copy(pos_top).reshape(3, 1)
    camera_origin_world[2, 0] = -6-13
    data = Rz @ (data - camera_origin_world)
    pos_top_camera = copy.copy(pos_top) - camera_origin_world.squeeze()
    print("data.shape:", data.shape)
    print("pos_top_camera.shape:", pos_top_camera.shape)
    return data.T, pos_top_camera

# input: nx3, object pos in camera (cm)
# output: nx3, object pos in world (cm)
def pos_camera2world(data, pos_top_world):
    data = copy.copy(data)
    data = data.T
    theta_z = np.pi/2
    Rz = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                   [np.sin(theta_z), np.cos(theta_z), 0],
                   [0, 0, 1]])
    camera_origin_world = copy.copy(pos_top_world).reshape(3, 1)
    camera_origin_world[2, 0] = -6-13
    print("pos_top_world:", pos_top_world)
    data = Rz.T @ data + camera_origin_world
    return data.T


def filterSingleImage(image):
    r = 3
    threshold1 = 20  # 用于边缘链接
    threshold2 = 70  # 用于强边缘的检测
    blurred = cv2.GaussianBlur(image, (r, r), 0)  # 高斯矩阵的长与宽都是r，标准差为0
    # image = image - blurred
    mask1 = cv2.Canny(blurred, threshold1, threshold2)
    # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    # mask1 = cv2.dilate(mask1, kernel)  # 膨胀图像
    # mask1 = cv2.erode(mask1, kernel)  # 腐蚀图像
    mask1[np.where(mask1 > 1)] = 1
    mask1[np.where(mask1 < 1)] = 0
    mask1 = np.expand_dims(mask1, 2).repeat(3, axis=2)
    image = np.multiply(image, mask1)
    return image

save_path = './data_excavation/' + '1727617004.4468644' + '/'

filename = save_path + 'pos_fingertip_0_0' + '.npy'
pos_fingertip_world = np.load(filename)  # nxhxwx3 (3: x, y, depth)
pos_fingertip_world = np.delete(pos_fingertip_world, np.where(pos_fingertip_world > 999), axis=0)  # cm
pos_top_index = np.argmin(pos_fingertip_world[:, 2])
pos_top_world = pos_fingertip_world[pos_top_index, :].squeeze()

# filename = save_path + 'visual_points_all_0' + '.npy'
# visual_depth_count = 0
# visual_points_all = np.load(filename)  # nxhxwx3 (3: x, y, depth)
################### 读取图像 ########################
frame1 = cv2.imread(save_path + "frame1s_0_2.jpg", cv2.IMREAD_COLOR)
frame2 = cv2.imread(save_path + "frame2s_0_2.jpg", cv2.IMREAD_COLOR)
frame1_filtered = filterSingleImage(frame1)  # Canny边缘提取与原图像掩膜后
frame2_filtered = filterSingleImage(frame2)
imgStack = np.hstack((frame1_filtered, frame2_filtered))  # 相同大小图像水平拼接
cv2.imshow("Camera", imgStack)  # 在窗口 "Demo4" 显示图像 imgStack
SC = StereoCamera()
disparity1, disparity2 = SC.stereoMatchSGBM(frame1_filtered, frame2_filtered)  # 视差计算
visual_points_all = SC.getDepthMapWithQ(disparity1)  # 深度计算
disparity1_show = disparity1/np.max(disparity1)  # : cv2颜色是 0-255
depth_show = visual_points_all[:, :, 2]/np.max(visual_points_all[:, :, 2])
disparity_and_depth = np.hstack((disparity1_show, depth_show))  # 相同大小图像水平拼接
cv2.imshow("Disparity1 and Depth", disparity_and_depth)  # 在窗口 "Demo4" 显示图像 imgStack
# 滤波
##################################################
# DBSCAN 无监督聚类
visual_points_o3d, visual_points_labels, visual_points_centers = camera.clusterPoints(visual_points_all)
# 找离触觉检测最高点最近的簇中心
pos_fingertip_camera, pos_top_camera = pos_world2camera(pos_fingertip_world, pos_top_world)  # cm
distances = np.linalg.norm(visual_points_centers - pos_top_camera, axis=1)
print("distances:", distances)
distance_min_index = np.argmin(distances)
print("pos_top_world:", pos_top_world)
print("pos_top_camera:", pos_top_camera)
print("visual_points_centers:", visual_points_centers)
# distance_min_index = 0
object_points_index = np.where(visual_points_labels == distance_min_index)
object_points = visual_points_o3d[object_points_index]
other_points_index = np.where(visual_points_labels != distance_min_index)
other_points = visual_points_o3d[other_points_index]
other_points_labels = visual_points_labels[other_points_index]
print("visual_points_all.shape:", visual_points_all.shape)
print("pos_fingertip_camera.shape:", pos_fingertip_camera.shape)
print("visual_points_all.shape:", visual_points_all.shape)
print("visual_points_o3d.shape:", visual_points_o3d.shape)
print("object_points.shape:", object_points.shape)
print("other_points.shape:", other_points.shape)


a = np.array([1, 2, 3])
b = pos_world2camera(pos_camera2world(a, pos_top_world), pos_top_world)
print("a, b:", a, b)

# 视觉物体点,青色
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(object_points)  # mm
colors = np.zeros((object_points.shape))
colors[:, 1:] = 1
pcd.colors = o3d.utility.Vector3dVector(colors)

# 世界坐标系中的视觉物体点,青色
pos_fingertip_world = pos_camera2world(object_points, pos_top_world)  # cm
pcd_world = o3d.geometry.PointCloud()
pcd_world.points = o3d.utility.Vector3dVector(pos_fingertip_world)  # mm
colors = np.zeros((pos_fingertip_world.shape))
colors[:, 1:] = 1
pcd_world.colors = o3d.utility.Vector3dVector(colors)

# 视觉其他点,彩色
other_pcd = o3d.geometry.PointCloud()
other_pcd.points = o3d.utility.Vector3dVector(other_points)  # mm
max_label = 5#other_points_labels.max()
colors = plt.get_cmap("tab20")(other_points_labels / (max_label if max_label > 0 else 1))
other_pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

# 坐标轴
axis_pcd = o3d.geometry.TriangleMesh().create_coordinate_frame(size=5, origin=[0, 0, 0])

# 簇中心点,白色
centers_pcd = o3d.geometry.PointCloud()
centers_pcd.points = o3d.utility.Vector3dVector(visual_points_centers)
colors = np.zeros((visual_points_centers.shape))
colors[:, :] = 1
centers_pcd.colors = o3d.utility.Vector3dVector(colors)

# 指尖接触物体点,红色
fingertip_pcd = o3d.geometry.PointCloud()
fingertip_pcd.points = o3d.utility.Vector3dVector(pos_fingertip_camera)
colors = np.zeros((pos_fingertip_camera.shape))
colors[:, 0] = 1
fingertip_pcd.colors = o3d.utility.Vector3dVector(colors)

# 挖掘轨迹生成
pos_center, featurevector, eigenvalue = calculate_principal_axis(object_points)
# 沿第二主轴方向挖掘
direction = featurevector[:, 1]
length = eigenvalue[1] + 5  # 延长挖掘长度，单位cm
if direction[1] < 0:  # 从y轴负轴方向往正轴方向挖掘
    direction = -1 * direction
theta = math.atan2(direction[1], direction[0])  # unit: rad
# 挖掘的中心点
excavation_pos_center_camera = copy.copy(pos_center)
# excavation_pos_center_camera[2] += 0.5  # unit: mm
# 挖掘的起始位置和终止位置
excavation_pos_start = copy.copy(excavation_pos_center_camera)
excavation_pos_start[:2] = excavation_pos_start[:2] - length * direction[:2]
excavation_pos_end = copy.copy(excavation_pos_center_camera)
excavation_pos_end[:2] = excavation_pos_end[:2] + length * direction[:2]
excavation_pos = np.concatenate((excavation_pos_start.reshape(1, 3), excavation_pos_center_camera.reshape(1, 3), excavation_pos_end.reshape(1, 3)), axis=0)
# 视觉物体点,黄色
lines = [[0, 1], [1, 2]]
#绘制线条
lines_pcd = o3d.geometry.LineSet()
lines_pcd.lines = o3d.utility.Vector2iVector(lines)
colors = np.zeros((len(lines), 3))
colors[0, :2] = 0.5
colors[1, :2] = 1
lines_pcd.colors = o3d.utility.Vector3dVector(colors) #线条颜色
lines_pcd.points = o3d.utility.Vector3dVector(excavation_pos)
print("eigenvalue:", eigenvalue)
print("excavation_pos_start:", excavation_pos_start)
print("pos_center:", pos_center)
print("excavation_pos_end:", excavation_pos_end)

vis = o3d.visualization.Visualizer()
vis.create_window(window_name='3D points', width=960, height=960, left=800, top=100)
vis.add_geometry(pcd)
vis.add_geometry(pcd_world)
vis.add_geometry(other_pcd)
vis.add_geometry(lines_pcd)
vis.add_geometry(axis_pcd)
vis.add_geometry(centers_pcd)
vis.add_geometry(fingertip_pcd)

opt = vis.get_render_option()
opt.background_color = np.asarray([0, 0, 0])
render_option = vis.get_render_option()  # 设置点云渲染参数
render_option.point_size = 3
ctr = vis.get_view_control()
ctr.set_up((0, -1, 0))  # set the positive direction of the x-axis as the up direction
ctr.set_front((0, 0, -1))  # set the positive direction of the x-axis toward you
# o3d.visualization.draw_geometries([pcd])
vis.run()
vis.destroy_window()
cv2.waitKey(0)

# colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
# colors[labels < 0] = 0
# self.pcd = o3d.geometry.PointCloud()
# self.pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
