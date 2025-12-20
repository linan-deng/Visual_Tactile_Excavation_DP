import matplotlib.pyplot as plt
import numpy as np

datapath = "data/"
imufilename = "1700666706.3355513_0.0_0.015_imu_data.txt"
imupath = datapath + imufilename
imu_data = np.loadtxt(imupath, delimiter=' ', dtype=float)
# imu_data = imu_data[::2, :]  # downsampling
imu_time = imu_data[:, 0]
imu_data = imu_data.take([2, 3, 7, 20], 1)
imu_data[:, 3] = imu_data[:, 3] - imu_data[0, 3]
# imu_data = abs(imu_data)
print("imu_data.shape:", imu_data.shape)

tactilepath = datapath + imufilename.split('_')[0] + "_tactile_data.txt"
tactile_data = np.loadtxt(tactilepath, delimiter=' ', dtype=float)
tactile_time = tactile_data[::16, 0]
tactile_data = tactile_data[:, 1:]
tactile_data = tactile_data.reshape((-1, 16, 16))
print("tactile_data.shape:", tactile_data.shape)


predict_imu_label = np.loadtxt("data_visulization/predict_imu_label.txt")
print(predict_imu_label.shape)
plt.close("all")
plot_title = ["acc_Y", "acc_Z", "w_X", "theta_X"]
for i in range(len(predict_imu_label)):
    if predict_imu_label[i] == 0:
        for j in range(4):
            plt.subplot(3, 2, j+1)
            plt.plot(i, imu_data[i, j], c='blue', marker='.', markersize=1)
    else:
        for j in range(4):
            plt.subplot(3, 2, j+1)
            plt.plot(i, imu_data[i, j], c='red', marker='.', markersize=1)
            plt.title(plot_title[j])

predict_tactile_label = np.loadtxt("data_visulization/predict_tactile_label.txt")
print(predict_tactile_label.shape)
for i in range(len(predict_tactile_label)):
    if predict_tactile_label[i] == 0:
        for j in range(2):
            plt.subplot(3, 2, 4+j+1)
            plt.plot(i, np.sum(tactile_data[i, :]), c='blue', marker='.', markersize=1)
    else:
        for j in range(2):
            plt.subplot(3, 2, 4+j+1)
            plt.plot(i, np.sum(tactile_data[i, :]), c='red', marker='.', markersize=1)

plt.show()