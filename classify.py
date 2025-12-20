import copy

import numpy as np
import torch
from net_classification import classNet  # 模型定义，在model文件夹里

# print("torch.cuda.is_available():", torch.cuda.is_available())
# device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

class contactClassify():

    def __init__(self):
        self.model_path = "model/model_of_500.pkh"
        self.model = classNet()  # 构建一个模型，模型的描述在net.py里
        self.model.to(device)  # 把模型给到cuda里去
        self.model.load_state_dict(torch.load(self.model_path, map_location=device, weights_only=True))
        self.model.eval() # 预测模式时，输入单个样本，需要用model.eval()关闭BatchNorm层的训练模式

    def norm(self, arr):
        arr_min = np.min(arr)
        arr_max = np.max(arr)
        if (arr_max - arr_min) == 0:
            arr_norm = 0
        else:
            arr_norm = (arr - arr_min)/(arr_max - arr_min) # Min-Max Normalization: [0, 1]
            # arr_norm = 2 * (arr - arr_min)/(arr_max - arr_min) - 1 # Min-Max Normalization: [-1, 1]
        return arr_norm

    def classify(self, imu_data, tactile_data):
        # load parameter
        # input size 2000*8 numpy
        class_name_list = [0, 1]
        mini_imu_data = copy.deepcopy(imu_data)
        mini_tactile_data = copy.deepcopy(tactile_data)
        # 归一化
        # mini_imu_data[:, 0] = norm(mini_imu_data[:, 0])
        # mini_imu_data[:, 1] = norm(mini_imu_data[:, 1])
        # mini_imu_data[:, 2] = norm(mini_imu_data[:, 2])
        # mini_imu_data[:, 3] = norm(mini_imu_data[:, 3])
        # mini_imu_data[:, 4] = norm(mini_imu_data[:, 4])
        # mini_imu_data[:, 5] = norm(mini_imu_data[:, 5])
        # mini_imu_data[:, 6] = norm(mini_imu_data[:, 6])
        # mini_imu_data[:, 7] = norm(mini_imu_data[:, 7])
        # mini_imu_data[:, 8] = norm(mini_imu_data[:, 8])
        mini_imu_data[:, 0] = mini_imu_data[:, 0] / 1.0  # acc, w, theta
        mini_imu_data[:, 1] = mini_imu_data[:, 1] / 1.0
        mini_imu_data[:, 2] = mini_imu_data[:, 2] / 10.0
        mini_imu_data[:, 3] = mini_imu_data[:, 3] / 80.0
        mini_tactile_data = mini_tactile_data / 1.0
        # for column in range(3):
        #     mini_imu_data[:, column] = np.convolve(mini_imu_data[:, column].T, np.ones((5)) / 5, mode="same").T

        mini_imu_data = np.expand_dims(mini_imu_data, axis=0)
        mini_tactile_data = np.expand_dims(mini_tactile_data, axis=0)
        # mini_imu_data = torch.FloatTensor(mini_imu_data).to(device)
        # mini_tactile_data = torch.FloatTensor(mini_tactile_data).to(device)
        mini_imu_data = torch.as_tensor(mini_imu_data, dtype=torch.float32).to(device)
        mini_tactile_data = torch.as_tensor(mini_tactile_data, dtype=torch.float32).to(device)

        predict = self.model(mini_imu_data.float(), mini_tactile_data.float())
        predict_code = predict.cpu().detach().numpy()[0]
        # print("predict code", predict_code)
        predict_index = np.argmax(predict_code)
        # print("predict index", predict_index)
        class_name = class_name_list[predict_index]
        return class_name

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import time
    datapath = "data/"
    imufilename = "1736015555.605893_imu_data.txt"
    imupath = datapath + imufilename
    imu_data = np.loadtxt(imupath, delimiter=' ', dtype=float)
    imu_time = imu_data[:, 0]
    imu_data = imu_data.take([2, 3, 7, 20], 1)
    imu_data[:, 3] = imu_data[:, 3] - imu_data[0, 3]
    # filter
    for column in range(3):
        imu_data[:, column] = np.convolve(imu_data[:, column], np.ones((5,)) / 5,
                                                        mode="same")
    print("imu_data.shape:", imu_data.shape)

    tactilepath = datapath + imufilename.split('_')[0] + "_tactile_data.txt"
    tactile_data = np.loadtxt(tactilepath, delimiter=' ', dtype=float)
    tactile_time = tactile_data[::16, 0]
    tactile_data = tactile_data[:, 1:]
    tactile_data = tactile_data.reshape((-1, 16, 16))
    print("tactile_data.shape:", tactile_data.shape)

    sample_imu_length = 100
    sample_tactile_length = 12
    predict_imu_label = np.zeros(imu_data.shape[0])
    predict_tactile_label = np.zeros(tactile_data.shape[0])

    classify = contactClassify()
    t1 = time.time()
    for i in range(imu_data.shape[0]):
        # print("index_imu_stop:", index_imu_stop)
        index_imu_stop = i
        imu_time_stop = imu_time[index_imu_stop]
        index_tactile_stop_array = np.where(tactile_time < imu_time_stop)[0]
        if len(index_tactile_stop_array) == 0:
            index_tactile_stop = 0
        else:
            index_tactile_stop = np.max(index_tactile_stop_array)

        if index_imu_stop >= sample_imu_length and index_tactile_stop >= sample_tactile_length:
            mini_imu_data = imu_data[index_imu_stop - sample_imu_length:index_imu_stop, :]
            mini_tactile_data = tactile_data[index_tactile_stop - sample_tactile_length:index_tactile_stop, :]
            label = classify.classify(mini_imu_data, mini_tactile_data)
            predict_imu_label[i] = label
            freq = 1/(time.time()-t1)
            t1 = time.time()
            print(i, label, freq)

    t1 = time.time()
    for i in range(tactile_data.shape[0]):
        # print("index_imu_stop:", index_imu_stop)
        index_tactile_stop = i
        tactile_time_stop = tactile_time[index_tactile_stop]
        index_imu_stop_array = np.where(imu_time > tactile_time_stop)[0]
        if len(index_imu_stop_array) == 0:
            index_imu_stop = 0
        else:
            index_imu_stop = np.min(index_imu_stop_array)

        if index_imu_stop >= sample_imu_length and index_tactile_stop >= sample_tactile_length:
            mini_imu_data = imu_data[index_imu_stop - sample_imu_length:index_imu_stop, :]
            mini_tactile_data = tactile_data[index_tactile_stop - sample_tactile_length:index_tactile_stop, :]
            # print(mini_tactile_data)
            label = classify.classify(mini_imu_data, mini_tactile_data)
            predict_tactile_label[i] = label
            freq = 1/(time.time()-t1)
            t1 = time.time()
            print(i, label, freq)

    np.savetxt("data_visulization/predict_imu_label.txt", predict_imu_label)
    np.savetxt("data_visulization/predict_tactile_label.txt", predict_tactile_label)

    # plt.figure()
    # for i in range(len(predict_label)):
    #     if predict_label[i] == 0:
    #         plt.plot(i, imu_data[i, 6], c='blue', marker='.', markersize=1)
    #     else:
    #         plt.plot(i, imu_data[i, 6], c='red', marker='.', markersize=5)
    # plt.show()




