import time
import numpy as np
import torch
from torch import nn

class classNet(nn.Module):
    def __init__(self):
        super(classNet, self).__init__()
        # 一个层的out_channelsx需要等于下一个层的in_channel
        # imu data processing
        # image size: output = (input-kernel+2*padding)/stride + 1
        self.conv11 = nn.Conv1d(in_channels=4, out_channels=32, stride=1, kernel_size=7, padding=0)
        self.batch11 = nn.BatchNorm1d(32)
        self.leakRelu11 = nn.LeakyReLU(negative_slope=0.2)
        self.pooling11 = nn.AvgPool1d(stride=2, kernel_size=3, padding=0)

        self.conv12 = nn.Conv1d(in_channels=32, out_channels=16, stride=3, kernel_size=7, padding=0)
        self.batch12 = nn.BatchNorm1d(16)
        self.leakRelu12 = nn.LeakyReLU(negative_slope=0.2)
        self.pooling12 = nn.AvgPool1d(stride=2, kernel_size=3, padding=0)
        #
        # self.conv13 = nn.Conv1d(in_channels=256, out_channels=128, stride=7, kernel_size=7, padding=0)
        # self.batch13 = nn.BatchNorm1d(128)
        # self.leakRelu13 = nn.LeakyReLU(negative_slope=0.2)
        # self.avgpooling11 = nn.AvgPool1d(stride=1, kernel_size=4, padding=0)
        # self.maxpooling11 = nn.MaxPool1d(stride=5, kernel_size=4, padding=0)

        self.lstm11 = nn.LSTM(input_size=16, hidden_size=16, num_layers=1, batch_first=True)
        # self.ln11 = nn.Linear(32, 1)
        self.flatten11 = nn.Flatten()

        # tactile data processing
        self.conv21 = nn.Conv1d(in_channels=256, out_channels=64, stride=1, kernel_size=3, padding=1)
        self.batch21 = nn.BatchNorm1d(64)
        self.leakRelu21 = nn.LeakyReLU(negative_slope=0.2)
        self.pooling21 = nn.AvgPool1d(stride=2, kernel_size=3, padding=0)
        # 7 * 7
        self.conv22 = nn.Conv1d(in_channels=64, out_channels=32, stride=2, kernel_size=3, padding=1)
        self.batch22 = nn.BatchNorm1d(32)
        self.leakRelu22 = nn.LeakyReLU(negative_slope=0.2)
        self.pooling22 = nn.AvgPool1d(stride=2, kernel_size=3, padding=0)
        # self.avgpooling21 = nn.AvgPool2d(stride=1, kernel_size=4, padding=0)
        # self.maxpooling21 = nn.MaxPool2d(stride=4, kernel_size=3, padding=0)

        self.lstm21 = nn.LSTM(input_size=32, hidden_size=16, num_layers=1, batch_first=True)
        # self.ln11 = nn.Linear(32, 1)
        self.flatten21 = nn.Flatten()

        self.ln1 = nn.Linear(96+16, 2)
        self.batch_ln1 = nn.BatchNorm1d(2)
        self.leakRelu_ln1 = nn.LeakyReLU(negative_slope=0.2)
        self.softmax = nn.Softmax(dim=1)

    def forward(self, input1, input2):
        input1 = input1.permute(0, 2, 1)
        # print("input1.shape:", input1.shape)
        input1 = self.conv11(input1)
        input1 = self.batch11(input1)
        input1 = self.leakRelu11(input1)
        # print("input1.shape:", input1.shape)
        input1 = self.pooling11(input1)
        # print("input1.shape:", input1.shape)
        input1 = self.conv12(input1)
        input1 = self.batch12(input1)
        input1 = self.leakRelu12(input1)
        input1 = self.pooling12(input1)
        # print("input1.shape:", input1.shape)
        # input1 = self.conv13(input1)
        # input1 = self.batch13(input1)
        # input1 = self.leakRelu13(input1)
        # print("input1.shape:", input1.shape)
        # input1 = self.maxpooling11(input1)
        # print("input1.shape:", input1.shape)
        input1 = input1.permute(0, 2, 1)
        # print("input1.shape:", input1.shape)
        input1, _ = self.lstm11(input1)
        # print("input1.shape:", input1.shape)
        # input1 = self.ln11(input1)
        # print("input1.shape:", input1.shape)
        input1 = self.flatten11(input1)
        # print("input1.shape:", input1.shape)
        # input1 = input1.reshape(-1, 32)
        # print("input1.shape:", input1.shape)

        input2 = input2.reshape((-1, input2.shape[1], 256))
        input2 = input2.permute(0, 2, 1)
        input2 = self.conv21(input2)
        input2 = self.batch21(input2)
        input2 = self.leakRelu21(input2)
        input2 = self.pooling21(input2)
        # print("input2.shape:", input2.shape)
        input2 = self.conv22(input2)
        input2 = self.batch22(input2)
        input2 = self.leakRelu22(input2)
        input2 = self.pooling22(input2)
        # print("input2.shape:", input2.shape)
        # input2 = self.maxpooling21(input2)
        # print("input2.shape:", input2.shape)
        input2 = input2.permute(0, 2, 1)
        # print("input2.shape:", input2.shape)
        input2, _ = self.lstm21(input2)
        # print("input2.shape:", input2.shape)
        input2 = self.flatten21(input2)
        # print("input2.shape:", input2.shape)
        # input2 = self.flatten22(input2)
        # print("input2.shape:", input2.shape)

        # print("input1.shape:", input1.shape)
        # print("input2.shape:", input2.shape)
        linear_input = torch.cat((input1, input2), dim=1)
        linear_input = self.ln1(linear_input)
        linear_input = self.batch_ln1(linear_input)
        linear_input = self.leakRelu_ln1(linear_input)
        linear_output = self.softmax(linear_input)

        return linear_output

if __name__ == "__main__":
    # device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    device = torch.device('cpu')

    testData1 = np.ones((1, 100, 4))
    testData2 = np.ones((1, 12, 16, 16))
    # testDataTorch1 = torch.FloatTensor(testData1)
    # testDataTorch2 = torch.FloatTensor(testData2)
    testDataTorch1 = torch.as_tensor(testData1, dtype=torch.float32).to(device)
    testDataTorch2 = torch.as_tensor(testData2, dtype=torch.float32).to(device)

    model = classNet()
    model.to(device)  # 把模型给到cuda里去
    # model_path = "model/model_of_100.pkh"
    # model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval() # 预测模式时，输入单个样本，需要用model.eval()关闭BatchNorm层的训练模式
    # torch.no_grad()
    print(model.parameters())
    t1 = time.time()
    start_time = t1
    for i in range(1000):
        ans = model(testDataTorch1, testDataTorch2)
        # print("ans:", ans)
        print("freq:", 1/(time.time()-t1))
        t1 = time.time()
    print("average freq:", 1000/(time.time()-start_time))