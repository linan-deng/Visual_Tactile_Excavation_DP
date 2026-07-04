import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

class TactileEncoder(nn.Module):
    def __init__(self):
        super(TactileEncoder, self).__init__()
        # 定义编码器的各个层
        self.conv1 = nn.Conv2d(in_channels=1, out_channels=8, kernel_size=3, stride=1, padding=1)  # 假设输入图像为3通道
        self.gnorm1 = nn.GroupNorm(num_groups=4, num_channels=8)
        self.conv2 = nn.Conv2d(in_channels=8, out_channels=16, kernel_size=3, stride=1, padding=1)
        self.gnorm2 = nn.GroupNorm(num_groups=4, num_channels=16)
        self.relu = nn.ReLU(inplace=True)
        self.avgpool = nn.AdaptiveAvgPool2d((1, 1))  # 使用最大池化层来减小特征图的尺寸
        self.flatten = nn.Flatten()

    def forward(self, x):
        # 通过编码器传播输入
        # print('x.shape:', x.shape)  # torch.Size([8, 1, 16, 16])
        x = self.conv1(x)  # 应用ReLU激活函数
        # print('x.shape:', x.shape)  # torch.Size([8, 8, 16, 16])
        x = self.gnorm1(x)
        # print('x.shape:', x.shape)  # torch.Size([8, 8, 16, 16])
        x = self.relu(x)
        # print('x.shape:', x.shape)  # torch.Size([8, 8, 16, 16])
        x = self.conv2(x)  # 应用ReLU激活函数
        # print('x.shape:', x.shape)  # torch.Size([8, 16, 16, 16])
        x = self.gnorm2(x)
        # print('x.shape:', x.shape)  # torch.Size([8, 16, 16, 16])
        x = self.relu(x)
        # print('x.shape:', x.shape)  # torch.Size([8, 16, 16, 16])
        x = self.avgpool(x)           # 应用池化层
        # print('x.shape:', x.shape)  # torch.Size([8, 16, 1, 1])
        x = self.flatten(x)
        # print('x.shape:', x.shape)  # torch.Size([8, 16])
        # x现在是一个编码后的特征图，可以用于其他任务
        return x

if __name__ =="__main__":
    # 实例化编码器
    tactile_encoder = TactileEncoder()
    # 打印模型结构
    print(tactile_encoder)

    input = np.zeros((8, 1, 16, 16))

    input = torch.from_numpy(input).float()
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    input = input.to(device)
    encoder = tactile_encoder.to(device)
    output = encoder(input)
    print('input.shape:', input.shape)
    print('output.shape:', output.shape)