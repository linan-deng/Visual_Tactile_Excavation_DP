import math
import torch
import torch.nn as nn

class SinusoidalPosEmb(nn.Module):
    def __init__(self, dim):
        super().__init__()
        self.dim = dim

    def forward(self, x):  # 输入张量，其形状通常是 (batch_size, seq_length)。这个张量包含了序列的位置索引
        device = x.device  # 获取输入张量 x 所在的设备（CPU或GPU）
        half_dim = self.dim // 2  # 计算位置编码维度的一半。这是因为正弦和余弦编码将分别占据一半的维度
        emb = math.log(10000) / (half_dim - 1)  # 计算一个常数，用于后续计算每个位置的正弦和余弦频率
        emb = torch.exp(torch.arange(half_dim, device=device) * -emb)  # 创建一个从 0 到 half_dim - 1 的张量，并将其乘以 -emb，然后计算每个元素的指数。这将为每个频率分量生成一个衰减因子
        # print("x[:, None]:", x[:, None].shape)  # torch.Size([16, 1])
        # print("emb[None, :]:", emb[None, :].shape)  # torch.Size([1, 128])
        emb = x[:, None] * emb[None, :]  # 将输入位置索引 x 扩展一个维度，并与频率衰减因子 emb 相乘。这样，每个位置索引都会乘以对应的频率衰减因子
        # print("emb:", emb.shape)  # torch.Size([16, 128])
        emb = torch.cat((emb.sin(), emb.cos()), dim=-1)  # 对于每个位置索引和频率衰减因子的乘积，计算其正弦和余弦值，并将它们沿着最后一个维度连接起来。结果是一个形状为 (batch_size, seq_length, dim) 的张量，包含了位置的正弦和余弦编码
        # print("emb:", emb.shape)  # torch.Size([16, 256])
        return emb
