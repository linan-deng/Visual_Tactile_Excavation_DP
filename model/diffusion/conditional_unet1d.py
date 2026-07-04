from typing import Union
import logging
import torch
import torch.nn as nn
import einops
from einops.layers.torch import Rearrange

from model.diffusion.conv1d_components import (
    Downsample1d, Upsample1d, Conv1dBlock)
from model.diffusion.positional_embedding import SinusoidalPosEmb

logger = logging.getLogger(__name__)

class ConditionalResidualBlock1D(nn.Module):
    def __init__(self, 
            in_channels,  # 输入通道数
            out_channels,  # 输出通道数
            cond_dim,  # cond_dim = 256, 条件特征的维度, pushT任务有
            kernel_size=3,  # 卷积核大小
            n_groups=8,  # GroupNorm的组数
            cond_predict_scale=False):  # 是否预测每个通道的缩放和偏移
        super().__init__()

        # 初始化两个卷积块
        self.blocks = nn.ModuleList([
            Conv1dBlock(in_channels, out_channels, kernel_size, n_groups=n_groups),  # Conv1d --> GroupNorm --> Mish
            Conv1dBlock(out_channels, out_channels, kernel_size, n_groups=n_groups),  # Conv1d --> GroupNorm --> Mish
        ])

        # FiLM调制，参考论文 https://arxiv.org/abs/1709.07871
        # 预测每个通道的缩放和偏移
        # FiLM modulation https://arxiv.org/abs/1709.07871
        # predicts per-channel scale and bias
        cond_channels = out_channels
        if cond_predict_scale:
            cond_channels = out_channels * 2

        self.cond_predict_scale = cond_predict_scale
        self.out_channels = out_channels
        self.cond_encoder = nn.Sequential(
            nn.Mish(),  # 激活函数
            nn.Linear(cond_dim, cond_channels),
            Rearrange('batch t -> batch t 1'),
        )

        # 确保维度兼容
        # make sure dimensions compatible
        self.residual_conv = nn.Conv1d(in_channels, out_channels, 1) \
            if in_channels != out_channels else nn.Identity()

    def forward(self, x, cond):
        '''
            x : [ batch_size x in_channels x horizon ], features of actions A_t
            cond : [ batch_size x cond_dim], features of diffusion step k and observation O_t

            returns:
            out : [ batch_size x out_channels x horizon ]
        '''
        out = self.blocks[0](x)  # Conv1d --> GroupNorm --> Mish
        embed = self.cond_encoder(cond)  # Mish --> Linear --> Rearrange
        if self.cond_predict_scale:
            embed = embed.reshape(
                embed.shape[0], 2, self.out_channels, 1)
            # a
            scale = embed[:, 0, ...]
            # b
            bias = embed[:, 1, ...]
            # out: actions
            # FiLM conditioning: a * out + b
            out = scale * out + bias
        else:
            out = out + embed
        out = self.blocks[1](out)  # Conv1d --> GroupNorm --> Mish
        out = out + self.residual_conv(x)
        return out

class ConditionalUnet1D(nn.Module):
    def __init__(self,
        input_dim,  # 输入特征的维度
        local_cond_dim=None,  # 局部条件特征的维度
        global_cond_dim=None,  # 全局条件特征的维度, global_cond_dim = obs_dim * obs_horizon = (512*2+16+10) * 2
        diffusion_step_embed_dim = 256,  # 扩散步骤嵌入的维度
        down_dims=[256, 512, 1024],  # 下采样的维度列表
        kernel_size=3,  # 卷积核的大小
        n_groups=8,  # GroupNorm中的组数
        cond_predict_scale=False  # 是否预测条件比例
        ):
        super().__init__()
        all_dims = [input_dim] + list(down_dims)  # 拼接输入维度和下采样维度
        start_dim = down_dims[0]  # 获取下采样起始维度

        dsed = diffusion_step_embed_dim  # 256
        print("dsed:", dsed)
        # 扩散步骤编码器，用于将扩散步骤 k 编码成特征
        diffusion_step_encoder = nn.Sequential(
            SinusoidalPosEmb(dsed),  # 正弦位置编码
            nn.Linear(dsed, dsed * 4),  # 线性层
            nn.Mish(),  # Mish激活函数
            nn.Linear(dsed * 4, dsed),  # 线性层
        )
        cond_dim = dsed  # cond_dim = 256, 条件特征的维度, pushT任务有
        if global_cond_dim is not None:
            cond_dim += global_cond_dim  # 如果有全局条件，则增加其维度

        # 生成下采样层的输入输出维度对
        in_out = list(zip(all_dims[:-1], all_dims[1:]))

        # 局部条件编码器，对局部条件特征进行编码
        local_cond_encoder = None
        if local_cond_dim is not None:
            _, dim_out = in_out[0]
            dim_in = local_cond_dim
            local_cond_encoder = nn.ModuleList([
                # down encoder
                # 下编码器
                ConditionalResidualBlock1D(
                    dim_in, dim_out, cond_dim=cond_dim,  # cond_dim = 256, 条件特征的维度, pushT任务有
                    kernel_size=kernel_size, n_groups=n_groups,
                    cond_predict_scale=cond_predict_scale),
                # up encoder
                # 上编码器
                ConditionalResidualBlock1D(
                    dim_in, dim_out, cond_dim=cond_dim, 
                    kernel_size=kernel_size, n_groups=n_groups,
                    cond_predict_scale=cond_predict_scale)
            ])

        # 中间层的维度
        mid_dim = all_dims[-1]
        # 中间层模块
        self.mid_modules = nn.ModuleList([
            ConditionalResidualBlock1D(
                mid_dim, mid_dim, cond_dim=cond_dim,
                kernel_size=kernel_size, n_groups=n_groups,
                cond_predict_scale=cond_predict_scale
            ),
            ConditionalResidualBlock1D(
                mid_dim, mid_dim, cond_dim=cond_dim,
                kernel_size=kernel_size, n_groups=n_groups,
                cond_predict_scale=cond_predict_scale
            ),
        ])

        # 下采样模块
        down_modules = nn.ModuleList([])
        for ind, (dim_in, dim_out) in enumerate(in_out):
            is_last = ind >= (len(in_out) - 1)
            down_modules.append(nn.ModuleList([
                ConditionalResidualBlock1D(
                    dim_in, dim_out, cond_dim=cond_dim, 
                    kernel_size=kernel_size, n_groups=n_groups,
                    cond_predict_scale=cond_predict_scale),
                ConditionalResidualBlock1D(
                    dim_out, dim_out, cond_dim=cond_dim, 
                    kernel_size=kernel_size, n_groups=n_groups,
                    cond_predict_scale=cond_predict_scale),
                Downsample1d(dim_out) if not is_last else nn.Identity()
            ]))

        # 上采样模块
        up_modules = nn.ModuleList([])
        for ind, (dim_in, dim_out) in enumerate(reversed(in_out[1:])):
            is_last = ind >= (len(in_out) - 1)
            up_modules.append(nn.ModuleList([
                ConditionalResidualBlock1D(
                    dim_out*2, dim_in, cond_dim=cond_dim,
                    kernel_size=kernel_size, n_groups=n_groups,
                    cond_predict_scale=cond_predict_scale),
                ConditionalResidualBlock1D(
                    dim_in, dim_in, cond_dim=cond_dim,
                    kernel_size=kernel_size, n_groups=n_groups,
                    cond_predict_scale=cond_predict_scale),
                Upsample1d(dim_in) if not is_last else nn.Identity()
            ]))

        # 最终卷积层，用于生成输出
        final_conv = nn.Sequential(
            Conv1dBlock(start_dim, start_dim, kernel_size=kernel_size),  # Conv1d --> GroupNorm --> Mish
            nn.Conv1d(start_dim, input_dim, 1),
        )

        self.diffusion_step_encoder = diffusion_step_encoder
        self.local_cond_encoder = local_cond_encoder
        self.up_modules = up_modules
        self.down_modules = down_modules
        self.final_conv = final_conv

        # 计算并记录模型参数的总数
        logger.info(
            "number of parameters: %e", sum(p.numel() for p in self.parameters())
        )

    def forward(self, 
            sample: torch.Tensor, 
            timestep: Union[torch.Tensor, float, int],  # Union类型是一种类型注解，用于指定一个变量可能属于多种类型中的任何一种
            local_cond=None,
            global_cond=None, **kwargs):
        """
        前向传播函数定义，输入包括样本数据、时间步、局部条件和全局条件。
        x: (B, T, input_dim)
        timestep: (B,) or int, diffusion step
        local_cond: (B, T, local_cond_dim)
        global_cond: (B, global_cond_dim), observation
        output: (B, T, input_dim)
        """
        # 重新排列（permute）或重塑（reshape）张量
        # b(batch size,批次大小),h(height,高度或可以认为是其他维度,比如特征维度)和 t(time,时间步)
        sample = einops.rearrange(sample, 'b h t -> b t h')  # 在einops库中用于重新排列张量(tensor)维度顺序

        # 1. time
        # 处理时间步数据 k，确保其与样本数据在相同的设备上，并且具有正确的维度
        timesteps = timestep
        # print("timesteps:", timesteps)
        if not torch.is_tensor(timesteps):
            # 如果时间步不是张量，则将其转换为张量
            # TODO: this requires sync between CPU and GPU. So try to pass timesteps as tensors if you can
            timesteps = torch.tensor([timesteps], dtype=torch.long, device=sample.device)
        elif torch.is_tensor(timesteps) and len(timesteps.shape) == 0:
            # 如果时间步是零维张量，则增加一个维度
            timesteps = timesteps[None].to(sample.device)
        # broadcast to batch dimension in a way that's compatible with ONNX/Core ML
        # 将时间步广播到与样本数据相同的批次大小
        timesteps = timesteps.expand(sample.shape[0])
        # 打印样本数据的形状和时间步数据
        # print("sample.shape:", sample.shape)  # torch.Size([16, 2, 16])
        # print("timesteps.expand(sample.shape[0]):", timesteps)  # timesteps: tensor([51, 94, 80, 46, 47, 56, 41, 10, 95, 81, 24, 69, 14, 75, 44, 84], device='cuda:0')
        # 使用扩散步骤编码器对时间步进行编码，生成全局特征(正弦编码-线性层-激活函数-线性层)
        global_feature = self.diffusion_step_encoder(timesteps)
        # print("local_cond:", local_cond)
        # print("global_cond:", global_cond)
        # print("global_cond.shape:", global_cond.shape)  # torch.Size([16, 1028])
        ############### 如果提供了全局条件，将其与全局特征进行拼接, pushT任务有这步 ########################
        if global_cond is not None:
            # print("global_cond is not None")
            global_feature = torch.cat([global_feature, global_cond], axis=-1)  # features of diffusion step k and observation O_t
        
        # encode local features
        # 编码局部特征, pushT任务没有这步
        h_local = list()
        if local_cond is not None:
            print("local_cond is not None")
            # 对局部条件进行维度重新排列
            local_cond = einops.rearrange(local_cond, 'b h t -> b t h')
            # 使用局部条件编码器对局部条件进行编码
            resnet, resnet2 = self.local_cond_encoder
            x = resnet(local_cond, global_feature)  # ConditionalResidualBlock1D
            h_local.append(x)
            x = resnet2(local_cond, global_feature)  # ConditionalResidualBlock1D
            h_local.append(x)

        # 初始化 x 为样本数据
        x = sample
        h = []
        # 通过下采样模块处理数据
        for idx, (resnet, resnet2, downsample) in enumerate(self.down_modules):
            # x: action A_t, global_feature: step k and observation O_t
            x = resnet(x, global_feature)  # ConditionalResidualBlock1D
            if idx == 0 and len(h_local) > 0:
                x = x + h_local[0]
            x = resnet2(x, global_feature)  # ConditionalResidualBlock1D
            h.append(x)
            x = downsample(x)  # Downsample1d

        # 通过中间模块处理数据
        for mid_module in self.mid_modules:
            x = mid_module(x, global_feature)  # ConditionalResidualBlock1D, ConditionalResidualBlock1D

        # 通过上采样模块处理数据
        for idx, (resnet, resnet2, upsample) in enumerate(self.up_modules):
            x = torch.cat((x, h.pop()), dim=1)
            x = resnet(x, global_feature)  # ConditionalResidualBlock1D
            # 这里有一个注释掉的代码块，表明有一个可能的错误，但为了与已发布的检查点兼容，未进行更改
            # The correct condition should be:
            # if idx == (len(self.up_modules)-1) and len(h_local) > 0:
            # However this change will break compatibility with published checkpoints.
            # Therefore it is left as a comment.
            if idx == len(self.up_modules) and len(h_local) > 0:
                x = x + h_local[1]
            x = resnet2(x, global_feature)  # ConditionalResidualBlock1D
            x = upsample(x)  # Upsample1d

        # 使用最后一个卷积层处理数据
        x = self.final_conv(x)
        # 将输出数据的维度重新排列回原始形状
        x = einops.rearrange(x, 'b t h -> b h t')
        return x

