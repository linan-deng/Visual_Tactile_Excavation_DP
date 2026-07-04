import os
import time

import numpy as np
import torch
import torch.nn as nn
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from diffusers.training_utils import EMAModel
from diffusers.optimization import get_scheduler
from tqdm.auto import tqdm
from model.vision.model_getter import get_resnet
from model.diffusion.conditional_unet1d import ConditionalUnet1D
from typing import Callable
from model.tactile_enconder import TactileEncoder
from diffusers import DDIMPipeline
from diffusers import DDIMScheduler

class PredictAction():
    def __init__(self, model_name):

        # parameters
        self.obs_horizon = 2
        self.pred_horizon = 16
        self.action_horizon = 8
        # |o|o|                             observations: 2
        # | |a|a|a|a|a|a|a|a|               actions executed: 8
        # |p|p|p|p|p|p|p|p|p|p|p|p|p|p|p|p| actions predicted: 16

        # 构建ResNet18编码器
        # 将所有BatchNorm替换为GroupNorm以与EMA一起工作
        # 如果忘记这样做，性能将大幅下降
        self.vision_encoder1 = get_resnet('resnet18')
        self.vision_encoder1 = self.replace_bn_with_gn(self.vision_encoder1)

        self.vision_encoder2 = get_resnet('resnet18')
        self.vision_encoder2 = self.replace_bn_with_gn(self.vision_encoder2)

        # tactile_encoder = get_resnet('resnet18')
        # tactile_encoder = replace_bn_with_gn(tactile_encoder)
        self.tactile_encoder = TactileEncoder()

        # ResNet18的输出维度为512
        self.vision_feature_dim = 512
        self.tactile_feature_dim = 16
        self.lowdim_obs_dim = 4 + 4 + 6  # imu + gripper + robot
        # 每步的观察特征总维度
        self.obs_dim = self.vision_feature_dim * 2 + self.tactile_feature_dim + self.lowdim_obs_dim
        #  self.obs_dim = 512*2 + 16 + 14 = 1054
        self.action_dim = 4 + 6  # gripper + robot

        # create network object
        # 创建网络对象
        self.noise_pred_net = ConditionalUnet1D(
            input_dim=self.action_dim,
            global_cond_dim=self.obs_dim * self.obs_horizon)
        # the final arch has 2 parts
        # 最终架构有两个部分
        self.nets = nn.ModuleDict({
            'vision_encoder1': self.vision_encoder1,
            'vision_encoder2': self.vision_encoder2,
            'tactile_encoder': self.tactile_encoder,
            'noise_pred_net': self.noise_pred_net})

        '''
        接下来，代码设置了一个DDPMScheduler，用于在100个扩散迭代中进行操作：
        '''
        # for this demo, we use DDPMScheduler with 100 diffusion iterations
        # 对于这个演示，我们使用DDPMScheduler，并进行100个扩散迭代
        self.num_diffusion_iters = 100
        # self.noise_scheduler = DDPMScheduler(
        #     num_train_timesteps=self.num_diffusion_iters,
        #     # the choise of beta schedule has big impact on performance
        #     # we found squared cosine works the best
        #     # beta计划的选择对性能有很大影响
        #     # 我们发现平方余弦效果最好
        #     beta_schedule='squaredcos_cap_v2',
        #     # clip output to [-1, 1] to improve stability
        #     # 将输出裁剪到[-1, 1]以改善稳定性
        #     clip_sample=True,
        #     # our network predicts noise (instead of denoised action)
        #     # 我们的网络预测噪声（而不是去噪动作）
        #     prediction_type='epsilon'
        # )

        self.noise_scheduler = DDIMScheduler(
            num_train_timesteps=self.num_diffusion_iters,
            # the choise of beta schedule has big impact on performance
            # we found squared cosine works the best
            # beta计划的选择对性能有很大影响
            # 我们发现平方余弦效果最好
            beta_schedule='squaredcos_cap_v2',
            # clip output to [-1, 1] to improve stability
            # 将输出裁剪到[-1, 1]以改善稳定性
            clip_sample=True,
            # our network predicts noise (instead of denoised action)
            # 我们的网络预测噪声（而不是去噪动作）
            prediction_type='epsilon'
        )
        '''
        然后，代码将模型转移到CUDA设备上，并设置训练参数，如EMA模型、优化器和学习率调度器：
        '''
        # device transfer
        # 设备转移
        self.device = torch.device('cuda')
        print("self.device:", self.device)
        _ = self.nets.to(self.device)
        num_epochs = 100
        # 指数移动平均（EMA）
        # 加速训练并提高稳定性
        # 保存模型权重的副本
        # power 参数是一个超参数，它决定了在计算移动平均时的衰减率。
        # 具体来说，power 控制了历史数据对于当前估计的影响程度。
        self.ema = EMAModel(
            parameters=self.nets.parameters(),
            power=0.75)
        '''
        最后，代码将EMA模型的权重用于推理，并设置了一个环境交互的评估循环，以执行pushT任务：
        '''
        # Weights of the EMA model
        # is used for inference
        # EMA模型的权重
        # 用于推理
        self.ema_nets = self.nets
        self.ema.copy_to(self.ema_nets.parameters())
        self.ema_nets.load_state_dict(torch.load('./model_save/'+model_name, map_location=self.device))
        self.ema_nets.to(self.device)

    # # data: b * k * c
    # def normalize_data_z_score(self, data):
    #     print("data:", data)
    #     mean = np.mean(data)
    #     std = np.std(data)
    #     data = (data - mean) / std
    #     return data

    def replace_submodules(self,
            root_module: nn.Module,
            predicate: Callable[[nn.Module], bool],
            func: Callable[[nn.Module], nn.Module]) -> nn.Module:
        """
        Replace all submodules selected by the predicate with
        the output of func.

        predicate: Return true if the module is to be replaced.
        func: Return new module to use.
        """
        # 如果根模块本身需要被替换，则直接替换并返回
        if predicate(root_module):
            return func(root_module)
        # 遍历所有子模块，并收集需要被替换的模块路径列表
        bn_list = [k.split('.') for k, m
                   in root_module.named_modules(remove_duplicate=True)
                   if predicate(m)]
        # 在PyTorch中，named_modules 是一个方法，它用于遍历神经网络模型中的所有模块，包括模型本身以及模型中的所有子模块
        # 此处假设 remove_duplicate=True 存在
        # 遍历需要被替换的模块路径列表，*parent 表示除了最后一个元素之外的所有路径部分，k 是最后一个元素，即模块名
        for *parent, k in bn_list:
            # 获取父模块
            parent_module = root_module
            # 如果路径不为空，则使用 get_submodule 方法获取父模块
            if len(parent) > 0:
                parent_module = root_module.get_submodule('.'.join(parent))
            # 获取需要被替换的模块
            if isinstance(parent_module, nn.Sequential):
                src_module = parent_module[int(k)]
            else:
                src_module = getattr(parent_module, k)
            # 使用 func 函数创建新的模块
            tgt_module = func(src_module)
            # 替换模块
            if isinstance(parent_module, nn.Sequential):
                parent_module[int(k)] = tgt_module
            else:
                setattr(parent_module, k, tgt_module)
        # verify that all modules are replaced
        # 再次遍历所有子模块，确保所有需要被替换的模块都被替换了
        bn_list = [k.split('.') for k, m
                   in root_module.named_modules(remove_duplicate=True)
                   if predicate(m)]
        # 断言检查，确保没有模块未被替换
        assert len(bn_list) == 0  # "仍有模块未被替换"
        return root_module

    def replace_bn_with_gn(self,
            root_module: nn.Module,
            features_per_group: int=16) -> nn.Module:
        """
        Relace all BatchNorm layers with GroupNorm.
        """
        self.replace_submodules(
            root_module=root_module,
            predicate=lambda x: isinstance(x, nn.BatchNorm2d),
            func=lambda x: nn.GroupNorm(num_groups=x.num_features//features_per_group, num_channels=x.num_features)
        )
        return root_module

    def predict_action(self, nimage1, nimage2, ntactile, nimu, nactionObs):
        nimage1 = np.array(nimage1, dtype=np.float16)
        nimage2 = np.array(nimage2, dtype=np.float16)
        ntactile = np.array(ntactile, dtype=np.float16)
        nimu = np.array(nimu, dtype=np.float16)
        nactionObs = np.array(nactionObs, dtype=np.float16)

        ### 观测数据归一化
        nimage1 = nimage1 / 255.0
        nimage2 = nimage2 / 255.0
        ntactile = ntactile
        # nimu[:, :, :2] = self.normalize_data_z_score(nimu[:, :, :2])
        # nimu[:, :, 2] = self.normalize_data_z_score(nimu[:, :, 2])
        # nimu[:, :, 3] = self.normalize_data_z_score(nimu[:, :, 3])
        ### 动作数据归一化
        nactionObs[:, :3] = nactionObs[:, :3] / 0.5  # robot end: x,y,z (unit: m)
        nactionObs[:, 3:6] = nactionObs[:, 3:6] / np.pi  # robot end: θx,θy,θz (unit: rad)
        nactionObs[:, 6:10] = nactionObs[:, 6:10] / 90.0  # servo: θ11,θ12,θ21,θ22 (unit: °)

        ### nnumpy数组转torch tensor，数据传给CPU或者GPU
        nimage1 = torch.tensor(nimage1).to(self.device).float()
        nimage2 = torch.tensor(nimage2).to(self.device).float()
        ntactile = torch.tensor(ntactile).to(self.device).float()
        nimu = torch.tensor(nimu).to(self.device).float()  # torch.Size([2, 4])
        nactionObs = torch.tensor(nactionObs).to(self.device).float()  # torch.Size([2, 10])

        # encoder vision features
        # 编码器视觉特征
        # nimage1.shape = torch.Size([2, 3, 320, 240])
        image_features1 = self.nets['vision_encoder1'](nimage1)  # torch.Size([2, 512])
        image_features2 = self.nets['vision_encoder2'](nimage2)  # torch.Size([2, 512])
        tactile_features = self.nets['tactile_encoder'](ntactile)  # torch.Size([2, 16])
        # (B,self.obs_horizon,D)

        # concatenate vision feature and low-dim obs
        # 连接视觉特征和低维观察
        obs_features = torch.cat([image_features1, image_features2, tactile_features,
                                  nimu, nactionObs], dim=-1)  # torch.Size([2, 1054])
        obs_cond = obs_features.unsqueeze(0).flatten(start_dim=1)  # torch.Size([1, 2108])

        # infer action
        # 使用模型进行动作预测
        with torch.no_grad():
            # initialize action from Guassian noise
            # 从高斯噪声中初始化动作
            noisy_action = torch.randn(
                (1, self.pred_horizon, self.action_dim), device=self.device)
            naction = noisy_action

            # init scheduler
            # 初始化调度器，DDIM
            self.noise_scheduler.set_timesteps(int(self.num_diffusion_iters/5))

            ###### 执行反向扩散过程，逐步去除噪声
            # print("self.noise_scheduler.timesteps:", self.noise_scheduler.timesteps)
            for k in self.noise_scheduler.timesteps:
                # predict noise
                noise_pred = self.ema_nets['noise_pred_net'](
                    sample=naction,
                    timestep=k,
                    global_cond=obs_cond)

                # inverse diffusion step (remove noise)
                naction = self.noise_scheduler.step(
                    model_output=noise_pred,
                    timestep=k,
                    sample=naction).prev_sample

        naction = naction.detach().to('cpu').numpy()
        # (B, self.pred_horizon, self.action_dim)
        naction = naction[0]
        # print("naction.shape:", naction.shape)
        # unnormalize action
        # 将动作反标准化
        # naction[:, :3] = naction[:, :3] * 0.5  # robot end: x,y,z (unit: m)
        # naction[:, 3:6] = naction[:, 3:6] * np.pi  # robot end: θx,θy,θz (unit: rad)
        # naction[:, 6:10] = naction[:, 6:10] * 90.0  # servo: θ11,θ12,θ21,θ22 (unit: °)
        naction[:, :3] = naction[:, :3] * 0.005  # robot end: x,y,z (unit: m)
        naction[:, 3:6] = naction[:, 3:6] * 0.05  # robot end: θx,θy,θz (unit: rad)
        naction[:, 6:10] = naction[:, 6:10] * 5  # servo: θ11,θ12,θ21,θ22 (unit: °)
        action_pred = naction

        # only take self.action_horizon number of actions
        # 只取self.action_horizon数量的动作
        start = self.obs_horizon - 1
        end = start + self.action_horizon
        action = action_pred[start:end, :]
        # print("action:", action)
        return action, naction

    def test(self):
        nimage1 = np.zeros((2, 3, 320, 240))
        nimage2 = np.zeros((2, 3, 320, 240))
        ntactile = np.zeros((2, 1, 16, 16))
        nimu = np.zeros((2, 4))
        nactionObs = np.zeros((2, 10))
        t1 = time.time()
        for i in range(1000):
            action = self.predict_action(nimage1, nimage2, ntactile, nimu, nactionObs)
            if i % 1 == 0:
                print("freq:", 1/(time.time()-t1))
                t1 = time.time()

if __name__=="__main__":
    import skvideo.io
    PA = PredictAction(model_name='ema_model_100.pth')

    # PA.test()
    file_path = './data/dataset_raw_test/'
    time_name = '1737017364.244507'

    train_data_image = skvideo.io.vread(file_path+time_name+'_frame_merge.mp4')  # (4325, 640, 960, 3)

    train_data_tactile = np.loadtxt(file_path+time_name+'_tactile_data.txt', delimiter=' ')
    train_data_tactile = train_data_tactile[:, :].reshape(-1, 1, 16, 17)

    train_data_imu = np.loadtxt(file_path+time_name+'_imu_data.txt', delimiter=' ')
    train_data_imu = train_data_imu[:, :] - train_data_imu[0, :]

    train_data_robot = np.loadtxt(file_path+time_name+'_robot_data.txt', delimiter=',')
    train_data_robot = train_data_robot[:, :]

    train_data_gripper = np.loadtxt(file_path+time_name+'_gripper_data.txt', delimiter=' ')
    train_data_gripper = train_data_gripper[:, :]
    gripper_start_index = np.min(np.where(train_data_gripper[:, -1] == 80))
    print("gripper_start_index:", gripper_start_index)
    train_data_gripper = train_data_gripper[gripper_start_index:, :]
    train_data_image_time = train_data_gripper[gripper_start_index:, 0]

    train_data_image1 = train_data_image[gripper_start_index:, :, :480, :]
    train_data_image2 = train_data_image[gripper_start_index:, :, 480:, :]
    train_data_image1 = train_data_image1[:, ::4, ::4, :]  # 图片降维
    train_data_image2 = train_data_image2[:, ::4, ::4, :]  # 图片降维
    train_data_image1 = np.moveaxis(train_data_image1, -1, 1)
    train_data_image2 = np.moveaxis(train_data_image2, -1, 1)

    train_data_image1_all_step = train_data_image1
    train_data_image2_all_step = train_data_image2
    train_data_tactile_all_step = np.zeros((train_data_image1.shape[0], 1, 16, 16), dtype=np.float16)
    train_data_imu_all_step = np.zeros((train_data_image1.shape[0], 4), dtype=np.float16)
    train_data_action_all_step = np.zeros((train_data_image1.shape[0], 10), dtype=np.float16)

    # 遍历每个图像文件的所有时间步，先对齐单步的数据
    for image_end_idx in range(train_data_image_time.shape[0]):
        # print("image_end_idx:", image_end_idx)
        # 以image的时间为基准，选取其他模态传感器数据
        image_time = train_data_image_time[image_end_idx]
        gripper_end_idx = image_end_idx

        train_data_tactile_idx = train_data_tactile
        train_data_tactile_idx_time = train_data_tactile_idx[:, 0, 0]
        temp = np.where(train_data_tactile_idx_time < image_time)[0]
        if len(temp) > 0:
            tactile_end_idx = np.max(temp)
        else:
            tactile_end_idx = 0

        train_data_imu_idx = train_data_imu
        train_data_imu_idx_time = train_data_imu_idx[:, 0]
        temp = np.where(train_data_imu_idx_time < image_time)[0]
        if len(temp) > 0:
            imu_end_idx = np.max(temp)
        else:
            imu_end_idx = 0

        train_data_robot_idx = train_data_robot
        train_data_robot_idx_time = train_data_robot_idx[:, 0]
        temp = np.where(train_data_robot_idx_time < image_time)[0]
        if len(temp) > 0:
            robot_end_idx = np.max(temp)
        else:
            robot_end_idx = 0

        # (1, 3, 640, 480), 4325
        seq_image1 = train_data_image1[image_end_idx, :, :, :]
        # (1, 3, 640, 480), 4325
        seq_image2 = train_data_image2[image_end_idx, :, :, :]
        # (1, 16, 16), 1785
        seq_tactile = train_data_tactile[tactile_end_idx, :, :, 1:]
        # (1, 4), 9238
        seq_imu = train_data_imu[imu_end_idx, [2, 3, 7, 20]]  # acc_x, acc_y, w_x, theta_x
        # (1, 6), 1306
        seq_robot = train_data_robot[robot_end_idx, 7:13]
        # (1, 4), 4325
        seq_gripper = train_data_gripper[gripper_end_idx, 1:5]
        # (1, 10)
        seq_action = np.concatenate((seq_robot, seq_gripper), axis=0)
        train_data_image1_all_step[image_end_idx, :] = seq_image1
        train_data_image2_all_step[image_end_idx, :] = seq_image2
        train_data_tactile_all_step[image_end_idx, 0, :] = seq_tactile
        train_data_imu_all_step[image_end_idx, :] = seq_imu
        train_data_action_all_step[image_end_idx, :] = seq_action

    action_all = []
    delta_action_all = []
    action = np.zeros((1, 10), dtype=np.float16)
    action[0, 9] = 80
    obs_horizon = 2
    pred_horizon = 16
    action_horizon = 8
    time_low = obs_horizon-1
    time_high = train_data_image1_all_step.shape[0]
    print("time_low:", time_low)
    print("time_high:", time_high)

    for idx_step in range(time_low, time_high):
        if idx_step % action_horizon == time_low:
            print("idx_step:", idx_step)
            ### 每隔10步取一段序列
            # train_data_instance = 10
            # for i in range(int((total_length-self.pred_horizon)/train_data_instance)):
            #     idx_step = train_data_instance*i + self.obs_horizon - 1
            #### observations
            image1 = train_data_image1_all_step[idx_step+1-obs_horizon:idx_step+1, :, :, :]
            image2 = train_data_image2_all_step[idx_step+1-obs_horizon:idx_step+1, :, :, :]
            tactile = train_data_tactile_all_step[idx_step+1-obs_horizon:idx_step+1, :, :, :]
            imu = train_data_imu_all_step[idx_step+1-obs_horizon:idx_step+1, :]
            actionObs = train_data_action_all_step[idx_step+1-obs_horizon:idx_step+1, :]

            delta_action, predict_delta_action = PA.predict_action(nimage1=image1, nimage2=image2, ntactile=tactile, nimu=imu, nactionObs=actionObs)
            delta_action_all.append(delta_action)

    delta_action_all = np.array(delta_action_all).reshape(-1, 10)
    with open('./data/predict/'+time_name+'_delta_action.txt', 'w') as f:
        print("delta_action_all.shape:", delta_action_all.shape)
        np.savetxt(f, delta_action_all)
        f.close()
    with open('./data/predict/'+time_name+'_train_data_action_all.txt', 'w') as f:
        print("train_data_action_all_step.shape:", train_data_action_all_step.shape)
        np.savetxt(f, train_data_action_all_step)
        f.close()
    print('saved'+'./data/predict/'+time_name+'_delta_action.txt')








