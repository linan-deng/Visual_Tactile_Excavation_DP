import time
import os
import glob
import skvideo
# skvideo.setFFmpegPath('./miniconda3/envs/DPRE/lib/python3.8/site-packages')
import skvideo.io
import cv2
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
from skimage.transform import resize

# normalize data
def get_data_stats(data):
    data = data.reshape(-1, data.shape[-1])
    stats = {
        'min': np.min(data, axis=0),
        'max': np.max(data, axis=0)
    }
    return stats

def normalize_data(data, stats):
    # 分批处理数据，避免内存溢出
    batch_size = 10
    for i in range(0, data.shape[0], batch_size):
        end = i + batch_size
        # nomalize to [0,1]
        data[i:end] = (data[i:end] - stats['min']) / (stats['max'] - stats['min'])
        # normalize to [-1, 1]
        data[i:end] = data[i:end] * 2.0 - 1.0
    return data

# data: b * k * c
def normalize_data_z_score(data):
    mean = np.mean(data)
    std = np.std(data)
    data = (data - mean) / std
    return data

def unnormalize_data(ndata, stats):
    ndata = (ndata + 1) / 2
    data = ndata * (stats['max'] - stats['min']) + stats['min']
    return data

class RobotExcavationDataset(torch.utils.data.Dataset):
    def __init__(self,
                 dataset_raw_path: str,
                 image_name: str,
                 pred_horizon: int,
                 obs_horizon: int,
                 action_horizon: int):

        file_name_image = []
        file_name_gripper = []
        file_name_imu = []
        file_name_robot = []
        file_name_tactile = []

        time_name = image_name.split('_')[0]
        file_name_image.append(image_name)
        file_name_gripper.append(time_name+'_gripper_data.txt')
        file_name_imu.append(time_name+'_imu_data.txt')
        file_name_robot.append(time_name+'_robot_data.txt')
        file_name_tactile.append(time_name+'_tactile_data.txt')

        # obs: image, tactile, imu
        train_data_tactile = []
        for item in file_name_tactile:
            print('item:', item)
            data = np.loadtxt(dataset_raw_path+item, delimiter=' ')
            print('data.shape:', data.shape)
            data = data[:, :].reshape(-1, 1, 16, 17)
            # print('data.shape:', data.shape)
            # data = np.expand_dims(data, 3).repeat(3, axis=3)
            # print('data.shape:', data.shape)
            # data = np.moveaxis(data, -1, 1)
            # print('data.shape:', data.shape)
            # print('train_data_tactile.shape:', data.shape)
            train_data_tactile.append(data)

        train_data_imu = []
        for item in file_name_imu:
            print('item:', item)
            data = np.loadtxt(dataset_raw_path+item, delimiter=' ')
            data = data[:, :]
            print('train_data_imu.shape:', data.shape)
            train_data_imu.append(data)

        # action: robot, gripper
        train_data_robot = []
        for item in file_name_robot:
            print('item:', item)
            data = np.loadtxt(dataset_raw_path+item, delimiter=',')
            data = data[:, :]
            print('train_data_robot.shape:', data.shape)
            train_data_robot.append(data)

        train_data_gripper = []
        for item in file_name_gripper:
            print('item:', item)
            data = np.loadtxt(dataset_raw_path+item, delimiter=' ')
            data = data[:, :]
            print('train_data_gripper.shape:', data.shape)
            train_data_gripper.append(data)

        # image 4325
        self.seq_length_imu = 1  # 9238
        self.seq_length_tactile = 1  # 1785
        self.seq_length_robot = 1  # 1306
        self.seq_length_gripper = 1  # 4325

        self.pred_horizon = pred_horizon
        self.action_horizon = action_horizon
        self.obs_horizon = obs_horizon

        train_data_image1_all = []
        train_data_image2_all = []
        train_data_tactile_all = []
        train_data_imu_all = []
        train_data_actionObs_all = []
        train_data_action_all = []

        # 遍历图像文件
        for idx, item in enumerate(file_name_image):
            print('idx, item:', idx, item)
            file_time = item.split('_')[0]
            train_data_image_time = np.loadtxt(dataset_raw_path+file_time+'_gripper_data.txt')[:, 0]

            t1 = time.time()
            train_data_image = skvideo.io.vread(dataset_raw_path+item)  # (4325, 640, 960, 3)
            ####################### cv2读取视频 #######################
            # cap = cv2.VideoCapture(dataset_raw_path+item)
            # wid = int(cap.get(3))
            # hei = int(cap.get(4))
            # framenum = int(cap.get(7))
            # train_data_image = np.zeros((framenum,hei,wid,3),dtype='float16')
            # cnt = 0
            # for i in range(framenum):
            #     a, b=cap.read()
            #     train_data_image[i] = np.array(b, dtype='float16')/255.0
            #     print("cnt:", cnt)
            #     cnt+=1
            #########################################################
            train_data_image1 = train_data_image[:, :, :480, :]
            train_data_image2 = train_data_image[:, :, 480:, :]
            print('train_data_image1.shape:', train_data_image1.shape)
            train_data_image1 = train_data_image1[:, ::2, ::2, :]  # 图片降维
            train_data_image2 = train_data_image2[:, ::2, ::2, :]  # 图片降维
            train_data_image1 = np.moveaxis(train_data_image1, -1, 1)
            train_data_image2 = np.moveaxis(train_data_image2, -1, 1)
            print('train_data_image1.shape:', train_data_image1.shape)
            print('time cost:', time.time()-t1)

            train_data_image1_all_step = []
            train_data_image2_all_step = []
            train_data_tactile_all_step = []
            train_data_imu_all_step = []
            train_data_action_all_step = []
            # 遍历每个图像文件的所有时间步，先对齐单步的数据
            for image_end_idx in range(train_data_image_time.shape[0]):
                # 以image的时间为基准，选取其他模态传感器数据
                image_time = train_data_image_time[image_end_idx]
                gripper_end_idx = image_end_idx

                train_data_tactile_idx = train_data_tactile[idx]
                train_data_tactile_idx_time = train_data_tactile_idx[:, 0, 0]
                temp = np.where(train_data_tactile_idx_time < image_time)[0]
                if len(temp) > 0:
                    tactile_end_idx = np.max(temp)
                else:
                    tactile_end_idx = 0

                train_data_imu_idx = train_data_imu[idx]
                train_data_imu_idx_time = train_data_imu_idx[:, 0]
                temp = np.where(train_data_imu_idx_time < image_time)[0]
                if len(temp) > 0:
                    imu_end_idx = np.max(temp)
                else:
                    imu_end_idx = 0

                train_data_robot_idx = train_data_robot[idx]
                train_data_robot_idx_time = train_data_robot_idx[:, 0]
                temp = np.where(train_data_robot_idx_time < image_time)[0]
                if len(temp) > 0:
                    robot_end_idx = np.max(temp)
                else:
                    robot_end_idx = 0

                if tactile_end_idx >= self.seq_length_tactile and \
                        imu_end_idx >= self.seq_length_imu and \
                        robot_end_idx >= self.seq_length_robot and \
                        gripper_end_idx >= self.seq_length_gripper:

                    # (1, 3, 640, 480), 4325
                    seq_image1 = train_data_image1[image_end_idx, :, :, :]
                    # (1, 3, 640, 480), 4325
                    seq_image2 = train_data_image2[image_end_idx, :, :, :]
                    # (1, 16, 16), 1785
                    seq_tactile = train_data_tactile[idx][tactile_end_idx, :, :, 1:]
                    # (1, 4), 9238
                    seq_imu = train_data_imu[idx][imu_end_idx, [2, 3, 7, 20]]  # acc_x, acc_y, w_x, theta_x
                    # (12, 6), 1306
                    seq_robot = train_data_robot[idx][robot_end_idx, 7:13]
                    # (12, 4), 4325
                    seq_gripper = train_data_gripper[idx][gripper_end_idx, 1:5]
                    # (12, 10)
                    seq_action = np.concatenate((seq_robot, seq_gripper), axis=0)

                    train_data_image1_all_step.append(seq_image1)
                    train_data_image2_all_step.append(seq_image2)
                    train_data_tactile_all_step.append(seq_tactile)
                    train_data_imu_all_step.append(seq_imu)
                    train_data_action_all_step.append(seq_action)

            train_data_image1_all_step = np.array(train_data_image1_all_step)
            train_data_image2_all_step = np.array(train_data_image2_all_step)
            train_data_tactile_all_step = np.array(train_data_tactile_all_step)
            train_data_imu_all_step = np.array(train_data_imu_all_step)
            train_data_action_all_step = np.array(train_data_action_all_step)
            print("train_data_image1_all_step.shape:", train_data_image1_all_step.shape)
            print("train_data_tactile_all_step.shape:", train_data_tactile_all_step.shape)
            print("train_data_imu_all_step.shape:", train_data_imu_all_step.shape)
            print("train_data_action_all_step.shape:", train_data_action_all_step.shape)
            total_length = train_data_image1_all_step.shape[0]

            # 每隔10步取一段序列
            for i in range(int(total_length/10)):
                idx_step = 10*i
                # print("idx_step:", idx_step)
                if idx_step >= self.obs_horizon and idx_step <= total_length-self.pred_horizon:
                    # state
                    image1 = train_data_image1_all_step[idx_step-self.obs_horizon:idx_step, :, :, :]
                    image2 = train_data_image2_all_step[idx_step-self.obs_horizon:idx_step, :, :, :]
                    tactile = train_data_tactile_all_step[idx_step-self.obs_horizon:idx_step, :, :, :]
                    imu = train_data_imu_all_step[idx_step-self.obs_horizon:idx_step, :]
                    actionObs = train_data_action_all_step[idx_step-self.obs_horizon:idx_step, :]
                    # action, 预测增量以减小振荡
                    action = train_data_action_all_step[idx_step-self.obs_horizon:idx_step-self.obs_horizon+self.pred_horizon, :] -\
                             train_data_action_all_step[idx_step-self.obs_horizon-1:idx_step-self.obs_horizon-1+self.pred_horizon, :]

                    train_data_image1_all.append(image1)
                    train_data_image2_all.append(image2)
                    train_data_tactile_all.append(tactile)
                    train_data_imu_all.append(imu)
                    train_data_actionObs_all.append(actionObs)
                    train_data_action_all.append(action)

        train_data_image1_all = np.array(train_data_image1_all, dtype=np.float32)
        train_data_image2_all = np.array(train_data_image2_all, dtype=np.float32)
        train_data_tactile_all = np.array(train_data_tactile_all, dtype=np.float32)
        train_data_imu_all = np.array(train_data_imu_all, dtype=np.float32)
        train_data_actionObs_all = np.array(train_data_actionObs_all, dtype=np.float32)
        train_data_action_all = np.array(train_data_action_all, dtype=np.float32)
        print('len(train_data_image1_all) {}: {}'.format(idx, len(train_data_image1_all)))  # (431, 2, 3, 640, 480)
        print("train_data_tactile_all.shape:", train_data_tactile_all.shape)  # (431, 2, 3, 16, 16)
        print("train_data_imu_all.shape:", train_data_imu_all.shape)  # (431, 2, 4)
        print("train_data_action_all.shape:", train_data_action_all.shape)  # (431, 8, 10)

        print('len(train_data_image1_all total): {}'.format(len(train_data_image1_all)))  # 431

        # gripper, imu, robot, tactile, image
        train_data = {
            'image1': train_data_image1_all,
            'image2': train_data_image2_all,
            'tactile': train_data_tactile_all,
            'imu': train_data_imu_all[:, :],
            'actionObs': train_data_actionObs_all,
            'action': train_data_action_all,
        }

        # compute statistics and normalized data to [-1,1]
        # 状态和动作归一化
        stats = dict()
        normalized_train_data = dict()
        for key, data in train_data.items():
            # images are already normalized
            # print("key, data:", key, data)
            if key == 'image1' or key == 'image2':
                normalized_train_data[key] = data/255.0
                # print(data[0])
                print('image normalized !!!')

            if key == 'tactile':
                # stats[key] = get_data_stats(data)
                # normalized_train_data[key] = normalize_data(data, stats[key])
                normalized_train_data[key] = data
                # print(data[0])
                print('tactile normalized !!!')

            # imu各类通道单独归一化, acc_x, acc_y, w_x, theta_x
            if key == 'imu':
                # data[:, :, [0, 1]] = normalize_data_z_score(data[:, :, [0, 1]])
                # data[:, :, 2] = normalize_data_z_score(data[:, :, 2])
                # data[:, :, 3] = normalize_data_z_score(data[:, :, 3])
                normalized_train_data[key] = data
                # print(data[0])
                print('imu normalized !!!')

            # action包含机器人动作和机械手动作
            if key == 'actionObs':
                data[:, :, :3] = data[:, :, :3]/0.5  # robot end: x,y,z (unit: m)
                data[:, :, 3:6] = data[:, :, 3:6]/np.pi  # robot end: θx,θy,θz (unit: rad)
                data[:, :, 6:10] = data[:, :, 6:10]/90.0  # servo: θ11,θ12,θ21,θ22 (unit: °)
                normalized_train_data[key] = data
                # print(data[0])
                print('actionObs normalized !!!')

            # action包含机器人动作和机械手动作
            if key == 'action':
                data[:, :, :3] = data[:, :, :3]/0.005  # robot end: x,y,z (unit: m)
                data[:, :, 3:6] = data[:, :, 3:6]/0.05  # robot end: θx,θy,θz (unit: rad)
                data[:, :, 6:10] = data[:, :, 6:10]/5  # servo: θ11,θ12,θ21,θ22 (unit: °)
                normalized_train_data[key] = data
                # print(data[0])
                print('action normalized !!!')

        self.normalized_train_data = normalized_train_data

    # def __len__(self):
    #     return len(self.normalized_train_data['image1'])
    #
    # def __getitem__(self, idx):
    #     sample = {
    #         'image1': self.normalized_train_data['image1'][idx, :],
    #         'image2': self.normalized_train_data['image2'][idx, :],
    #         'tactile': self.normalized_train_data['tactile'][idx, :],
    #         'imu': self.normalized_train_data['imu'][idx, :],
    #         'actionObs': self.normalized_train_data['actionObs'][idx, :],
    #         'action': self.normalized_train_data['action'][idx, :],
    #     }
    #     return sample

def replace_submodules(
        root_module: nn.Module,
        predicate: Callable[[nn.Module], bool],
        func: Callable[[nn.Module], nn.Module]) -> nn.Module:
    """
    Replace all submodules selected by the predicate with
    the output of func.

    predicate: Return true if the module is to be replaced.
    func: Return new module to use.
    """
    """
    将所有通过 predicate 选择器选中的子模块替换为 func 返回的新模块。

    参数:
    root_module: 要修改的神经网络模型。
    predicate: 一个可调用对象，如果模块应该被替换则返回 True。
    func: 一个可调用对象，返回替换原有模块的新模块。

    返回:
    修改后的神经网络模型。
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

def replace_bn_with_gn(
        root_module: nn.Module,
        features_per_group: int=16) -> nn.Module:
    """
    Relace all BatchNorm layers with GroupNorm.
    """
    replace_submodules(
        root_module=root_module,
        predicate=lambda x: isinstance(x, nn.BatchNorm2d),
        func=lambda x: nn.GroupNorm(num_groups=x.num_features//features_per_group, num_channels=x.num_features)
    )
    return root_module

if __name__=="__main__":
    # 打印分隔线
    print('-'*50)

    # parameters
    obs_horizon = 2
    pred_horizon = 8
    action_horizon = 4
    # |o|o|                             observations: 2
    # | |a|a|a|a|a|a|a|a|               actions executed: 8
    # |p|p|p|p|p|p|p|p|p|p|p|p|p|p|p|p| actions predicted: 16

    # 构建ResNet18编码器
    vision_encoder1 = get_resnet('resnet18')
    # 将所有BatchNorm替换为GroupNorm以与EMA一起工作
    # 如果忘记这样做，性能将大幅下降
    vision_encoder1 = replace_bn_with_gn(vision_encoder1)

    vision_encoder2 = get_resnet('resnet18')
    vision_encoder2 = replace_bn_with_gn(vision_encoder2)

    # tactile_encoder = get_resnet('resnet18')
    # tactile_encoder = replace_bn_with_gn(tactile_encoder)
    tactile_encoder = TactileEncoder()

    # ResNet18的输出维度为512
    vision_feature_dim = 512
    tactile_feature_dim = 16
    lowdim_obs_dim = 4 + 4 + 6  # imu + gripper + robot
    # 每步的观察特征总维度
    obs_dim = vision_feature_dim * 2 + tactile_feature_dim + lowdim_obs_dim

    action_dim = 4 + 6  # gripper + robot

    # create network object
    # 创建网络对象
    noise_pred_net = ConditionalUnet1D(
        input_dim=action_dim,
        global_cond_dim=obs_dim * obs_horizon
    )

    # the final arch has 2 parts
    # 最终架构有两个部分
    nets = nn.ModuleDict({
        'vision_encoder1': vision_encoder1,
        'vision_encoder2': vision_encoder2,
        'tactile_encoder': tactile_encoder,
        'noise_pred_net': noise_pred_net
    })

    # # demo
    # with torch.no_grad():
    #     # example inputs
    #     # 示例输入
    #     image1 = torch.zeros((1, obs_horizon, 3, 96, 96))
    #     image2 = torch.zeros((1, obs_horizon, 3, 96, 96))
    #     tactile = torch.zeros((1, obs_horizon, 3, 96, 96))
    #     imu = torch.zeros((1, obs_horizon, 4))
    #     actionObs = torch.zeros((1, obs_horizon, 10))
    #     # (1,2,2)
    #     action = torch.zeros((1, obs_horizon, action_dim))
    #     # vision encoder
    #     # 视觉编码器
    #     print("image1.shape:", image1.shape)  # torch.Size([1, 2, 3, 96, 96])
    #     image_features1 = nets['vision_encoder1'](image1.flatten(end_dim=1))
    #     print("image_features1.shape:", image_features1.shape)  # torch.Size([2, 512])
    #     image_features2 = nets['vision_encoder2'](image2.flatten(end_dim=1))
    #     tactile_features = nets['tactile_encoder'](tactile.flatten(end_dim=1))
    #
    #     # (1,2,512)
    #     image_features1 = image_features1.reshape(*image1.shape[:2], -1)
    #     image_features2 = image_features2.reshape(*image2.shape[:2], -1)
    #     tactile_features = tactile_features.reshape(*tactile.shape[:2], -1)
    #     # (1,2,514)
    #     obs = torch.cat([image_features1, image_features2, tactile_features, imu, actionObs], dim=-1)
    #     # print("obs.shape:", obs.shape)  # torch.Size([1, 2, 1546])
    #
    #     # (1, 16, 2)，torch.randn()函数用于生成具有标准正态分布的随机数，即均值为0，标准差为1的随机数
    #     noised_action = torch.randn((1, pred_horizon, action_dim))
    #     print("noised_action.shape:", noised_action.shape)
    #     diffusion_iter = torch.zeros((1,))
    #
    #     # the noise prediction network
    #     # takes noisy action, diffusion iteration and observation as input
    #     # predicts the noise added to action
    #     # 噪声预测网络预测添加到动作的噪声，接受（带噪声的动作，扩散迭代和观察）作为输入
    #     noise = nets['noise_pred_net'](
    #         sample=noised_action,
    #         timestep=diffusion_iter,
    #         global_cond=obs.flatten(start_dim=1))  # 1028
    #
    #     # illustration of removing noise
    #     # the actual noise removal is performed by NoiseScheduler
    #     # and is dependent on the diffusion noise schedule
    #     # 去噪动作 = 带噪声的动作 - 噪声
    #     denoised_action = noised_action - noise

    '''
    接下来，代码设置了一个DDPMScheduler，用于在100个扩散迭代中进行操作：
    '''
    # for this demo, we use DDPMScheduler with 100 diffusion iterations
    # 对于这个演示，我们使用DDPMScheduler，并进行100个扩散迭代
    num_diffusion_iters = 100
    noise_scheduler = DDPMScheduler(
        num_train_timesteps=num_diffusion_iters,
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
    device = torch.device('cuda:0')
    _ = nets.to(device)
    num_epochs = 100
    # 指数移动平均（EMA）
    # 加速训练并提高稳定性
    # 保存模型权重的副本
    # power 参数是一个超参数，它决定了在计算移动平均时的衰减率。
    # 具体来说，power 控制了历史数据对于当前估计的影响程度。
    ema = EMAModel(
        parameters=nets.parameters(),
        power=0.75)

    dataset_raw_path = 'data/dataset_raw/'
    dataset_path = 'data/dataset/'

    record_time = time.time()
    command = "train"
    if command == "train":
        print("command:", command)

        dataset_exist_flag = True
        if not dataset_exist_flag:
            print('making dataset !!!')
            ### create dataset from file
            ### 从文件创建数据集，分批次保存数据
            file_name_all = os.listdir(dataset_raw_path)
            image_file_count = 0
            for idx, item in enumerate(file_name_all):
                if 'frame_merge.mp4' in item:
                    print('processing:', item)
                    dataset = RobotExcavationDataset(
                        dataset_raw_path=dataset_raw_path,
                        image_name=item,
                        pred_horizon=pred_horizon,
                        obs_horizon=obs_horizon,
                        action_horizon=action_horizon
                    )
                    np.save(dataset_path+'robot_excavation_'+str(image_file_count)+'.npy', dataset)
                    image_file_count += 1
                    print('saved dataset:', idx, item)
        else:
            print('dataset existed !!!')

        # create dataloader
        dataset_name_all = os.listdir(dataset_path)
        with tqdm(range(num_epochs), desc='Epoch') as tglobal:
            epoch_loss = []
            for epoch_idx in tglobal:  # tglobal是一个变量名，它是用于引用tqdm迭代器的，epoch_idx=0,1,2,...
                epoch_loss_temp = []
                # 遍历所有训练集
                for dataset_idx in range(len(dataset_name_all)):
                    merge_num = 51
                    if dataset_idx % merge_num == 0 and dataset_idx < len(dataset_name_all):
                        print(f"\ndataset_idx: {dataset_idx}/{len(dataset_name_all)} dataset_name: {dataset_name_all[dataset_idx]}")
                        print("dataset_idx:", dataset_idx)
                        for i in range(merge_num):
                            print(f"reading dataset num:{i}")
                            if i == 0:
                                dataset = np.load(dataset_path+dataset_name_all[dataset_idx+i], allow_pickle=True)
                            else:
                                dataset_new = np.load(dataset_path+dataset_name_all[dataset_idx+i], allow_pickle=True)
                                dataset = np.concatenate((dataset, dataset_new))

                        dataloader = torch.utils.data.DataLoader(
                            dataset,
                            batch_size=16,
                            num_workers=2,
                            shuffle=True,  # 打乱数据
                            # accelerate cpu-gpu transfer
                            # 加速CPU-GPU传输
                            pin_memory=True,
                            # don't kill worker process afte each epoch
                            # 不要在每个epoch后杀死工作进程
                            persistent_workers=True
                        )

                        # visualize data in batch
                        # 可视化批次中的数据
                        batch = next(iter(dataloader))
                        print("batch['image1'].shape:", batch['image1'].shape)
                        print("batch['image2'].shape:", batch['image2'].shape)
                        print("batch['tactile'].shape", batch['tactile'].shape)
                        print("batch['imu'].shape", batch['imu'].shape)
                        print("batch['actionObs'].shape", batch['actionObs'].shape)
                        print("batch['action'].shape", batch['action'].shape)

                        # 打印分隔线
                        print('-' * 50)
                        print('\n')

                        # Standard ADAM optimizer
                        # Note that EMA parametesr are not optimized
                        # 标准ADAM优化器
                        # 注意EMA参数不被优化
                        optimizer = torch.optim.AdamW(
                            params=nets.parameters(),
                            lr=1e-4, weight_decay=1e-6)

                        # 余弦学习率调度器（Cosine Learning Rate Scheduler）是一种常用的学习率衰减策略，它基于余弦函数来降低学习率，通常与线性预热（Linear Warmup）结合使用
                        # Cosine LR schedule with linear warmup
                        # 余弦LR计划，带有线性预热
                        # 余弦LR计划：余弦学习率调度器会随着训练的进行逐渐减小学习率，其变化规律类似于余弦函数，即在训练初期快速降低，在训练后期缓慢降低。
                        # 带有线性预热：线性预热是指在训练的开始阶段，学习率从较小的值线性增加到一个预设的最大值，这样做可以帮助模型在训练初期更快地收敛。
                        lr_scheduler = get_scheduler(
                            name='cosine',
                            optimizer=optimizer,
                            num_warmup_steps=500,
                            num_training_steps=len(dataloader) * num_epochs
                        )

                        '''
                        接下来，代码进入训练循环，其中每个epoch都包含一个批处理循环，用于优化模型：
                        '''
                        # fig, ax = plt.subplots()
                        # tqdm对象被创建并用于追踪进度
                        # range(num_epochs): 这是一个Python内置函数，用于生成一个从0到num_epochs - 1的数字序列。在这个例子中，它表示训练过程中的总迭代次数（即总的epoch数量）
                        # desc='Epoch': 这个参数是可选的，用于设置进度条的描述。在这个例子中，进度条将显示“Epoch”作为其描述
                        # with tqdm(range(num_epochs), desc='Epoch') as tglobal:
                        print("tglobal:", tglobal)
                        # epoch loop
                        # nets.load_state_dict(torch.load('./model_save/ema_model_59.pth'))
                        # _ = nets.to(device)
                        # batch loop
                        with tqdm(dataloader, desc='Batch', leave=False) as tepoch:
                            for nbatch in tepoch:
                                # data normalized in dataset
                                # device transfer
                                # 数据在数据集中标准化
                                # 设备转移
                                nimage1 = nbatch['image1'][:, :obs_horizon].to(device).float()  # torch.Size([8, 2, 3, 640, 480])
                                nimage2 = nbatch['image2'][:, :obs_horizon].to(device).float()
                                ntactile = nbatch['tactile'][:, :obs_horizon].to(device).float()  # torch.Size([8, 2, 3, 16, 16])
                                nimu = nbatch['imu'][:, :obs_horizon].to(device).float()
                                nactionObs = nbatch['actionObs'].to(device).float()
                                naction = nbatch['action'].to(device).float()
                                B = nimage1.shape[0]  # B = batch number

                                # torch.Size([16, 2, 3, 96, 96]), nimage.shape
                                # torch.Size([16, 2, 2]), nagent_pos.shape
                                # torch.Size([16, 16, 2]), naction.shape
                                # 16, B
                                # print("nimage.shape, nagent_pos.shape, naction.shape, B:", nimage.shape, nagent_pos.shape, naction.shape, B)

                                # image visualize
                                # img_show = nimage[0, 1].permute((1, 2, 0)).cpu().numpy()
                                # print("image: ", img_show.shape)
                                # ax.cla()   # 清除键
                                # ax.imshow(img_show.astype(np.uint8))
                                # plt.pause(0.1)
                                # breakpoint()

                                # encoder vision features
                                # 编码器视觉特征
                                # print('nimage1.shape:', nimage1.shape)  # torch.Size([8, 2, 3, 640, 480])
                                image_features1 = nets['vision_encoder1'](nimage1.flatten(end_dim=1))  # torch.Size([16, 512])
                                image_features1 = image_features1.reshape(*nimage1.shape[:2], -1)  # 解包操作，它会取出nimage数组形状元组中的前两个元素
                                image_features2 = nets['vision_encoder2'](nimage2.flatten(end_dim=1))
                                image_features2 = image_features2.reshape(*nimage2.shape[:2], -1)  # 解包操作，它会取出nimage数组形状元组中的前两个元素
                                tactile_features = nets['tactile_encoder'](ntactile.flatten(end_dim=1))
                                tactile_features = tactile_features.reshape(*ntactile.shape[:2], -1)  # 解包操作，它会取出nimage数组形状元组中的前两个元素
                                # (B,obs_horizon,D)

                                # concatenate vision feature and low-dim obs
                                # 连接视觉特征和低维观察
                                obs_features = torch.cat([image_features1, image_features2, tactile_features, nimu, nactionObs], dim=-1)
                                obs_cond = obs_features.flatten(start_dim=1)
                                # (B, obs_horizon * obs_dim)

                                # sample noise to add to actions
                                # 对动作添加噪声进行采样，torch.randn()函数用于生成具有标准正态分布的随机数，即均值为0，标准差为1的随机数
                                noise = torch.randn(naction.shape, device=device)

                                # sample a diffusion iteration for each data point
                                # 对每个数据点随机采样一个扩散迭代
                                # num_train_timesteps = num_diffusion_iters = 100
                                timesteps = torch.randint(0, noise_scheduler.config.num_train_timesteps, (B,), device=device).long()
                                # print('timesteps:', timesteps)  # tensor([68, 12, 79, 29,  5, 88, 78, 62], device='cuda:0')
                                # add noise to the clean images according to the noise magnitude at each diffusion iteration
                                # (this is the forward diffusion process)
                                # 根据每个扩散迭代的噪声幅度向干净图像添加噪声
                                ###########################（这是正向扩散过程）###############################
                                noisy_actions = noise_scheduler.add_noise(naction, noise, timesteps)
                                # print('noisy_actions:', naction.shape, noise.shape, noisy_actions.shape)

                                # predict the noise residual
                                # 预测噪声残差
                                # print('noisy_actions.shape:', noisy_actions.shape)  # torch.Size([8, 8, 10])
                                noise_pred = noise_pred_net(noisy_actions, timesteps, global_cond=obs_cond)

                                # L2 loss
                                # L2损失
                                loss = nn.functional.mse_loss(noise_pred, noise)

                                # optimize
                                # 优化
                                loss.backward()
                                optimizer.step()
                                optimizer.zero_grad()
                                # step lr scheduler every batch
                                # this is different from standard pytorch behavior
                                # 每个批处理步进余弦学习率调度器
                                # 余弦学习率调度器（Cosine Learning Rate Scheduler）是一种常用的学习率衰减策略
                                # 这与标准的pytorch行为不同
                                lr_scheduler.step()  #

                                # update Exponential Moving Average of the model weights
                                # 更新模型权重的指数移动平均
                                ema.step(nets.parameters())

                                # logging
                                # 记录
                                loss_cpu = loss.item()
                                epoch_loss_temp.append(loss_cpu)
                                tepoch.set_postfix(loss=loss_cpu)

                # breakpoint()
                tglobal.set_postfix(loss=np.mean(epoch_loss_temp))
                epoch_loss.append(np.mean(epoch_loss_temp))
                with open('./model_save/loss_'+str(record_time)+'.txt', 'a') as f:
                    f.write(str(np.array(epoch_loss)))
                    f.close()
                if epoch_idx % 10 == 0:
                    torch.save(nets.state_dict(), "./model_save/ema_model_" + str(epoch_idx) + ".pth")
