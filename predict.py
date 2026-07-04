from typing import Callable
import numpy as np
import torch
import torch.nn as nn
import collections
import zarr
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from diffusers.training_utils import EMAModel
from diffusers.optimization import get_scheduler
from tqdm.auto import tqdm
import sys

sys.path.append('../..')
# extra import
from demo.DPRE.model import get_resnet
from demo.DPRE.model import ConditionalUnet1D

# from env.pusht.pusht_image_env import PushTImageEnv
# from dataset.pusht_image_dataset import PushTImageDataset
from push_env import PushTImageEnv

'''
indices = create_sample_indices(
    episode_ends=episode_ends,
    sequence_length=pred_horizon,
    pad_before=obs_horizon-1,
    pad_after=action_horizon-1)
'''
# parameters
# obs_horizon = 2
# action_horizon = 8
# pred_horizon = 16
# |o|o|                             observations: 2
# | |a|a|a|a|a|a|a|a|               actions executed: 8
# |p|p|p|p|p|p|p|p|p|p|p|p|p|p|p|p| actions predicted: 16
def create_sample_indices(
        episode_ends: np.ndarray,
        sequence_length: int,
        pad_before: int = 0,  # obs_horizon-1
        pad_after: int = 0):  # action_horizon-1
    indices = list()
    for i in range(len(episode_ends)):
        start_idx = 0
        if i > 0:
            start_idx = episode_ends[i-1]
        end_idx = episode_ends[i]
        episode_length = end_idx - start_idx

        min_start = -pad_before
        max_start = episode_length - sequence_length + pad_after

        # range stops one idx before end
        for idx in range(min_start, max_start+1):
            buffer_start_idx = max(idx, 0) + start_idx
            buffer_end_idx = min(idx+sequence_length, episode_length) + start_idx
            start_offset = buffer_start_idx - (idx+start_idx)  # 初始那一步加一个偏置start_offset
            end_offset = (idx+sequence_length+start_idx) - buffer_end_idx

            sample_start_idx = 0 + start_offset
            sample_end_idx = sequence_length - end_offset
            indices.append([
                buffer_start_idx, buffer_end_idx,
                sample_start_idx, sample_end_idx])
    indices = np.array(indices)
    return indices


def sample_sequence(train_data, sequence_length,
                    buffer_start_idx, buffer_end_idx,
                    sample_start_idx, sample_end_idx):
    result = dict()
    for key, input_arr in train_data.items():
        sample = input_arr[buffer_start_idx:buffer_end_idx]
        data = sample
        if (sample_start_idx > 0) or (sample_end_idx < sequence_length):
            data = np.zeros(
                shape=(sequence_length,) + input_arr.shape[1:],
                dtype=input_arr.dtype)
            if sample_start_idx > 0:
                data[:sample_start_idx] = sample[0]
            if sample_end_idx < sequence_length:
                data[sample_end_idx:] = sample[-1]
            data[sample_start_idx:sample_end_idx] = sample
        result[key] = data
    return result

# normalize data
def get_data_stats(data):
    data = data.reshape(-1, data.shape[-1])
    stats = {
        'min': np.min(data, axis=0),
        'max': np.max(data, axis=0)
    }
    return stats

def normalize_data(data, stats):
    # nomalize to [0,1]
    ndata = (data - stats['min']) / (stats['max'] - stats['min'])
    # normalize to [-1, 1]
    ndata = ndata * 2 - 1
    return ndata

def unnormalize_data(ndata, stats):
    ndata = (ndata + 1) / 2
    data = ndata * (stats['max'] - stats['min']) + stats['min']
    return data


class PushTImageDataset(torch.utils.data.Dataset):
    def __init__(self,
                 dataset_path: str,
                 pred_horizon: int,
                 obs_horizon: int,
                 action_horizon: int):

        # read from zarr dataset
        dataset_root = zarr.open(dataset_path, 'r')
        print("dataset_root:", dataset_root)

        # float32, [0,1], (N,96,96,3)
        train_image_data = dataset_root['data']['img'][:]
        train_image_data = np.moveaxis(train_image_data, -1, 1)
        # (N,3,96,96)
        print("train_image_data.shape:", train_image_data.shape)

        # (N, D)
        train_data = {
            # first two dims of state vector are agent (i.e. gripper) locations
            'agent_pos': dataset_root['data']['state'][:,:2],
            'action': dataset_root['data']['action'][:]
        }
        state = dataset_root['data']['state'][:,:2]
        action = dataset_root['data']['action'][:]
        episode_ends = dataset_root['meta']['episode_ends'][:]
        print("state.shape:", state.shape)  # (25650, 2)
        print("action.shape:", action.shape)  # (25650, 2)
        print("episode_ends.shape:", episode_ends.shape)

        # compute start and end of each state-action sequence
        # also handles padding
        print('-'*50, "in create_sample_indices")
        # print('episode_ends', episode_ends)

        # |o|o|                             observations: 2
        # | |a|a|a|a|a|a|a|a|               actions executed: 8
        # |p|p|p|p|p|p|p|p|p|p|p|p|p|p|p|p| actions predicted: 16
        indices = create_sample_indices(
            episode_ends=episode_ends,
            sequence_length=pred_horizon,
            pad_before=obs_horizon-1,
            pad_after=action_horizon-1)
        print("indices.shape:", indices.shape)
        print("indices[:16, :]:", indices[:16, :])

        # compute statistics and normalized data to [-1,1]
        # 状态和动作归一化
        stats = dict()
        normalized_train_data = dict()
        for key, data in train_data.items():
            # print("key, data:", key, data)
            stats[key] = get_data_stats(data)
            normalized_train_data[key] = normalize_data(data, stats[key])

        # images are already normalized
        normalized_train_data['image'] = train_image_data

        self.indices = indices
        self.stats = stats
        self.normalized_train_data = normalized_train_data
        self.pred_horizon = pred_horizon
        self.action_horizon = action_horizon
        self.obs_horizon = obs_horizon

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, idx):
        # get the start/end indices for this datapoint
        buffer_start_idx, buffer_end_idx, \
            sample_start_idx, sample_end_idx = self.indices[idx]

        # get nomralized data using these indices
        nsample = sample_sequence(
            train_data=self.normalized_train_data,
            sequence_length=self.pred_horizon,
            buffer_start_idx=buffer_start_idx,
            buffer_end_idx=buffer_end_idx,
            sample_start_idx=sample_start_idx,
            sample_end_idx=sample_end_idx
        )

        # discard unused observations
        nsample['image'] = nsample['image'][:self.obs_horizon,:]
        nsample['agent_pos'] = nsample['agent_pos'][:self.obs_horizon,:]
        return nsample


# def get_resnet(name:str, weights=None, **kwargs) -> nn.Module:
#     """
#     name: resnet18, resnet34, resnet50
#     weights: "IMAGENET1K_V1", None
#     """
#     # Use standard ResNet implementation from torchvision
#     func = getattr(torchvision.models, name)
#     resnet = func(weights=weights, **kwargs)
#
#     # remove the final fully connected layer
#     # for resnet18, the output dim should be 512
#     resnet.fc = torch.nn.Identity()
#     return resnet


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
        in root_module.named_modules(remove_duplicate=True) # 此处假设 remove_duplicate=True 存在
        if predicate(m)]
    # 遍历需要被替换的模块路径列表
    for *parent, k in bn_list:
        # 获取父模块
        parent_module = root_module
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
    assert len(bn_list) == 0  #"仍有模块未被替换"
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
    print('demo the PushImageEnv:')
    # 0. create env object
    # 创建环境对象
    env = PushTImageEnv()

    # 1. seed env for initial state.
    # Seed 0-200 are used for the demonstration dataset.
    # 设置环境种子以获得初始状态
    # 种子0-200用于演示数据集
    env.seed(1000)

    # 2. must reset before use
    # 在使用前必须重置环境
    obs, info = env.reset()

    # 3. 2D positional action space [0, 512]
    # 2D位置动作空间 [0, 512]
    action = env.action_space.sample()
    print("action.shape (env.action_space.sample()):", action.shape)  # (2,)

    # 4. Standard gym step method
    # obs, reward, terminated, info = env.step(action)
    # 使用标准的gym步骤方法
    obs, reward, terminated, truncated, info = env.step(action)

    # prints and explains each dimension of the observation and action vectors
    # # 打印观察和动作向量的每个维度
    with np.printoptions(precision=4, suppress=True, threshold=5):
        print("obs['image'].shape:", obs['image'].shape)
        print("obs['agent_pos'].shape:", obs['agent_pos'].shape)
        print("action.shape: ", action.shape)

    # 数据集路径
    dataset_path = "data/pusht_cchi_v7_replay.zarr"
    # print("os.path.isfile(dataset_path):", os.path.isfile(dataset_path))
    # 如果文件不存在，则下载
    # if not os.path.isfile(dataset_path):
    #     id = "1KY1InLurpMvJDRb14L9NlXT_fEsCvVUq&confirm=t"
    #     gdown.download(id=id, output=dataset_path, quiet=False)

    # parameters
    pred_horizon = 16
    obs_horizon = 2
    action_horizon = 8
    # |o|o|                             observations: 2
    # | |a|a|a|a|a|a|a|a|               actions executed: 8
    # |p|p|p|p|p|p|p|p|p|p|p|p|p|p|p|p| actions predicted: 16


    # print(break_down)
    # @markdown ### **Network Demo**

    # construct ResNet18 encoder
    # if you have multiple camera views, use seperate encoder weights for each view.
    # 构建ResNet18编码器
    vision_encoder = get_resnet('resnet18')

    # IMPORTANT!
    # replace all BatchNorm with GroupNorm to work with EMA
    # performance will tank if you forget to do this!
    # 将所有BatchNorm替换为GroupNorm以与EMA一起工作
    # 如果忘记这样做，性能将大幅下降
    vision_encoder = replace_bn_with_gn(vision_encoder)

    # ResNet18 has output dim of 512
    # ResNet18的输出维度为512
    vision_feature_dim = 512
    # agent_pos is 2 dimensional
    # agent_pos是2维的
    lowdim_obs_dim = 2
    # observation feature has 514 dims in total per step
    # 每步的观察特征总维度为514
    obs_dim = vision_feature_dim + lowdim_obs_dim
    action_dim = 2
    obs_horizon = 2
    pred_horizon = 16

    # create network object
    # 创建网络对象
    noise_pred_net = ConditionalUnet1D(
        input_dim=action_dim,
        global_cond_dim=obs_dim * obs_horizon  # 514 * 2
    )

    # the final arch has 2 parts
    # 最终架构有两个部分
    nets = nn.ModuleDict({
        'vision_encoder': vision_encoder,
        'noise_pred_net': noise_pred_net
    })

    # demo
    # 演示
    with torch.no_grad():
        # example inputs
        # 示例输入
        image = torch.zeros((1, obs_horizon, 3, 96, 96))
        # (1,2,2)
        agent_pos = torch.zeros((1, obs_horizon, 2))
        # vision encoder
        # 视觉编码器
        image_features = nets['vision_encoder'](image.flatten(end_dim=1))

        # (1,2,512)
        image_features = image_features.reshape(*image.shape[:2], -1)
        # (1,2,514)
        obs = torch.cat([image_features, agent_pos], dim=-1)

        # (1, 16, 2)，torch.randn()函数用于生成具有标准正态分布的随机数，即均值为0，标准差为1的随机数
        noised_action = torch.randn((1, pred_horizon, action_dim))
        diffusion_iter = torch.zeros((1,))

        # the noise prediction network
        # takes noisy action, diffusion iteration and observation as input
        # predicts the noise added to action
        # 噪声预测网络预测添加到动作的噪声，接受（带噪声的动作，扩散迭代和观察）作为输入
        noise = nets['noise_pred_net'](
            sample=noised_action,
            timestep=diffusion_iter,
            global_cond=obs.flatten(start_dim=1))  # 1028

        # illustration of removing noise
        # the actual noise removal is performed by NoiseScheduler
        # and is dependent on the diffusion noise schedule
        # 去噪动作 = 带噪声的动作 - 噪声
        denoised_action = noised_action - noise

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
    device = torch.device('cuda')
    _ = nets.to(device)
    num_epochs = 100
    # Exponential Moving Average
    # accelerates training and improves stability
    # holds a copy of the model weights
    # 指数移动平均（EMA）
    # 加速训练并提高稳定性
    # 保存模型权重的副本
    # power 参数是一个超参数，它决定了在计算移动平均时的衰减率。
    # 具体来说，power 控制了历史数据对于当前估计的影响程度。
    ema = EMAModel(
        parameters=nets.parameters(),
        power=0.75)

    command = "test"
    if command == "train":
        print("command:", command)

        # create dataset from file
        # 从文件创建数据集
        dataset = PushTImageDataset(
            dataset_path=dataset_path,
            pred_horizon=pred_horizon,
            obs_horizon=obs_horizon,
            action_horizon=action_horizon
        )
        # save training data statistics (min, max) for each dim
        # 保存训练数据统计（最小值，最大值）每个维度
        stats = dataset.stats
        print("stats", stats)

        # create dataloader
        dataloader = torch.utils.data.DataLoader(
            dataset,
            batch_size=16,
            num_workers=1,
            shuffle=True,
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
        print("batch['image'].shape:", batch['image'].shape)
        print("batch['agent_pos'].shape:", batch['agent_pos'].shape)
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
        with tqdm(range(num_epochs), desc='Epoch') as tglobal:
            print("tglobal:", tglobal)
            # epoch loop
            for epoch_idx in tglobal:  # tglobal是一个变量名，它是用于引用tqdm迭代器的，epoch_idx=0,1,2,...
                epoch_loss = list()
                # batch loop
                with tqdm(dataloader, desc='Batch', leave=False) as tepoch:
                    print("tepoch:", tepoch)
                    for nbatch in tepoch:
                        # data normalized in dataset
                        # device transfer
                        # 数据在数据集中标准化
                        # 设备转移
                        nimage = nbatch['image'][:, :obs_horizon].to(device)
                        nagent_pos = nbatch['agent_pos'][:, :obs_horizon].to(device)
                        naction = nbatch['action'].to(device)
                        B = nagent_pos.shape[0]  # B = batch number

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
                        image_features = nets['vision_encoder'](nimage.flatten(end_dim=1))
                        image_features = image_features.reshape(*nimage.shape[:2], -1)  # 解包操作，它会取出nimage数组形状元组中的前两个元素
                        # (B,obs_horizon,D)

                        # concatenate vision feature and low-dim obs
                        # 连接视觉特征和低维观察
                        obs_features = torch.cat([image_features, nagent_pos], dim=-1)
                        obs_cond = obs_features.flatten(start_dim=1)
                        # (B, obs_horizon * obs_dim)

                        # sample noise to add to actions
                        # 对动作添加噪声进行采样，torch.randn()函数用于生成具有标准正态分布的随机数，即均值为0，标准差为1的随机数
                        noise = torch.randn(naction.shape, device=device)

                        # sample a diffusion iteration for each data point
                        # 对每个数据点随机采样一个扩散迭代
                        # num_train_timesteps = num_diffusion_iters = 100
                        timesteps = torch.randint(0, noise_scheduler.config.num_train_timesteps, (B,), device=device).long()

                        # add noise to the clean images according to the noise magnitude at each diffusion iteration
                        # (this is the forward diffusion process)
                        # 根据每个扩散迭代的噪声幅度向干净图像添加噪声
                        ###########################（这是正向扩散过程）###############################
                        noisy_actions = noise_scheduler.add_noise(naction, noise, timesteps)
                        # print('noisy_actions', naction.shape, noise.shape, noisy_actions.shape)

                        # predict the noise residual
                        # 预测噪声残差
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
                        epoch_loss.append(loss_cpu)
                        tepoch.set_postfix(loss=loss_cpu)

                        # breakpoint()
                tglobal.set_postfix(loss=np.mean(epoch_loss))

                if epoch_idx % 10 == 0:
                    torch.save(nets.state_dict(), "./model/ema_model_" + str(epoch_idx) + ".pth")

    if command == "test":
        print("command:", command)
        '''
        最后，代码将EMA模型的权重用于推理，并设置了一个环境交互的评估循环，以执行pushT任务：
        '''
        # Weights of the EMA model
        # is used for inference
        # EMA模型的权重
        # 用于推理
        ema_nets = nets
        ema.copy_to(ema_nets.parameters())
        ema_nets.load_state_dict(torch.load('./model_save/ema_model_39.pth'))
        _ = ema_nets.to(device)

        stats = {'agent_pos': {'min': np.array([13.456424, 32.938293]), 'max': np.array([496.14618, 510.9579])},
                 'action': {'min': np.array([12., 25.]), 'max': np.array([511., 511.])}}
        # limit enviornment interaction to 200 steps before termination
        # 设置环境交互的最大步数为200步
        max_steps = 2000
        env = PushTImageEnv()
        # use a seed >200 to avoid initial states seen in the training dataset
        # 使用一个大于200的种子以避免在训练数据集中出现的初始状态
        env.seed(100000)

        # get first observation
        # 获取第一个观察值
        obs, info = env.reset()

        # keep a queue of last 2 steps of observations
        # 维护一个队列，存储最后2步的观察值，deque类似列表(list)的容器，实现了在两端快速添加(append)和弹出(pop)
        obs_deque = collections.deque(
            [obs] * obs_horizon, maxlen=obs_horizon)  # obs_horizon = 2
        # save visualization and rewards
        # 保存可视化图像和奖励
        imgs = [env.render(mode='rgb_array')]
        rewards = list()
        done = False
        step_idx = 0

        '''
        # 使用tqdm创建一个进度条，用于显示评估过程中的进度
        '''
        with tqdm(total=max_steps, desc="Eval PushTImageEnv") as pbar:
            while not done:
                B = 1
                # stack the last obs_horizon number of observations
                # print([x for x in obs_deque])
                # 将最后obs_horizon数量的观察值堆叠起来
                images = np.stack([x['image'] for x in obs_deque])
                agent_poses = np.stack([x['agent_pos'] for x in obs_deque])

                # normalize observation
                # 观察值标准化
                nagent_poses = normalize_data(agent_poses, stats=stats['agent_pos'])
                # images are already normalized to [0,1]
                nimages = images

                # device transfer
                # 将数据转移到设备上
                nimages = torch.from_numpy(nimages).to(device, dtype=torch.float32)
                # (2,3,96,96)
                nagent_poses = torch.from_numpy(nagent_poses).to(device, dtype=torch.float32)
                # (2,2)

                # infer action
                # 使用模型进行动作预测
                with torch.no_grad():
                    # get image features
                    # 获取图像特征
                    image_features = ema_nets['vision_encoder'](nimages)
                    # (2,512)

                    # concat with low-dim observations
                    # 将图像特征与低维观察值连接起来
                    obs_features = torch.cat([image_features, nagent_poses], dim=-1)

                    # reshape observation to (B,obs_horizon*obs_dim)
                    # 将观察值重塑为(B,obs_horizon*obs_dim)
                    obs_cond = obs_features.unsqueeze(0).flatten(start_dim=1)

                    # initialize action from Guassian noise
                    # 从高斯噪声中初始化动作
                    noisy_action = torch.randn(
                        (B, pred_horizon, action_dim), device=device)
                    naction = noisy_action

                    # init scheduler
                    # 初始化调度器
                    noise_scheduler.set_timesteps(num_diffusion_iters)

                    # 执行反向扩散过程，逐步去除噪声
                    for k in noise_scheduler.timesteps:
                        # predict noise
                        noise_pred = ema_nets['noise_pred_net'](
                            sample=naction,
                            timestep=k,
                            global_cond=obs_cond
                        )

                        # inverse diffusion step (remove noise)
                        naction = noise_scheduler.step(
                            model_output=noise_pred,
                            timestep=k,
                            sample=naction
                        ).prev_sample

                # unnormalize action
                # 将动作反标准化
                naction = naction.detach().to('cpu').numpy()
                # (B, pred_horizon, action_dim)
                naction = naction[0]
                action_pred = unnormalize_data(naction, stats=stats['action'])

                # only take action_horizon number of actions
                # 只取action_horizon数量的动作
                start = obs_horizon - 1
                end = start + action_horizon
                action = action_pred[start:end, :]
                # (action_horizon, action_dim)

                # execute action_horizon number of steps
                # without replanning
                # 执行action_horizon数量的步骤，期间不重新规划
                for i in range(len(action)):
                    # stepping env
                    obs, reward, done, _, info = env.step(action[i])
                    # save observations
                    obs_deque.append(obs)
                    # and reward/vis
                    rewards.append(reward)
                    imgs.append(env.render(mode='rgb_array'))

                    # update progress bar
                    # 更新进度条
                    step_idx += 1
                    pbar.update(1)
                    pbar.set_postfix(reward=reward)
                    if step_idx > max_steps:
                        done = True
                    if done:
                        break

        # print out the maximum target coverage
        # 打印出最大目标覆盖范围
        print('Score:', max(rewards))

        # visualize
        # 可视化
        from skvideo.io import vwrite
        from IPython.display import Video

        vwrite('vis.mp4', imgs)
        Video('vis.mp4', embed=True, width=256, height=256)

