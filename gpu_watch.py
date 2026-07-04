import torch

if torch.cuda.is_available():
    num_gpus = torch.cuda.device_count()
    print(f"GPU的数量: {num_gpus}")

    for i in range(num_gpus):
        print(torch.cuda.get_device_name(i))
else:
    print("当前没有可用的GPU")
