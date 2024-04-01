
from torch.utils.data.dataset import Dataset
from Env import Env
from data_gen import data_gen

class Generator(Dataset):
    def __init__(self, arglist, env,num_samples):
        self.data = env.get_batch_nodes(num_samples)  # 采样


    def __getitem__(self, idx):
        return self.data[idx]

    def __len__(self):
        return self.data.size(0)
