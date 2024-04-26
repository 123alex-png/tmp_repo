import torch
import torch.nn as nn
import torch.nn.functional as F


class Critic(nn.Module):

    def __init__(self, arglist):
        super().__init__()
        self.embedding_num_units = 128
        self.hidden_num_units = 128
        self.parameter_init_min = -0.08
        self.parameter_init_max = 0.08
        self.num_glimpse = 1
        self.n_process = 3  # 使用glimpse的次数
        self.softmax_T = 1.0  # 可能改善探索；softmax温度默认值为1.0，但2.0、2.2和1.5可能会产生更好的结果
        self.embed_dim = 3
        self.embedding_layer = nn.Linear(self.embed_dim,
                                         self.embedding_num_units,
                                         bias=False)  # 输入为三维坐标 神经元个数128
        self.Encoder = nn.LSTM(
            self.embedding_num_units, self.hidden_num_units, batch_first=True
        )  # batch_first   一次喂入 batch_size 个路线点集合 一个集合包含20/100个点 一个点是三个坐标xyz （batch_size, 100,3）
        self.Decoder = nn.LSTM(self.embedding_num_units,
                               self.hidden_num_units,
                               batch_first=True)

        if torch.cuda.is_available():
            self.Vec = nn.Parameter(
                torch.cuda.FloatTensor(
                    self.embedding_num_units))  # 将embed输入的转换为tensor
        else:
            self.Vec = nn.Parameter(torch.FloatTensor(
                self.embedding_num_units))  # 将embed输入的转换为tensor

        self.W_q = nn.Linear(self.hidden_num_units,
                             self.hidden_num_units,
                             bias=True)  # query的参数矩阵 用于训练
        self.W_ref = nn.Conv1d(self.hidden_num_units, self.hidden_num_units, 1,
                               1)  # why conv1d?

        self.layer_2FC = nn.Sequential(
            nn.Linear(self.hidden_num_units, self.hidden_num_units,
                      bias=False), nn.ReLU(),
            nn.Linear(self.hidden_num_units, 1, bias=False))

        # init parameter
        self._initialize_weights(self.parameter_init_min,
                                 self.parameter_init_max)

    def _initialize_weights(self, init_min, init_max):
        for param in self.parameters():
            nn.init.uniform_(param.data, init_min, init_max)

    def forward(self, x, device):
        """

        :param x: (batch_size, view_point_num ,3)
        :param device:
        :return:
        """
        # 1. 喂入embedding
        x = x.to(device)
        batch_size, view_point_num, xyz = x.size()
        embed_encoder_inputs = self.embedding_layer(
            x)  # 输入embed后得到的输出是encoder的输入  128*1
        embed = embed_encoder_inputs.size(2)
        # 2. 喂入encoder
        encoder_h, (h, c) = self.Encoder(
            embed_encoder_inputs,
            None)  # encoder 输出有两个 一个是h_t=encoder_h  一个是C_t= (h,c)
        reference = encoder_h  # 来自编码器的一组隐藏状态。[h_1,h_2,...h_n]
        # h(1,batch_size,embed)
        query = h[-1]
        # 3. Attention
        for i in range(self.n_process):
            for i in range(self.num_glimpse):
                query = self.glimpse(query, reference)
        # 4. FC linear
        pred_length = self.layer_2FC(query).squeeze(-1).squeeze(-1)

        return pred_length

    def glimpse(self, query, reference, inf=1e8):
        """

        :param query:  s_t (batch_size,128)
        :param reference: [h_1,h_2,h_3,...h_n]  (batch_size, view_point_num , 128)
        :param inf:
        :return:
        """
        u1 = self.W_q(query)  # W_q * q   (512,128)
        u1 = u1.unsqueeze(-1).repeat(
            1, 1, reference.size(1))  # (512,128) -> (512,128,1)->(512,128,100)
        u2 = self.W_ref(
            reference.permute(0, 2, 1)
        )  # (batch_size, view_point_num , 128) -> (batch_size,128,view_point_num)
        V = self.Vec.unsqueeze(0).unsqueeze(0).repeat(
            reference.size(0), 1,
            1)  # 128->(1,128)->(1,1,128)->(batch_size,1,128)
        u = torch.bmm(V, torch.tanh(u1 + u2)).squeeze(1)
        # V: (batch, 1, 128) * u1+u2: (batch, 128, view_point_num) => u: (batch, 1, view_point_num) => (batch, view_point_num)
        a = F.softmax(u, dim=1)  # a_ti
        d = torch.bmm(u2, a.unsqueeze(2)).squeeze(2)
        # u2: (batch, 128, city_t) * a: (batch, city_t, 1) => d: (batch, 128)
        return d
