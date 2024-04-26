import torch
import torch.nn as nn
import torch.nn.functional as F


class Greedy(nn.Module):

    def __init__(self):
        super().__init__()

    def forward(self, log_p):
        return torch.argmax(log_p, dim=1).long()


class Categorical(nn.Module):

    def __init__(self):
        super().__init__()

    def forward(self, log_p):
        return torch.multinomial(log_p.exp(), 1).long().squeeze(1)


class PtrNet(nn.Module):

    def __init__(self, arglist):
        super().__init__()
        self.embedding_num_units = 128
        self.hidden_num_units = 128
        self.parameter_init_min = -0.08
        self.parameter_init_max = 0.08
        self.num_glimpse = 1
        self.clip_logits = 10  # 梯度剪裁 提升探索
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
            print(torch.cuda.is_available())
            self.Vec = nn.Parameter(
                torch.cuda.FloatTensor(
                    self.embedding_num_units))  # 将embed输入的转换为tensor
            self.Vec2 = nn.Parameter(
                torch.cuda.FloatTensor(self.embedding_num_units))  # Vec V参数矩阵
        else:
            self.Vec = nn.Parameter(torch.FloatTensor(
                self.embedding_num_units))  # 将embed输入的转换为tensor
            self.Vec2 = nn.Parameter(
                torch.FloatTensor(self.embedding_num_units))  # Vec ?

        self.W_q = nn.Linear(self.hidden_num_units,
                             self.hidden_num_units,
                             bias=True)  # query的参数矩阵 用于训练
        self.W_q2 = nn.Linear(self.hidden_num_units,
                              self.hidden_num_units,
                              bias=True)

        self.W_ref = nn.Conv1d(self.hidden_num_units, self.hidden_num_units, 1,
                               1)  # why conv1d?
        self.W_ref2 = nn.Conv1d(self.hidden_num_units, self.hidden_num_units,
                                1, 1)
        self.decoder_input = nn.Parameter(
            torch.FloatTensor(self.embedding_num_units))  # [128]

        # init parameter
        self._initialize_weights(self.parameter_init_min,
                                 self.parameter_init_max)

        #self.view_point_selector = Greedy()  # 采用贪心选择输出的城市
        self.view_point_selector = Categorical()  # 采用概率选择输出的城市

    def _initialize_weights(self, init_min, init_max):
        for param in self.parameters():
            nn.init.uniform_(param.data, init_min, init_max)

    def forward(self, x, device):
        """

        :param x: (batch_size, view_point_num ,3)
        :param device:
        :return:
        """
        x = x.to(device)
        batch_size, view_point_num, _ = x.size()
        embed_encoder_inputs = self.embedding_layer(
            x)  # 输入embed后得到的输出是encoder的输入  128*1 (128,1,view_point_num)?
        embed = embed_encoder_inputs.size(2)  # embed = view_point_num?

        mask = torch.zeros((batch_size, view_point_num),
                           device=device)  # (512,100) 标记哪些视点已经被输出 保证只会输出一次

        encoder_h, (h, c) = self.Encoder(
            embed_encoder_inputs,
            None)  # encoder 输出有两个 一个是h_t=encoder_h  一个是C_t= (h,c)
        reference = encoder_h  # 来自编码器的一组隐藏状态。[h_1,h_2,...h_n]
        PI_list = []  # 一个视点的排列
        log_ps = []  # 存每次输出的所有节点的概率分布

        decoder_input = self.decoder_input.unsqueeze(0).repeat(
            batch_size, 1
        ).unsqueeze(1).to(
            device
        )  # [128] -> [1,128] -> [batch_size,128] ->[batch_size,1,128]?https://blog.csdn.net/m0_58810879/article/details/121772339

        for i in range(view_point_num):
            _, (h, c) = self.Decoder(
                decoder_input,
                (h,
                 c))  # decoder input 输入[batch_size,1,128] 隐藏层节点特征维度(h,c)  依次输出
            query = h.squeeze(
                0)  # query: 当前解码器的隐藏状态[s_t] shape:(batch_size,128)

            for i in range(self.num_glimpse):
                query = self.glimpse(query, reference, mask)
            logits = self.pointer(query, reference,
                                  mask)  # 指针网络输出 (batch, view_point_num)

            log_p = torch.log_softmax(
                logits, dim=-1)  # 输出为0-1的概率分布 (batch, view_point_num)
            next_node = self.view_point_selector(
                log_p)  # 根据贪心or抽取 输出视点节点 (batch, 1)? 不一定对 打断点
            decoder_input = torch.gather(
                input=embed_encoder_inputs,
                dim=1,
                index=next_node.unsqueeze(1).unsqueeze(-1).repeat(1, 1, embed))
            PI_list.append(next_node)
            log_ps.append(log_p)  # 存每次输出的概率分布
            mask += torch.zeros(
                (batch_size, view_point_num),
                device=device).scatter_(dim=1,
                                        index=next_node.unsqueeze(1),
                                        value=1)

        PI = torch.stack(PI_list, dim=1)  # 将列表存为张量 用于输入critic？
        ll = self.get_log_likelihood(torch.stack(log_ps, 1),
                                     PI)  #   条件极大似然损失函数？

        return PI, ll

    def glimpse(self, query, reference, mask, inf=1e8):
        """

        :param query:  s_t (batch_size,128)
        :param reference: [h_1,h_2,h_3,...h_n]  (batch_size, view_point_num , 128)
        :param mask:   (batch_size, view_point_num)
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
            reference.shape[0], 1,
            1)  # 128->(1,128)->(1,1,128)->(batch_size,1,128)
        u = torch.bmm(V, torch.tanh(u1 + u2)).squeeze(1)
        # V: (batch, 1, 128) * u1+u2: (batch, 128, view_point_num) => u: (batch, 1, view_point_num) => (batch, view_point_num)

        u = u - inf * mask  # 若某视点已经被输出，则将该u计算为负无穷大
        a = F.softmax(u / self.softmax_T, dim=1)  # a_ti
        d = torch.bmm(u2, a.unsqueeze(2)).squeeze(2)
        # u2: (batch, 128, city_t) * a: (batch, city_t, 1) => d: (batch, 128)
        return d

    def pointer(self, query, reference, mask, inf=1e8):
        """
        :param query:  s_t (batch_size,128)
        :param reference: [h_1,h_2,h_3,...h_n]  (batch_size, view_point_num , 128)
        :param mask:   (batch_size, view_point_num)
        :param inf:
        :return:
        """
        u1 = self.W_q2(query)  # W_q * q   (512,128)
        u1 = u1.unsqueeze(-1).repeat(
            1, 1, reference.size(1))  # (512,128) -> (512,128,1)->(512,128,100)
        u2 = self.W_ref2(
            reference.permute(0, 2, 1)
        )  # (batch_size, view_point_num , 128) -> (batch_size,128,view_point_num)
        V = self.Vec2.unsqueeze(0).unsqueeze(0).repeat(
            reference.shape[0], 1,
            1)  # 128->(1,128)->(1,1,128)->(batch_size,1,128)
        u = torch.bmm(V, self.clip_logits * torch.tanh(u1 + u2)).squeeze(1)
        # V: (batch, 1, 128) * u1+u2: (batch, 128, view_point_num) => u: (batch, 1, view_point_num) => (batch, view_point_num)
        u = u - inf * mask  # 若某视点已经被输出，则将该u计算为负无穷大
        return u

    def get_log_likelihood(self, _log_p, pi):
        """	args:
            _log_p: (batch_size, view_point_num, view_point_num)
            pi: (batch_size, view_point_num), predicted tour
            return: (batch_size)
        """
        log_p = torch.gather(input=_log_p, dim=2, index=pi[:, :, None])
        return torch.sum(log_p.squeeze(-1), 1)
