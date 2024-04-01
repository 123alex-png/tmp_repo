import os.path

import numpy as np
import torch
import torch.nn as nn
from time import time
from Env import Env
from actor import PtrNet
from argument import argparser
from critic import Critic
from dataset import Generator
from torch.utils.data import DataLoader
from datetime import datetime
def train_model(arglist, env,log_path=None):


    date = datetime.now().strftime('%m%d_%H_%M')
    param_path = arglist.log_dir +'%s_%s_param.csv' % (date, arglist.view_point_num)  # arglist.log_dir = ./Csv/
    print(f'generate {param_path}')
    with open(param_path, 'w') as f:
        f.write(''.join('%s,%s\n' % item for item in vars(arglist).items()))


    num_samples = arglist.batch_size * arglist.max_episode
    #num_samples = 1000000
    print(num_samples)
    # ------------------------ init-------------------------------------
    actor_model = PtrNet(arglist)
    optimizer_a = torch.optim.Adam(actor_model.parameters(), lr=arglist.lr)
    # 学习率衰减
    # if arglist.lr_decay:
    #     actor_lr_sheduler = optim.lr_scheduler.StepLR(optimizer_a,
    # 					step_size=arglist.lr_decay_step, gamma=arglist.lr_decay)
    actor_model = actor_model.to(arglist.device)

    critic_model = Critic(arglist)
    optimizer_c = torch.optim.Adam(critic_model.parameters(), lr=arglist.lr)
    # 学习率衰减
    # if arglist.lr_decay:
    #     critic_lr_sheduler = optim.lr_scheduler.StepLR(optimizer_c,
    # 					step_size=arglist.lr_decay_step, gamma=arglist.lr_decay)

    critic_model = critic_model.to(arglist.device)
    average_critic_loss = 0.
    mse_loss = nn.MSELoss()
    # -----------------data --------------------====================================
    dataset = Generator(arglist,env,num_samples)  # 1000000 /512 = episode = 1953
    dataloader = DataLoader(dataset, batch_size=arglist.batch_size, shuffle=True,drop_last=True)

    # --------------train----------------------=======================================
    average_actor_loss = 0.
    average_length = 0.
    min_length = 1e7
    count = 0
    t1 = time()

    for i,inputs in enumerate(dataloader):
        # "addmm_cuda" not implemented for 'Long' cuda不支持整数张量矩阵乘法
        inputs = inputs.to(arglist.device)
        pred_tour, ll = actor_model(inputs,arglist.device) # 输入节点的顺序存在pred_tour  此输出未添加start end


        # if arglist.use_rrt == True:
        #    #  real_length = env.stack_rrt_length(inputs,pred_tour)
        #     real_length = env.stack_length_fast(inputs,pred_tour)
        #
        # else:
        #     #real_length = env.stack_length(inputs,pred_tour)  # actor实际输出的length
        #     real_length = env.stack_length_fast(inputs,pred_tour)  # actor实际输出的length
        real_length = env.stack_length_fast(inputs, pred_tour)  # actor实际输出的length
        # print("real_length: ")
        # print(real_length.detach())
        pred_length = critic_model(inputs,arglist.device)  # critic 预测的较好的length
        critic_loss = mse_loss(pred_length,real_length.detach())  # why?  因为不需要更新actor所以不需要real的梯度吗？
        optimizer_c.zero_grad()
        critic_loss.backward()
        # 梯度剪裁
        nn.utils.clip_grad_norm(critic_model.parameters(), max_norm=5., norm_type=2)
        optimizer_c.step()
        # if arglist.lr_decay:
        #     critic_lr_sheduler.step()


        error = real_length.detach() - pred_length.detach()
        actor_loss = (error * ll).mean()
        optimizer_a.zero_grad()
        actor_loss.backward()
        # 梯度剪裁
        nn.utils.clip_grad_norm(actor_model.parameters(), max_norm=5., norm_type=2)
        optimizer_a.step()
        # if arglist.lr_decay:
        #     actor_lr_sheduler.step()

        average_actor_loss += actor_loss.item()
        average_critic_loss += critic_loss.item()
        average_length += real_length.mean().item()

        if i % arglist.log_step == 0:
            t2 = time()
            print('episode:%d/%d, actic loss:%1.3f, critic loss:%1.3f, Length:%1.3f, %dmin%dsec' % (
                i, arglist.max_episode, average_actor_loss / (i + 1), average_critic_loss / (i + 1), average_length / (i + 1), (t2 - t1) // 60,
                (t2 - t1) % 60))

            if log_path is None:
                log_path = arglist.log_dir + '%s_%s_train.csv' % (date, arglist.view_point_num)  # cfg.log_dir = ./Csv/
                with open(log_path, 'w') as f:
                    f.write('episode,actor loss,critic loss,average distance,time\n')
            else:
                with open(log_path, 'a') as f:
                    f.write('%d,%1.4f,%1.4f,%1.4f,%dmin%dsec\n' % (
                    i, average_actor_loss / (i + 1), average_critic_loss / (i + 1), average_length / (i + 1), (t2 - t1) // 60,
                    (t2 - t1) % 60))
            if (average_length / (i + 1) < min_length):
                min_length = average_length / (i + 1)
            else:
                count += 1
                print(f'count: {count}/100')
                if (count >= 100):
                    print('early stop, average cost cant decrease anymore')
                    # torch.save(actor_model.state_dict(), arglist.model_dir + '%s_%s_episode%d_act.pt' % (
                    # arglist.view_point_num, date, i))  # 'cfg.model_dir = ./Pt/'
                    break
            t1 = time()
            if i % 5000 == 0:
                torch.save(actor_model.state_dict(),arglist.model_dir + '%s_%s_episode%d_act.pt' % (arglist.view_point_num, date, i))  # 'cfg.model_dir = ./Pt/'
    print('save model...')
    # last save model
    torch.save(actor_model.state_dict(), arglist.model_dir + '%s_%s_episode%d_act_last.pt' % (
        arglist.view_point_num, date, arglist.max_episode))  # 'cfg.model_dir = ./Pt/'
    # ==============test ===================
    test_input = env.get_nodes(1)
    # simplest way
    print('sampling ...')
    t1 = time()
    test_inputs = test_input.repeat(arglist.batch_size, 1, 1)
    pred_tours, _ = actor_model(test_inputs,arglist.device)
    if arglist.use_rrt == True:
        l_batch = env.stack_rrt_length(test_inputs, pred_tours)
    else:
        # l_batch = env.stack_length(inputs,pred_tour)  # actor实际输出的length
        l_batch = env.stack_length_fast(test_inputs, pred_tours)  # actor实际输出的length
    index_length_min = torch.argmin(l_batch)
    best_tour = pred_tours[index_length_min]
    t2 = time()
    print('%dmin %1.2fsec\n' % ((t2 - t1) // 60, (t2 - t1) % 60))
    # env.show(test_input, best_tour)

    env.show_rrt_html(test_input,best_tour)







def train(arglist):
    # ==================== 获取path_length_list
    # if os.path.exists('../data/path_length_list.txt'):
    #     f = open('../data/path_length_list.txt')
    #     path_length_list = f.readlines()  # 读取全部内容
    # else:
    #     path_length_list = []
    #path_length_list = np.empty([],dtype=float)

    ## subtask 中的开始点和结束点
    target_start = [10,10,1]
    target_end = [90,60,60]
    env = Env(arglist,target_start,target_end)



    if arglist.mode == "train":
        train_model(arglist, env)
    else:
        print(arglist.mode)
    return env


if __name__ == '__main__':
    arglist = argparser()
    torch.set_num_threads(3)
    train(arglist)

