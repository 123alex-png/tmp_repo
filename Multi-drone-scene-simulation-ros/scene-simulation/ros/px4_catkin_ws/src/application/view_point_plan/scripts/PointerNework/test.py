import numpy as np

from Env import *
from PointerNework.SubTask import SubTask
from argument import argparser
import torch
import torch.nn as nn
from time import time
from actor import PtrNet
import os

from getViewPoint.getView import *

def cal_path(arglist,sub_task):

    # ========== 读取subTask ===============
    # 1.读取target_start target_end
    target_start = sub_task.target_start
    target_end = sub_task.target_end
    fight_height = 10
    photo_dis = 5
    safe_dis = 3

    # # 2. 读取目标的坐标  如果是桥面、桥梁、 边坡、路面 则读取的是四个点
    # #    如果是桥墩 则是八个点   桥台、伸缩缝是一个点
    # target_type_list = {'Deck':target_cal,'Bearing':target_cal,'Pier':target_cal,'Joint':target_cal,'Abutment':bridge_support,
    #                'SlopeBottom':target_cal,'SlopeSurface':target_cal,'SlopeDitch':target_cal,
    #                 'Barrier':target_cal,'Guardarail':target_cal,'RoadSurface':target_cal
    #                     }
    # # 注意  桥面任务包括 桥的路面部分和桥侧面部分
    # argument = [sub_task.point_list,photo_dis,fight_height]
    # if sub_task.task_type in target_type_list:
    #     print("当前任务类型是:{}".format(sub_task.task_type))
    #     # 加载模型   需区分是桥梁、边坡还是路面
    #
    #     actor_model_path = '../models/' + sub_task.target_name + '_' + str(sub_task.target_id) + '.pt'  # bridge_0.pt
    #     print(actor_model_path)
    #     view_point_list = target_type_list[sub_task.task_type](argument)

    # test code
    actor_model_path = '../Pt/20_0712_19_35_episode20000_act_last.pt'
    if sub_task.id == 0:
        view_point_list = bridge_deck([10, 10, 35], [85, 10, 35], [85, 50, 35], [10, 50, 35], 5, 5)  # 桥面
    elif sub_task.id == 1:
        view_point_list = bridge_sidedeck([10, 10, 32], [85, 10, 32], -5, 5)  # 桥侧面  问题
    elif sub_task.id == 2:
        view_point_list = bridge_squat([30, 20, 0], [35, 20, 0], [35, 25, 0], [30, 25, 0],
                                    [30, 20, 30], [35, 20, 30], [30, 25, 30], [30, 25, 30], 5, 5, 8, 8)  # 桥墩

    # 3. 载入环境  桥梁、边坡和路面的障碍物不同
    env = Env(arglist,target_start,target_end)

    # random
    print('generate tour...')
    data = torch.tensor(view_point_list,device=arglist.device)
    data = data.type(torch.float32)
    test_input = data
    # ======================load_model==================
    # simplest way
    print('sampling ...')
    t1 = time()
    test_inputs = test_input.repeat(arglist.batch_size, 1, 1)
    device = torch.device('cuda:1' if torch.cuda.is_available() else 'cpu')
    actor_model = PtrNet(arglist)
    if os.path.exists(actor_model_path):
        actor_model.load_state_dict(torch.load(actor_model_path, map_location=device))
    else:
        print('specify pretrained model path')
    actor_model = actor_model.to(device)
    pred_tours, _ = actor_model(test_inputs, device)
    if arglist.use_rrt == True:
       # l_batch = env.stack_rrt_length(test_inputs, pred_tours)
        l_batch = env.stack_length_fast(test_inputs, pred_tours)  # actor实际输出的length
    else:
        # l_batch = env.stack_length(inputs,pred_tour)  # actor实际输出的length
        l_batch = env.stack_length_fast(test_inputs, pred_tours)  # actor实际输出的length
    index_lmin = torch.argmin(l_batch)
    best_tour = pred_tours[index_lmin]
    t2 = time()
    print('%dmin %1.2fsec\n' % ((t2 - t1) // 60, (t2 - t1) % 60))
    # env.show(test_input, best_tour)
    # env.show_rrt_html(test_input,best_tour)
    all_path_list = env.show_rrt_star_bid_h_html(test_input,best_tour,False)
    all_path_length = 0
    for i in range(len(all_path_list) - 1):
        all_path_length += between_nodes_distance(all_path_list[i], all_path_list[i+1])
    # print(all_path_list)
    print('point num:{} real_distance:{:.3f}'.format(len(all_path_list),all_path_length))  # 实际距离 即rrt距离
    return all_path_list,all_path_length


if __name__ == '__main__':
    arglist = argparser()
    torch.set_num_threads(3)
    #    read file
    subTask_msg = []
    subTask_filename = '../data/subTask.txt'   # linux下路径
    with open(subTask_filename,'r') as f:
        lines = f.readlines()
        for line in lines:
            tokens = line.split(' ')
            subTask_msg.append(tokens)
            # print(line)
    del subTask_msg[0]
    subTask_list = []
    for sub_task in subTask_msg:
        sub_task[-1] = sub_task[-1].replace("\n","")
        # print(sub_task)
        subTask_list.append(SubTask(int(sub_task[0]),
                                    sub_task[1],
                                    sub_task[2],
                                    int(sub_task[3]),
                                    [float(i) for i in sub_task[4].replace("(","").replace(")","").split(',')],
                                    [float(i) for i in sub_task[5].replace("(","").replace(")","").split(',')],
                                    [float(i) for i in sub_task[6].replace("(","").replace(")","").split(',')],
                                    [float(i) for i in sub_task[7].replace("(","").replace(")","").split(',')],
                                    [[float(i) for i in point.replace("(","").replace(")","").split(',')] for point in sub_task[8:]]
                                    )
                            )
    # print(subTask_list[-1].point_list)

    for sub_task in subTask_list:
        path,length = cal_path(arglist,sub_task)


    
    # 将path 和length 发送给task generation


