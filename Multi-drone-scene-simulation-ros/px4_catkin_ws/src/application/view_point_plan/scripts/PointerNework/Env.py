import random

import numpy as np
import torch
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

current_path = os.path.abspath(__file__)
current_dir = os.path.dirname(current_path)
parent_path = os.path.dirname(current_dir)
sys.path.insert(0, parent_path)

from RRT.collsion_check import collision_check
from RRT.rrt_3dmain import *
from RRT.geometry import *
from RRT_star_bid_h.rrt_star_bid_h_3d_main import *


def between_nodes_distance(node_a, node_b):
    a_x, a_y, a_z = node_a[0], node_a[1], node_a[2]
    b_x, b_y, b_z = node_b[0], node_b[1], node_b[2]
    if isinstance(node_a, torch.Tensor):
        return torch.sqrt((b_x - a_x).pow(2) + (b_y - a_y).pow(2) +
                          (b_z - a_z).pow(2))
    elif isinstance(node_a, (list, np.ndarray)):
        return math.sqrt(
            pow(b_x - a_x, 2) + pow(b_y - a_y, 2) + pow(b_z - a_z, 2))
    else:
        raise TypeError


def rrt_between_nodes_distance(X_dimensions, Obstacles, node_a, node_b):
    between_2nodes_length = 0
    np_node_a = node_a.cpu().numpy()
    np_node_b = node_b.cpu().numpy()
    _, path = rrt_main(X_dimensions,
                       (np_node_a[0], np_node_a[1], np_node_a[2]),
                       (np_node_b[0], np_node_b[1], np_node_b[2]), Obstacles)
    if path is not None:
        tmp_node = np_node_a
        for i in path:
            between_2nodes_length += dist_between_points(
                tmp_node, np.array([i[0], i[1], i[2]]))
            tmp_node = np.array([i[0], i[1], i[2]])
    else:
        between_2nodes_length = 1e6

    return between_2nodes_length


class Env():
    """
    该类用于模型训练与计算rrt路径
    """

    def __init__(self, arglist, id, target_start, target_end, x_range, y_range,
                 z_range, Obstacles, min_Obstacles, EnterPoint, LeavePoint,
                 target_point_list):
        self.batch_size = arglist.batch_size
        self.target_point_num = arglist.target_point_num
        self.dim = 3  # 随机生成三维坐标
        self.x_range = x_range  #(0,90)
        self.y_range = y_range  #(0,60)
        self.z_range = z_range  # (0,50)
        self.X_dimensions = np.array(
            [self.x_range, self.y_range,
             self.z_range])  # dimensions of Search Space
        self.Obstacles = np.array(Obstacles)
        self.min_Obstacles = np.array(min_Obstacles)
        self.target_start = target_start
        self.target_end = target_end
        self.EnterPoint = EnterPoint
        self.LeavePoint = LeavePoint
        self.currentId = id
        self.target_point_list = target_point_list

    def get_tour_distance(self, nodes, tour):
        length = 0
        # 直线距离
        length += between_nodes_distance(torch.tensor(self.target_start),
                                         nodes[tour[0]])
        for i in range(len(nodes) - 1):
            length += between_nodes_distance(nodes[tour[i]],
                                             nodes[tour[(i + 1)]])
        length += between_nodes_distance(torch.tensor(self.target_end),
                                         nodes[tour[-1]])
        return length

    def get_rrt_tour_distance(self, nodes, tour):
        length = 0
        device = torch.device('cuda:1' if torch.cuda.is_available() else 'cpu')
        length += between_nodes_distance(torch.tensor(self.target_start),
                                         nodes[tour[0]])

        for i in range(len(nodes) - 1):
            # ====先用直线距离测试==========
            length += between_nodes_distance(nodes[tour[i]],
                                             nodes[tour[(i + 1)]])
            # length +=rrt_between_nodes_distance(self.X_dimensions,self.Obstacles,nodes[tour[i]],nodes[tour[(i+1)%self.view_point_num]])
        length += between_nodes_distance(torch.tensor(self.target_end),
                                         nodes[tour[-1]])
        length = np.float(length)
        length = torch.tensor(length, device=device)
        return length

    def stack_length(self, inputs, tours):
        list = []
        for i in range(self.batch_size):
            list.append(self.get_tour_distance(inputs[i], tours[i]))
        l_batch = torch.stack(list, dim=0)
        return l_batch

    def stack_length_fast(self, inputs, tours):
        """
        *** this function is faster version of stack_l! ***
        inputs: (batch, city_t, 2), Coordinates of nodes
        tours: (batch, city_t), predicted tour
        d: (batch, city_t, 2)
        """
        device = torch.device('cuda:1' if torch.cuda.is_available() else 'cpu')
        d = torch.gather(input=inputs,
                         dim=1,
                         index=tours[:, :, None].repeat(1, 1, 3))  #(512,20,3)
        Enter = torch.tensor(self.EnterPoint).repeat(self.batch_size,
                                                     1).to(device)
        Leave = torch.tensor(self.LeavePoint).repeat(self.batch_size,
                                                     1).to(device)
        start = torch.tensor(self.target_start).repeat(self.batch_size,
                                                       1).to(device)
        end = torch.tensor(self.target_end).repeat(self.batch_size,
                                                   1).to(device)
        dis1 = torch.sum((d[:, 1:] - d[:, :-1]).norm(p=2, dim=2), dim=1)
        dis2 = (Enter - start).norm(p=2, dim=1) + (start - d[:, 0]).norm(
            p=2, dim=1)  # enter to tar_start to first select node
        dis3 = (end - d[:, -1]).norm(p=2, dim=1) + (Leave - end).norm(
            p=2, dim=1)  # last select node to tar_end to Leave
        #dis2 = (d[:, 0] - d[:, -1]).norm(p=2,dim=1)  # distance from last node to first selected node
        return dis1 + dis2 + dis3

    def stack_rrt_length(self, inputs, tours):
        list = []
        for i in range(self.batch_size):
            list.append(self.get_rrt_tour_distance(inputs[i], tours[i]))
        l_batch = torch.stack(list, dim=0)
        return l_batch

    def cal_path(self, X, node_s, node_e, all_path_list, show, plot):
        _, path = rrt_star_bid_h_main(self.X_dimensions,
                                      (node_s[0], node_s[1], node_s[2]),
                                      (node_e[0], node_e[1], node_e[2]),
                                      self.Obstacles)
        while path == None:
            print("rrt path is None")
            _, path = rrt_star_bid_h_main(self.X_dimensions,
                                          (node_s[0], node_s[1], node_s[2]),
                                          (node_e[0], node_e[1], node_e[2]),
                                          self.Obstacles)
        collision = False
        while True:
            if collision == True:
                _, path = rrt_star_bid_h_main(
                    self.X_dimensions, (node_s[0], node_s[1], node_s[2]),
                    (node_e[0], node_e[1], node_e[2]), self.Obstacles)
                while path == None:
                    print("rrt path is None")
                    _, path = rrt_star_bid_h_main(
                        self.X_dimensions, (node_s[0], node_s[1], node_s[2]),
                        (node_e[0], node_e[1], node_e[2]), self.Obstacles)
            collision = False
            path = np.array(path)
            steps = len(path)
            cc = collision_check(self.Obstacles)
            for i in range(1, steps):
                segment = [
                    path[i - 1][0], path[i - 1][1], path[i - 1][2], path[i][0],
                    path[i][1], path[i][2]
                ]
                if cc.check(segment):
                    collision = True
                    break
            if collision is False:
                for p in path:
                    all_path_list.append(p)
                #print(path)
                break
        # plot
        if show is True:
            plot.plot_start(X, (node_s[0], node_s[1], node_s[2]))
            plot.plot_goal(X, (node_e[0], node_e[1], node_e[2]))
            plot.plot_path(X, path)
        return all_path_list

    def show_rrt_star_bid_h_html(self, nodes, tour, show=True):
        nodes = nodes.cpu().detach().numpy()  # 有序
        print('point num:{} line_distance:{:.3f}'.format(
            len(nodes), self.get_tour_distance(nodes, tour)))  # 直线距离
        tour_length = 0
        all_path_list = []
        print(tour)  # [18,6,3,49,21,6] 无序

        current_path = os.path.abspath(__file__)
        current_dir = os.path.dirname(current_path)
        parent_path = os.path.dirname(current_dir)
        plot = Plot(parent_path + "/html/Task_" + str(self.currentId) + ".html")
        tour = tour[:].cpu().detach().numpy()
        X = SearchSpace(self.X_dimensions, self.Obstacles)

        # 计算Enter 到 start
        all_path_list = self.cal_path(X, self.EnterPoint, self.target_start,
                                      all_path_list, show, plot)
        # 计算从start到第一个节点
        all_path_list = self.cal_path(X, self.target_start, nodes[tour[0]],
                                      all_path_list, show, plot)
        # 计算输出的路径
        for i in range(len(tour) - 1):
            all_path_list = self.cal_path(X, nodes[tour[i]],
                                          nodes[tour[i + 1]], all_path_list,
                                          show, plot)
        # 计算到end
        all_path_list = self.cal_path(X, nodes[tour[-1]], self.target_end,
                                      all_path_list, show, plot)
        # 计算end到Leave
        all_path_list = self.cal_path(X, self.target_end, self.LeavePoint,
                                      all_path_list, show, plot)

        # plot.plot_obstacles(X, self.Obstacles)
        plot.plot_min_obstacles(X, self.min_Obstacles)
        plot.draw(auto_open=False)
        print("success generate path ")
        return all_path_list
