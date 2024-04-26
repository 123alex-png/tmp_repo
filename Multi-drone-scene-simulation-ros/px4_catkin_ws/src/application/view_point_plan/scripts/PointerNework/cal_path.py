#!/home/cs504/anaconda3/envs/torch/bin/python
#coding=utf-8

"""
@File: cal_path.py
@Author: lizeshan zhangruiheng
@Date: 2024-04-25
@Description: 对任务进行路径规划
"""

import rospy
import numpy as n
from Env import *
from SubTask import SubTask
from argument import argparser
import torch
import torch.nn as nn
from time import time
from actor import PtrNet
import os
from getViewPoint.getView import *
from uav_msgs.msg import Task_Path, Last_Task, TASK_State
from uav_msgs.srv import View_Plan, View_PlanResponse
from geometry_msgs.msg import Point32
from std_msgs.msg import String
from ObsEnv import ObsEnv
from scipy.spatial import ConvexHull


def cal_path(arglist, target_point_list, cur_ObsEnv, Obstacles, task_msg):

    arglist.target_point_num = len(target_point_list)
    current_path = os.path.abspath(__file__)
    current_dir = os.path.dirname(current_path)
    parent_path = os.path.dirname(current_dir)
    actor_model_path = parent_path + "/models/" + "model" + "_" + "0" + ".pt"
    print(actor_model_path)
    if os.path.exists(actor_model_path):
        print(f"加载模型{cur_ObsEnv.name}_{cur_ObsEnv.id}...")
    else:
        print(f"缺少符合任务的序贯决策模型{cur_ObsEnv.name}_{cur_ObsEnv.id}")
        return None, None

    # ========== 读取subTask ===============
    # 1.读取target_start target_end
    EnterPoint = [
        task_msg.EnterPoint.way_point_pos.x,
        task_msg.EnterPoint.way_point_pos.y,
        task_msg.EnterPoint.way_point_pos.z
    ]
    LeavePoint = [
        task_msg.LeavePoint.way_point_pos.x,
        task_msg.LeavePoint.way_point_pos.y,
        task_msg.LeavePoint.way_point_pos.z
    ]
    target_start = EnterPoint
    target_end = LeavePoint

    # 2. 载入环境  桥梁、边坡和路面的障碍物
    env = Env(arglist, task_msg.task_id, target_start, target_end,
              cur_ObsEnv.x_range, cur_ObsEnv.y_range, cur_ObsEnv.z_range,
              Obstacles, cur_ObsEnv.Obstacles, EnterPoint, LeavePoint,
              target_point_list)

    print('generate tour...')
    data = torch.tensor(target_point_list, device=arglist.device)
    data = data.type(torch.float32)
    test_input = data
    # ======================load_model==================
    # simplest way
    print('sampling ...')

    test_inputs = test_input.repeat(arglist.batch_size, 1, 1)
    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    actor_model = PtrNet(arglist)
    if os.path.exists(actor_model_path):
        actor_model.load_state_dict(
            torch.load(actor_model_path, map_location=device))
    else:
        print('specify pretrained model path')
    t1 = time()
    actor_model = actor_model.to(device)
    pred_tours, _ = actor_model(test_inputs, device)

    l_batch = env.stack_length_fast(test_inputs, pred_tours)  #  linear
    #l_batch = env.stack_rrt_length(test_inputs, pred_tours)  #  rrt

    index_lmin = torch.argmin(l_batch)
    best_tour = pred_tours[index_lmin]
    t2 = time()
    print('%dmin %1.4fsec\n' % ((t2 - t1) // 60, (t2 - t1) % 60))
    all_path_list = env.show_rrt_star_bid_h_html(test_input, best_tour, True)

    all_path_length = 0
    for i in range(len(all_path_list) - 1):
        all_path_length += between_nodes_distance(all_path_list[i],
                                                  all_path_list[i + 1])

    print('point num:{} real_distance:{:.3f}'.format(
        len(all_path_list), all_path_length))  # 实际距离 即rrt距离
    return all_path_list, all_path_length


def read_Obstacles(Obstacles_filename):
    """
    读取Obstacles文件,
    """
    Obstacles_msg = []
    with open(Obstacles_filename, 'r') as f:
        lines = f.readlines()
        for line in lines:
            tokens = line.split(' ')
            Obstacles_msg.append(tokens)
    del Obstacles_msg[0]
    return Obstacles_msg


def search_Obstacles(all_Obstacles, facilities_name, facilities_id):
    """
    筛选出与task_msg中的task_id相同的子任务范围内障碍物列表
    """
    for obs in all_Obstacles:
        obs[-1] = obs[-1].replace("\n", "")
        if obs[0] == facilities_name and int(obs[1]) == facilities_id:
            obs_list = obs[5:]
            min_obstacle = []
            for o in obs_list:
                point_list = [
                    float(i)
                    for i in o.replace("(", "").replace(")", "").split(',')
                ]
                points = []
                for i in range(0, len(point_list), 3):
                    points.append(
                        (point_list[i], point_list[i + 1], point_list[i + 2]))
                min_obstacle.append(points)

            cur_ObsEnv = ObsEnv(
                obs[0],  # name
                int(obs[1]),  # id
                eval(obs[2]),  # x_range
                eval(obs[3]),  # y_range
                eval(obs[4]),  # z_range
                min_obstacle  # min_Obstacles  
            )
            # print(f"cur_ObsEnv{cur_ObsEnv.name},{cur_ObsEnv.id},{cur_ObsEnv.Obstacles}")
            return cur_ObsEnv
    return None


def read_SubTask(subTask_filename, task_msg):
    """
    读取subTask文件,筛选出与task_msg中的task_id相同的子任务列表
    """
    subTask_msg = []
    with open(subTask_filename, 'r') as f:
        lines = f.readlines()
        for line in lines:
            tokens = line.split(' ')
            subTask_msg.append(tokens)

    del subTask_msg[0]
    subTask_list = []

    for sub_task in subTask_msg:
        if int(sub_task[2]) == task_msg.task_id:
            facilities_name = sub_task[3]
            facilities_id = int(sub_task[4])
            sub_task[-1] = sub_task[-1].replace("\n", "")
            # print(sub_task)
            subTask_list.append(
                SubTask(
                    int(sub_task[0]),  # id
                    sub_task[1],  # type
                    int(sub_task[2]),  # parent task id
                    sub_task[3],  # facilities name
                    int(sub_task[4]),  # facilities_id 
                    [
                        float(i) for i in sub_task[5].replace("(", "").replace(
                            ")", "").split(',')
                    ],
                    [
                        float(i) for i in sub_task[6].replace("(", "").replace(
                            ")", "").split(',')
                    ],
                    [
                        float(i) for i in sub_task[7].replace("(", "").replace(
                            ")", "").split(',')
                    ],
                    [
                        float(i) for i in sub_task[8].replace("(", "").replace(
                            ")", "").split(',')
                    ],
                    [[
                        float(i) for i in point.replace("(", "").replace(
                            ")", "").split(',')
                    ] for point in sub_task[9:]]))

    return subTask_list, facilities_name, facilities_id


def getObsEnv(ObsEnv_list, sub_task):
    for ObsEnv in ObsEnv_list:
        if sub_task.target_name == ObsEnv.name and sub_task.target_id == ObsEnv.id:
            return ObsEnv
        else:
            continue
    return None


def calculateAABB(points):
    # 计算边界
    xmin = np.min(points[:, 0])
    xmax = np.max(points[:, 0])
    ymin = np.min(points[:, 1])
    ymax = np.max(points[:, 1])
    zmin = np.min(points[:, 2])
    zmax = np.max(points[:, 2])

    return (xmin, ymin, zmin, xmax, ymax, zmax)


def is_inside_hull(hull, point):
    new_hull = ConvexHull(np.concatenate([hull.points, [point]]))
    return np.array_equal(new_hull.vertices, hull.vertices)


def gen_Obstacles(min_Obstacles, aabb_Obstacles):
    Obstacles = []
    for i, aabb_Obstacle in enumerate(aabb_Obstacles):
        for x in range(int(aabb_Obstacle[0]), int(aabb_Obstacle[3]), 2):
            for y in range(int(aabb_Obstacle[1]), int(aabb_Obstacle[4]), 2):
                for z in range(int(aabb_Obstacle[2]), int(aabb_Obstacle[5]),
                               2):
                    p = (x, y, z)
                    vertices = np.array(min_Obstacles[i])
                    hull = ConvexHull(vertices)
                    if is_inside_hull(hull, p):
                        Obstacles.append((p[0] - 1, p[1] - 1, p[2] - 1,
                                          p[0] + 1, p[1] + 1, p[2] + 1))

    return Obstacles


def cal_target_point_list(photo_dis, safe_dis, obsEnv, SubTaskList):
    photo_dis = photo_dis
    safe_dis = safe_dis
    SubTaskList = SubTaskList
    # --------------------------根据任务集合中的每个任务提取目标点-------------------------------------------------------------
    for sub_task in SubTaskList:
        print("当前任务类型是:{}".format(sub_task.task_type))
        if sub_task.task_type == 'Deck':
            sub_task.target_point_list = Deck(sub_task.point_list[0],
                                              sub_task.point_list[1],
                                              sub_task.point_list[2],
                                              sub_task.point_list[3],
                                              photo_dis, safe_dis)
        elif sub_task.task_type == 'Bearing':
            sub_task.target_point_list = Bearing(sub_task.point_list[0],
                                                 photo_dis, 2, 2)
        elif sub_task.task_type == 'Pier':
            sub_task.target_point_list = Pier(
                sub_task.point_list[0], sub_task.point_list[1],
                sub_task.point_list[2], sub_task.point_list[3],
                sub_task.point_list[4], sub_task.point_list[5],
                sub_task.point_list[6], sub_task.point_list[7], safe_dis,
                safe_dis, photo_dis, photo_dis)
        elif sub_task.task_type == 'Joint':
            sub_task.target_point_list = Joint(sub_task.point_list[0],
                                               photo_dis, 2, 2)
        elif sub_task.task_type == 'Abutment':
            sub_task.target_point_list = Abutment(sub_task.point_list[0],
                                                  photo_dis, 2, 2)
        elif sub_task.task_type == 'SlopeBottom':
            sub_task.target_point_list = SlopeBottom(sub_task.point_list[0],
                                                     sub_task.point_list[1],
                                                     sub_task.point_list[2],
                                                     sub_task.point_list[3],
                                                     photo_dis, safe_dis)
        elif sub_task.task_type == 'SlopeDitch':
            sub_task.target_point_list = SlopeDitch(sub_task.point_list[0],
                                                    sub_task.point_list[1],
                                                    sub_task.point_list[2],
                                                    sub_task.point_list[3],
                                                    photo_dis, safe_dis)
        elif sub_task.task_type == 'Barrier':
            sub_task.target_point_list = Barrier(sub_task.point_list[0],
                                                 sub_task.point_list[1],
                                                 photo_dis, safe_dis)
        elif sub_task.task_type == 'Guardarail':
            sub_task.target_point_list = Guardrail(sub_task.point_list[0],
                                                   sub_task.point_list[1],
                                                   photo_dis, safe_dis)
        elif sub_task.task_type == 'RoadSurface':
            sub_task.target_point_list = RoadSurface(sub_task.point_list[0],
                                                     sub_task.point_list[1],
                                                     sub_task.point_list[2],
                                                     sub_task.point_list[3],
                                                     18, 3)
        elif sub_task.task_type == 'Slope':
            sub_task.target_point_list = SlopeSurface(sub_task.point_list[0],
                                                      sub_task.point_list[1],
                                                      sub_task.point_list[2],
                                                      sub_task.point_list[3],
                                                      7, 7, safe_dis)

    target_point_list = []
    for sub_task in SubTaskList:
        for point in sub_task.target_point_list:
            target_point_list.append(point)
            # 需要判断目标点是否在环境范围
    return target_point_list


def view_point_plan_handle(requset):

    task_msg = requset.task

    arglist = argparser()

    #-------------------------------------read file-------------------------------------
    subTask_filename = rospy.get_param("subTask_filename",
                                       default='data/hill/subTask.txt')
    rospy.loginfo('subTask_filename is %s', subTask_filename)
    obstacles_filename = rospy.get_param("obstacles_filename",
                                         default='/data/hill/Obstacles.txt')
    rospy.loginfo('obstacles_filename is %s', obstacles_filename)
    subTask_list, facilities_name, facilities_id = read_SubTask(
        subTask_filename, task_msg)
    all_Obstacles = read_Obstacles(obstacles_filename)  # 保存的文件读取结果 是最小障碍物

    cur_ObsEnv = search_Obstacles(all_Obstacles, facilities_name,
                                  facilities_id)

    if cur_ObsEnv == None:
        print(f"任务{task_msg.task_id}没有匹配的障碍物模型,无法规划")
        task_msg.path_length = 0
        task_msg.task_path.append(task_msg.EnterPoint.way_point_pos)
        task_msg.task_path.append(task_msg.LeavePoint.way_point_pos)
        rospy.loginfo("task_msg_ path is generated,exit.....")
        return View_PlanResponse(task_msg.path_length, task_msg.task_path)

    # 根据subTask_list 计算目标点集合
    photo_dis = 5  # 拍摄距离
    safe_dis = 5  # 安全距离
    for o in cur_ObsEnv.Obstacles:
        cur_ObsEnv.aabb_Obstacles.append(calculateAABB(np.array(o)))

    Obstacles = gen_Obstacles(cur_ObsEnv.Obstacles,
                              cur_ObsEnv.aabb_Obstacles)  # 转换后的栅格格式  用于rrt求解
    print(len(Obstacles))
    # 任务下所有子任务提取出的目标点列表
    target_point_list = cal_target_point_list(photo_dis, safe_dis, cur_ObsEnv,
                                              subTask_list)
    print(target_point_list)
    path, length = cal_path(arglist, target_point_list, cur_ObsEnv, Obstacles,
                            task_msg)  # cur_ObsEnv[0]和[1]用于选择.pt文件

    if path == None and length == None:
        task_msg.path_length = 0
        task_msg.task_path.append(task_msg.EnterPoint.way_point_pos)
        task_msg.task_path.append(task_msg.LeavePoint.way_point_pos)
        rospy.loginfo("task_msg_ path is generated,exit.....")
        return View_PlanResponse(task_msg.path_length, task_msg.task_path)
    for p in path:
        task_msg.task_path.append(Point32(p[0], p[1], p[2]))
        task_msg.path_length = length

    rospy.loginfo("task_msg_ path is generated,exit.....")
    return View_PlanResponse(task_msg.path_length, task_msg.task_path)


if __name__ == '__main__':

    # ros
    uav_id = rospy.get_param('uav_id')
    rospy.init_node(str(rospy.get_name),
                    anonymous=True)  # /uav0/view_point_plan
    task_path_pub = rospy.Publisher('/task_path', Task_Path, queue_size=10)
    last_task_path_pub = rospy.Publisher('/last_cal_path',
                                         Last_Task,
                                         queue_size=10)
    rospy.Service('/uav' + str(uav_id) + '/view_point_plan_service', View_Plan,
                  view_point_plan_handle)  # /uav0/bid_price
    rospy.spin()
