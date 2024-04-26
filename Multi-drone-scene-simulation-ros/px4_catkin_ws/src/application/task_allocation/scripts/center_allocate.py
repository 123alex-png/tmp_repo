#!/usr/bin/python3
#coding=utf-8

"""
@File: center_allocate.py
@Author: lizeshan zhangruiheng
@Date: 2024-04-25
@Description: 任务分配的中心拍卖节点
"""

import math
import random
from threading import Thread
import threading
import numpy as np
from std_msgs.msg import Bool, Int8
from uav_msgs.msg import UAV_State, Task_List, TASK_State, UAV_Tasks, Bid_Task, Bid_Result, Last_Task
from uav_msgs.srv import Bid_Info
import matplotlib.pyplot as plt
import rospy


def task_list_handle(task_list_msg):
    rospy.loginfo("sub task list from task management")
    for task in task_list_msg.task_list:
        task_list.append(task)


def wait_bid_price(uav_num, task, EnvTemp, EnvRain, EnvWind):

    price_list = []
    S_t_list = []
    E_t_list = []
    for i in range(uav_num):
        price_list.append(-float('inf'))
        S_t_list.append(-1)
        E_t_list.append(-1)
    # 获取uav 的出价和index
    index_list = [-1 for i in range(uav_num)]
    for i in range(uav_num):
        rospy.wait_for_service('/uav' + str(i) + '/bid_price')
        try:
            bid_info = rospy.ServiceProxy('/uav' + str(i) + '/bid_price',
                                          Bid_Info)

            resp = bid_info(task, EnvTemp, EnvRain, EnvWind)
            price_list[
                resp.uav_id] = resp.bid_price  # 此处将price_list索引认为是无人机id，下同
            index_list[resp.uav_id] = resp.insert_index
            S_t_list[resp.uav_id] = resp.S_t
            E_t_list[resp.uav_id] = resp.E_t

        except rospy.ServiceException as e:
            print("Service call failed:%s" % e)
    return price_list, index_list, S_t_list, E_t_list


def cal_uav_num(uav_id):
    # rospy.loginfo("accept uav_id:{}".format(uav_id))
    if uav_id not in UAVs:
        rospy.loginfo("add uav:{}".format(uav_id))
        UAVs.append(uav_id)


def listen():
    rospy.spin()


if __name__ == '__main__':
    EnvTemp = 25
    EnvRain = 5
    EnvWind = 10
    task_list = []
    UAVs = []

    rospy.init_node('allocation', anonymous=True)
    rospy.Subscriber('/bid_to_center', Int8, cal_uav_num)
    ##rospy.Subscriber('/all_task_list',Task_List,task_list_handle)             ## 接收任务信息
    rospy.Subscriber('/all_task_allocate', Task_List,
                     task_list_handle)  ## 接收待任务信息
    bid_result_pub = rospy.Publisher('/bid_result', Bid_Result,
                                     queue_size=1)  # 给投标价格最高的无人机发送
    ## uav_msg = rospy.wait_for_message('/receiver', receiver_data, timeout=None)
    is_last_task_pub = rospy.Publisher('/last_task', Last_Task, queue_size=1)
    allocated_task_state_pub = rospy.Publisher('/allocated_task_state',
                                               TASK_State,
                                               queue_size=1)
    t2 = threading.Thread(target=listen)
    t2.start()
    all_price = 0
    while not rospy.is_shutdown():
        uav_num = len(UAVs)  # 改为订阅无人机节点计算真实个数
        if len(task_list) != 0 and len(UAVs) >= 2:
            for task in task_list:
                if task.done == 0:  # 存在任务未分配    0表示未分配 1表示已分配  2表示无无人机匹配 3表示正在执行 4表示执行完毕
                    rospy.loginfo(
                        "task {} is being allocated: uav_num:{}".format(
                            task.task_id, len(UAVs)))
                    # task_msg = task_to_msg(task)
                    #bid_task_pub.publish(task)

                    price_list, index_list, St_list, Et_list = wait_bid_price(
                        uav_num, task, EnvTemp, EnvRain,
                        EnvWind)  # 等待每个无人机出价  出价包括插入的位置索引和出价格
                    # 获取最大价格
                    max_price = max(price_list)
                    # 获取最大价格对应的无人机id
                    max_uav = price_list.index(max_price)
                    if index_list[max_uav] != -1:  # 任务满足约束 具有可插入位置 更新本地任务状态
                        task.done = 1
                        task.allocated = [max_uav]
                        task.S_t = St_list[max_uav]
                        task.E_t = Et_list[max_uav]
                        # 告知被选中的无人机
                        bid_result = Bid_Result()
                        bid_result.uav_id = max_uav  #此处视为 uav_id是从0开始的对应索引
                        bid_result.index = index_list[max_uav]
                        bid_result_pub.publish(bid_result)
                        rospy.loginfo(
                            "task {} is allocated to uav_num:{} price {}".
                            format(task.task_id, bid_result.uav_id, max_price))
                        allocated_task_state_pub.publish(
                            task)  # 将分配完成的任务结果发回task_management
                        if max_price > 0:
                            all_price = all_price + max_price
                    else:
                        rospy.loginfo(f"任务{task.task_id}没有满足约束的无人机")
                        task.allocated = [-1]
                        task.done = 2
                        allocated_task_state_pub.publish(
                            task)  # 将分配完成的任务结果发回task_management
                else:
                    continue
            is_task_msg = Last_Task()
            for task in task_list:
                if task.done == 0:
                    is_task_msg.is_last_task = False
                    break
                is_task_msg.is_last_task = True
            if is_task_msg.is_last_task is True:
                is_last_task_pub.publish(is_task_msg)
                print("all tasks is allocated!!!!!!")
                break
        else:
            continue
    rospy.loginfo("all price is {}".format(all_price))
