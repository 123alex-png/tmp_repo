#!/usr/bin/python3
#coding=utf-8
import rospy
from std_msgs.msg import Bool,Int8,UInt16
from uav_msgs.msg import UAV_State,Task_List,TASK_State,UAV_Tasks,Bid_Result,Last_Task
from uav_msgs.srv import Bid_Info,Bid_InfoResponse,View_Plan

import numpy as np
from copy import deepcopy
from threading import Thread
import threading

Max_Value=99

def cal_tf(uav, task):  # 计算无人机飞到任务所需的时间
    tf = 0
    distance = np.sum(abs(np.array(uav.pos) - np.array(task.enterpoint)))
    tf = distance / uav.fly_vel   # tf单位是s
    # tf = tf/60   # 转换为分钟
    return tf


def cal_ED_distance(a, b):   # 欧式距离
    return np.sqrt(np.sum(np.square( np.array(a) -np.array(b))))
def cal_MD_distance(a ,b):   # 曼哈顿距离
    return np.sum(abs(np.array(a)-np.array(b)))


def uav_state_handle(uav_state_msg):
    rospy.loginfo("sub uav state from uav control")
    print("uav:{}".format(uav_state_msg.uav_id))
    global uav_msg 
    uav_msg = uav_state_msg


def env_cost_cal(s_wind,WR_max,cur_OT,min_OT,max_OT,cur_WL,WL):
    """
    计算环境约束的任务成本   对任务插入位置无要求
    输入： 环境风速s_wind  无人机承受最大风速WR_max
          环境温度cur_OT  无人机工作温度范围min_OT,max_OT
          降雨量对应的防水等级需求cur_WL  无人机的防水等级WL
    :return:  环境约束成本Cost_e
    """
    if WR_max >= s_wind:
        Cost_wind = 0
    else:
        Cost_wind = Max_Value
    if cur_OT >= min_OT and cur_OT <= max_OT:
        Cost_temp = 0
    else:
        Cost_temp = Max_Value
    if WL >= cur_WL:
        Cost_rain = 0
    else:
        Cost_rain = Max_Value

    Cost_e = Cost_wind + Cost_temp + Cost_rain
    return Cost_e



def resource_cost_cal(Pg,EnterPoint,LeavePoint,SR_max,Seq,Pu,maxV,maxTF,TF):
    """
    计算无人机资源约束的任务成本
    输入：无人机连接地面站的位置Pg=（x,y,z） 任务进入点的位置EnterPoint=(x,y,z) 无人机的最大通信距离SR_max
         插入到k位置后的无人机的任务列表[Tj1,Tj2,...,Tjv] 无人机当前位置Pu、飞行速度maxV, 无人机的最长续航时间maxTF 已飞行时间TF
    :return:
    """
    # 计算通信距离成本  对任务插入位置无要求
    if cal_ED_distance(Pg,EnterPoint)<=SR_max and cal_ED_distance(Pg,LeavePoint)<=SR_max:
        Cost_signal = 0
    else:
        Cost_signal = Max_Value
    # 计算续航时间成本  对任务插入位置有要求
    Flength_line = cal_ED_distance(Pu,[Seq[0].EnterPoint.way_point_pos.x,Seq[0].EnterPoint.way_point_pos.y,Seq[0].EnterPoint.way_point_pos.z])
    for i in range(1,len(Seq)-1):
        Flength_line += cal_ED_distance([Seq[i].LeavePoint.way_point_pos.x,Seq[i].LeavePoint.way_point_pos.y,Seq[i].LeavePoint.way_point_pos.z],[Seq[i+1].EnterPoint.way_point_pos.x,Seq[i+1].EnterPoint.way_point_pos.y,Seq[i+1].EnterPoint.way_point_pos.z])
    FT_line = Flength_line / maxV
    FT_task = 0
    for i in range(0,len(Seq)-1):
        FT_task += Seq[i].execute_time
    ET = FT_line + FT_task

    if (maxTF-TF) > ET:
        Cost_power = 0
    else:
        Cost_power = Max_Value
    Cost_R = Cost_signal + Cost_power
    return Cost_R


def timewindow_cost_cal(E_tk, S_tk1, preT_k, sufTk1, Seq, insert_index):
    Seq[insert_index].Ts_l= Seq[insert_index].Te - Seq[insert_index].execute_time
    Seq[insert_index].Te_e =  Seq[insert_index].Ts + Seq[insert_index].execute_time
    # 四种情况
    # 1
    if E_tk <= Seq[insert_index].Ts and Seq[insert_index].Te <= S_tk1 and Seq[
        insert_index].execute_time < S_tk1 - E_tk:
        if Seq[insert_index].E_t + sufTk1 <= S_tk1 and Seq[insert_index].S_t >= Seq[insert_index].Ts and Seq[
            insert_index].S_t <= Seq[insert_index].Ts_l:
            Seq[insert_index].S_t = Seq[insert_index].S_t
        elif Seq[insert_index].Ts >= Seq[insert_index].S_t and Seq[insert_index].Te_e + sufTk1 <= S_tk1:
            Seq[insert_index].S_t = Seq[insert_index].Ts
        else:
            Seq[insert_index].S_t = Seq[insert_index].Ts - 1
        Seq[insert_index].E_t = Seq[insert_index].S_t + Seq[insert_index].execute_time
    elif E_tk <= Seq[insert_index].Ts and Seq[insert_index].Ts<=S_tk1 and Seq[insert_index].Te >= S_tk1 and Seq[
        insert_index].execute_time < S_tk1 - Seq[insert_index].Ts:
        if Seq[insert_index].Ts > Seq[insert_index].S_t and Seq[insert_index].Te_e + sufTk1 < S_tk1:
            Seq[insert_index].E_t = Seq[insert_index].Te_e
        elif Seq[insert_index].E_t + sufTk1 <= S_tk1 and Seq[insert_index].S_t <= Seq[insert_index].Ts_l:
            Seq[insert_index].E_t = Seq[insert_index].E_t
        else:
            Seq[insert_index].E_t = Seq[insert_index].Te_e + 1
        Seq[insert_index].S_t = Seq[insert_index].E_t - Seq[insert_index].execute_time
    elif E_tk >= Seq[insert_index].Ts and Seq[insert_index].Te <= S_tk1 and Seq[insert_index].Te >= E_tk and Seq[
        insert_index].execute_time < Seq[insert_index].Te - E_tk:
        if Seq[insert_index].Ts_l > Seq[insert_index].S_t and Seq[insert_index].E_t + sufTk1 < S_tk1:
            Seq[insert_index].S_t = Seq[insert_index].S_t
        else:
            Seq[insert_index].S_t = Seq[insert_index].Ts - 1
        Seq[insert_index].E_t = Seq[insert_index].S_t + Seq[insert_index].execute_time
    elif (E_tk >= Seq[insert_index].Ts or Seq[insert_index].Te >= S_tk1) and Seq[
        insert_index].execute_time >= min(Seq[insert_index].Te,S_tk1)-max(Seq[insert_index].Ts,E_tk):
        Seq[insert_index].S_t = Seq[insert_index].Ts-1
        Seq[insert_index].E_t = Seq[insert_index].Te+1
    return Seq[insert_index].S_t, Seq[insert_index].E_t


def task_cost_cal(TaskReqUav,TaskReqPayload,UavDomain,PayloadType,Seq,insert_index,maxV,pos,max_time):
    """
    任务约束成本计算
    输入：任务所需无人机类型TaskReqUav 任务所需载荷类型TaskReqPayload 无人机类型UavDomain 无人机的载荷类型PayloadType
         无人机插入任务后的执行列表Seq 插入位置insert_index  无人机的最大飞行速度maxV  无人机的当前位置pos 无人机的最大续航时间max_time = power
    :return:
    """
    # 任务所需资源成本计算 对任务插入位置有要求
    if TaskReqUav == UavDomain and TaskReqPayload == PayloadType:
        Cost_req = 0
    else:
        Cost_req = Max_Value


    # 任务时间窗成本计算
    if len(Seq) == 1:
        E_tk = 0
        S_tk1 = max_time
        preT_k = cal_MD_distance(pos, [Seq[insert_index].EnterPoint.way_point_pos.x,Seq[insert_index].EnterPoint.way_point_pos.y,Seq[insert_index].EnterPoint.way_point_pos.z]) / maxV
        sufTk1 = 0
        Seq[insert_index].S_t = E_tk + preT_k
        Seq[insert_index].E_t = Seq[insert_index].S_t + Seq[insert_index].execute_time
        # update
        Seq[insert_index].S_t, Seq[insert_index].E_t = timewindow_cost_cal(E_tk, S_tk1, preT_k, sufTk1, Seq,
                                                                           insert_index)
    else:
        if insert_index ==0:            # 插入第一个位置
            E_tk = 0
            S_tk1 = Seq[insert_index+1].S_t
            preT_k = cal_MD_distance(pos, [Seq[insert_index].EnterPoint.way_point_pos.x,Seq[insert_index].EnterPoint.way_point_pos.y,Seq[insert_index].EnterPoint.way_point_pos.z]) / maxV
            sufTk1 = cal_MD_distance([Seq[insert_index].LeavePoint.way_point_pos.x,Seq[insert_index].LeavePoint.way_point_pos.y,Seq[insert_index].LeavePoint.way_point_pos.z], [Seq[insert_index+1].EnterPoint.way_point_pos.x,Seq[insert_index+1].EnterPoint.way_point_pos.y,Seq[insert_index+1].EnterPoint.way_point_pos.z]) / maxV
            Seq[insert_index].S_t = E_tk + preT_k
            Seq[insert_index].E_t = Seq[insert_index].S_t + Seq[insert_index].execute_time
            # update
            Seq[insert_index].S_t, Seq[insert_index].E_t = timewindow_cost_cal(E_tk, S_tk1, preT_k, sufTk1, Seq,
                                                                               insert_index)
        elif insert_index == len(Seq)-1: # 插入最后位置
            E_tk = Seq[insert_index - 1].E_t
            S_tk1 = max_time
            preT_k = cal_MD_distance([Seq[insert_index-1].LeavePoint.way_point_pos.x,Seq[insert_index-1].LeavePoint.way_point_pos.y,Seq[insert_index-1].LeavePoint.way_point_pos.z], [Seq[insert_index].EnterPoint.way_point_pos.x,Seq[insert_index].EnterPoint.way_point_pos.y,Seq[insert_index].EnterPoint.way_point_pos.z]) / maxV
            sufTk1 = 0
            Seq[insert_index].S_t = E_tk + preT_k
            Seq[insert_index].E_t = Seq[insert_index].S_t + Seq[insert_index].execute_time
            # update
            Seq[insert_index].S_t, Seq[insert_index].E_t = timewindow_cost_cal(E_tk, S_tk1, preT_k, sufTk1, Seq,
                                                                               insert_index)
        else:                           # 插入中间位置
            E_tk = Seq[insert_index-1].E_t
            S_tk1 = Seq[insert_index+1].S_t
            preT_k = cal_MD_distance([Seq[insert_index-1].LeavePoint.way_point_pos.x,Seq[insert_index-1].LeavePoint.way_point_pos.y,Seq[insert_index-1].LeavePoint.way_point_pos.z],[Seq[insert_index].EnterPoint.way_point_pos.x,Seq[insert_index].EnterPoint.way_point_pos.y,Seq[insert_index].EnterPoint.way_point_pos.z])/maxV
            sufTk1 =  cal_MD_distance([Seq[insert_index].LeavePoint.way_point_pos.x,Seq[insert_index].LeavePoint.way_point_pos.y,Seq[insert_index].LeavePoint.way_point_pos.z],[Seq[insert_index+1].EnterPoint.way_point_pos.x,Seq[insert_index+1].EnterPoint.way_point_pos.y,Seq[insert_index+1].EnterPoint.way_point_pos.z])/maxV
            Seq[insert_index].S_t = E_tk + preT_k
            Seq[insert_index].E_t = Seq[insert_index].S_t + Seq[insert_index].execute_time
            # update
            Seq[insert_index].S_t,Seq[insert_index].E_t = timewindow_cost_cal(E_tk,S_tk1,preT_k,sufTk1,Seq,insert_index)

    if Seq[insert_index].S_t >= Seq[insert_index].Ts and Seq[insert_index].S_t<=  Seq[insert_index].Te and  Seq[insert_index].E_t >=  Seq[insert_index].Ts and Seq[insert_index].E_t<= Seq[insert_index].Te:
        Cost_time = 0
    else:
        Cost_time = Max_Value

    # return Cost_time, Seq[insert_index].S_t, Seq[insert_index].E_t

    Cost_T1 = Cost_req + Cost_time

    return Cost_T1,Seq[insert_index].S_t, Seq[insert_index].E_t



def dist_cost_cal(pos,Seq,insert_index):
    """
    计算任务飞行距离约束成本
    ：Input：无人机的位置pos  插入后的无人机任务序列Seq 插入位置insert_index


    :return:
    """
    if len(Seq) == 1 and insert_index==0:   ## 长度为0
        D_ori = 0
        D_new = cal_MD_distance(pos, [Seq[insert_index].EnterPoint.way_point_pos.x,Seq[insert_index].EnterPoint.way_point_pos.y,Seq[insert_index].EnterPoint.way_point_pos.z])  # 不计算最终回到起飞点的距离

    else:   # 插入前长度不为0
        if insert_index == 0:  # 插入第一个位置
            D_ori = cal_MD_distance(pos,[Seq[insert_index+1].EnterPoint.way_point_pos.x,Seq[insert_index+1].EnterPoint.way_point_pos.y,Seq[insert_index+1].EnterPoint.way_point_pos.z])
            D_new =  cal_MD_distance(pos,[Seq[insert_index].EnterPoint.way_point_pos.x,Seq[insert_index].EnterPoint.way_point_pos.y,Seq[insert_index].EnterPoint.way_point_pos.z])+cal_MD_distance([Seq[insert_index].LeavePoint.way_point_pos.x,Seq[insert_index].LeavePoint.way_point_pos.y,Seq[insert_index].LeavePoint.way_point_pos.z],[Seq[insert_index+1].EnterPoint.way_point_pos.x,Seq[insert_index+1].EnterPoint.way_point_pos.y,Seq[insert_index+1].EnterPoint.way_point_pos.z])
        elif insert_index == len(Seq)-1: # 插入最后一个位置
            D_ori = 0
            D_new = cal_MD_distance([Seq[insert_index-1].LeavePoint.way_point_pos.x,Seq[insert_index-1].LeavePoint.way_point_pos.y,Seq[insert_index-1].LeavePoint.way_point_pos.z],[Seq[insert_index].EnterPoint.way_point_pos.x,Seq[insert_index].EnterPoint.way_point_pos.y,Seq[insert_index].EnterPoint.way_point_pos.z])
        else:  # 插入中间位置
            D_ori = cal_MD_distance([Seq[insert_index-1].LeavePoint.way_point_pos.x,Seq[insert_index-1].LeavePoint.way_point_pos.y,Seq[insert_index-1].LeavePoint.way_point_pos.z],[Seq[insert_index+1].EnterPoint.way_point_pos.x,Seq[insert_index+1].EnterPoint.way_point_pos.y,Seq[insert_index+1].EnterPoint.way_point_pos.z])
            D_new = cal_MD_distance([Seq[insert_index-1].LeavePoint.way_point_pos.x,Seq[insert_index-1].LeavePoint.way_point_pos.y,Seq[insert_index-1].LeavePoint.way_point_pos.z],[Seq[insert_index].EnterPoint.way_point_pos.x,Seq[insert_index].EnterPoint.way_point_pos.y,Seq[insert_index].EnterPoint.way_point_pos.z])+ cal_MD_distance([Seq[insert_index].LeavePoint.way_point_pos.x,Seq[insert_index].LeavePoint.way_point_pos.y,Seq[insert_index].LeavePoint.way_point_pos.z],[Seq[insert_index+1].EnterPoint.way_point_pos.x,Seq[insert_index+1].EnterPoint.way_point_pos.y,Seq[insert_index+1].EnterPoint.way_point_pos.z])
    del_distance = D_new - D_ori
    Cost_dist = del_distance
    return Cost_dist

def bid_price_version2(uav,task,EnvWind,EnvTemp,EnvRain):

    Cost_E = env_cost_cal(EnvWind,uav.max_WR,EnvTemp,uav.work_temperature_min,uav.work_temperature_max,EnvRain,uav.wl) # env_cost_cal(s_wind,WR_max,cur_OT,min_OT,max_OT,cur_WL,WL):

    if Cost_E >= Max_Value:
        price = task.R-Cost_E
        print(f"对任务{task.task_id},无人机{uav.uav_id}不满足环境约束,price：{price}")
        return uav.uav_id, price, -1,task.S_t,task.E_t

    uav_accepted = accepted.copy()
    all_insert_Cost = []   # 记录每个插入位置的Cost值，在最后进行比较，选出最佳的插入位置
    all_insert_S_t = []
    all_insert_E_t = []

    for i in range(0,len(uav_accepted)+1):
        all_insert_Cost.append(0)
        all_insert_S_t.append(-1)
        all_insert_E_t.append(-1)
        temp_accepted = uav_accepted.copy()
        temp_accepted.insert(i, task)
        Cost_R = resource_cost_cal([uav.ground_station.x, uav.ground_station.y, uav.ground_station.z], [task.EnterPoint.way_point_pos.x,task.EnterPoint.way_point_pos.y,task.EnterPoint.way_point_pos.z],
                                   [task.LeavePoint.way_point_pos.x,task.LeavePoint.way_point_pos.y,task.LeavePoint.way_point_pos.z],
                                   uav.max_comm_length,temp_accepted,[uav.position.x,uav.position.y,uav.position.z],uav.velocity_size,uav.power,uav.flight_time)
        #print(f"对任务{task.id},无人机{uav.id}资源约束成本：{Cost_R}")
        if Cost_R >= Max_Value:
            all_insert_Cost[i] = Cost_R
            continue
        Cost_T1,all_insert_S_t[i],all_insert_E_t[i] = task_cost_cal(task.req_uav_type,task.req_payload_type,uav.uav_type,uav.payload_type,temp_accepted,i,uav.velocity_size,[uav.position.x,uav.position.y,uav.position.z],9999)
        #print(f"对任务{task.id},无人机{uav.id}任务所需无人机资源与时间窗约束成本：{Cost_T1}")
        if Cost_T1 >= Max_Value:
            all_insert_Cost[i] = Cost_T1
            continue
        Cost_T2 = dist_cost_cal([uav.position.x,uav.position.y,uav.position.z],temp_accepted,i)
        #print(f"对任务{task.id},无人机{uav.id}任务距离约束成本：{Cost_T2}")
        Cost_T = Cost_T1 + 0.01*Cost_T2

        Cost = Cost_R + Cost_T
        all_insert_Cost[i] = Cost
    min_Cost = min(all_insert_Cost)
    min_insert_index = all_insert_Cost.index(min(all_insert_Cost))
    task.S_t = int(all_insert_S_t[min_insert_index])
    task.E_t = int(all_insert_E_t[min_insert_index])
    price = task.R - min_Cost
    if min_Cost-10 >=Max_Value:
        print(f"对任务{task.task_id},无人机{uav.uav_id}不满足资源与任务约束,price：{price}")
        print(min_Cost)
        print(Max_Value)
        return uav.uav_id, price, -1,task.S_t,task.E_t
    else:
        print(f"对任务{task.task_id},无人机{uav.uav_id}插入在位置{min_insert_index}最佳，price：{price}")
        return uav.uav_id,price,min_insert_index,task.S_t,task.E_t
    
def bid_price(uav_msg,task_msg,env_temp):
    # """
    # 出价共有5类约束  分别是 天气约束、载荷能力约束、最大通信距离约束、时间窗约束、飞行距离约束

    # :param uav:
    # :param task:
    # :return:
    # """
    # min_power = 60
    # #================天气约束======================
    # if env_temp >= uav_msg.work_temperature_min and env_temp <= uav_msg.work_temperature_max:
    #     WC = 0
    # else:
    #     print(f"无人机{uav_msg.uav_id}不满足天气约束")
    #     return uav_msg.uav_id, float('-inf'), -1
    # # ==============载荷能力约束 ===================
    # if task_msg.req_uav_type == uav_msg.uav_type:
    #     PC = 0
    # else:
    #     print(f"无人机{uav_msg.uav_id}不满足载荷能力约束,tasktype:{task_msg.req_uav_type},uavtype:{uav_msg.uav_type}")
    #     return uav_msg.uav_id, float('-inf'), -1
    # # ===========最大图传距离约束 =================
    # # print("地面站与任务地点距离为:")
    # # print(cal_ED_distance([uav.start_x, uav.start_y, uav.start_z], task.pos))
    # if cal_ED_distance([uav_msg.uav_start_pos.x,uav_msg.uav_start_pos.y,uav_msg.uav_start_pos.z],task_msg.EnterPoint) <= uav_msg.max_comm_length:
    #     LC = 0
    # else:
    #     print(f"无人机{uav_msg.uav_id}不满足最大图传距离约束")
    #     return uav_msg.uav_id, float('-inf'), -1
    # #=============== 飞行距离约束====================
    # # uav_accepted = uav.accepted.deepcopy()
    # uav_accepted = deepcopy(uav_msg.accepted)
    # if len(uav_accepted) == 0:
        
    #     temp_accepted = deepcopy(uav_accepted)
    #     temp_accepted.insert(0, task_msg)
    
    #     #all_distance = cal_ED_distance(uav.pos,task.pos)+ cal_ED_distance(task.pos,[uav.start_x,uav.start_y,uav.start_z])
    #     all_distance = cal_ED_distance(uav_msg.position,task_msg.EnterPoint)  # 不计算最终回到起飞点的距离
    #     del_distance = all_distance
    #     #计算剩余电量
    #     battery = uav_msg.power - (all_distance/uav_msg.velocity_size + task_msg.execute_time)
    #     if battery <= min_power:
    #         print(f"无人机{uav_msg.uav_id}剩余电量不足")
    #         return uav_msg.uav_id,  float('-inf'), -1
    #     else:
    #         cost = WC+PC+LC-del_distance
    #         print(f"对任务{task_msg.task_id},无人机{uav_msg.uav_id}插入在位置0最佳，cost：{cost}")
    #         return uav_msg.uav_id, cost, 0
    # else:
    #     del_distance = []
    #     for i in range(len(uav_accepted)+1): # 4  i = 0 1 2 3  [3,2,4,5] 7
    #         temp_accepted = deepcopy(uav_accepted)
    #         temp_accepted.insert(i, task_msg)   # 插入后的列表
    #         if i == 0:
    #             ori_distance = cal_ED_distance(uav_msg.position, uav_accepted[i].EnterPoint)
    #             #print(f"原始飞行距离:{ori_distance}")
    #             new_distance = cal_ED_distance(uav_msg.position, temp_accepted[i].EnterPoint) + cal_ED_distance(temp_accepted[i].LeavePoint, temp_accepted[i+1].EnterPoint)
    #             #prinbid_result_pubt(f"插入新任务在位置{i}后的飞行距离:{new_distance}")
    #             all_exe_time = 0
    #             all_new_distance = cal_ED_distance(uav_msg.position,temp_accepted[0].EnterPoint)
    #             for j in range(len(temp_accepted)-1):
    #                 all_new_distance += cal_ED_distance(temp_accepted[j].LeavePoint,temp_accepted[j+1].EnterPoint)
    #                 all_exe_time += temp_accepted[j].execute_time
    #             battery = uav_msg.power - (all_new_distance/uav_msg.velocity_size + all_exe_time)
    #             if battery <= min_power:
    #                 print(f"无人机{uav_msg.uav_id}剩余电量不足")
    #                 new_distance = float('inf')
    #         elif i == len(uav_accepted):
    #             ori_distance = 0
    #             # ori_distance = cal_ED_distance(task_list[uav_accepted[i-1]].pos, [uav.start_x,uav.start_y,uav.start_z])
    #             #print(f"原始飞行距离:{ori_distance}")
    #             # new_distance = cal_ED_distance(task_list[temp_accepted[i-1]].pos, task_list[temp_accepted[i]].pos) + cal_ED_distance(
    #             #     task_list[temp_accepted[i]].pos, [uav.start_x,uav.start_y,uav.start_z])
    #             new_distance = cal_ED_distance(temp_accepted[i-1].LeavePoint, temp_accepted[i].EnterPoint)
    #             #print(f"插入新任务在位置{i}后的飞行距离:{new_distance}")
    #             all_exe_time = 0 + temp_accepted[-1].execute_time
    #             all_new_distance = cal_ED_distance(uav_msg.position, temp_accepted[0].EnterPoint)
    #             for j in range(len(temp_accepted) - 1):
    #                 all_new_distance += cal_ED_distance(temp_accepted[j].LeavePoint,
    #                                                     temp_accepted[j + 1].enterpoint)
    #                 all_exe_time += temp_accepted[j].execute_time
                
    #             battery = uav.power - (all_new_distance / uav.fly_vel + all_exe_time)
    #             if battery <= min_power:
    #                 print(f"无人机{uav.id}剩余电量不足")
    #                 new_distance = float('inf')
    #         else:
    #             ori_distance = cal_ED_distance(uav_accepted[i-1].leavepoint, uav_accepted[i].enterpoint)
    #             #print(f"原始飞行距离:{ori_distance}")
    #             new_distance = cal_ED_distance(temp_accepted[i-1].leavepoint,temp_accepted[i].enterpoint) +\
    #                        cal_ED_distance(temp_accepted[i].leavepoint, temp_accepted[i + 1].enterpoint)
    #             #print(f"插入新任务在位置{i}后的飞行距离:{new_distance}")
    #             all_exe_time = 0 + temp_accepted[-1].execute_time
    #             all_new_distance = cal_ED_distance(uav.pos, temp_accepted[0].enterpoint)
    #             for j in range(len(temp_accepted) - 1):
    #                 all_new_distance += cal_ED_distance(temp_accepted[j].leavepoint,
    #                                                     temp_accepted[j + 1].enterpoint)
    #                 all_exe_time += temp_accepted[j].execute_time
    #             battery = uav.power - (all_new_distance / uav.fly_vel + all_exe_time)
    #             if battery <= min_power:
    #                 print(f"无人机{uav.id}剩余电量不足")
    #                 new_distance = float('inf')
    #         del_distance.append(new_distance-ori_distance)
    #     min_del_distance = min(del_distance)
    #     min_index = del_distance.index(min(del_distance))
    #     cost = WC+PC+LC-min_del_distance
    # cost = WC+PC+LC
    # min_index = 0
    # print(f"对任务{task.id},无人机{uav.id}插入在位置{min_index}最佳，cost：{cost}")
    min_index=0
    cost = 0.05
    return uav_msg.uav_id, cost, min_index




def bid_price_handle(req):
    # 拍卖价格计算
    global task_msg 
    task_msg = req.task

    uav_id,price,index,task_msg.S_t,task_msg.E_t = bid_price_version2(uav_msg,req.task,req.EnvWind,req.EnvTemp,req.EnvRain)
    rospy.loginfo(f"uav:{uav_id},price:{price},index:{index}")
    return Bid_InfoResponse(uav_id,price,index,task_msg.S_t,task_msg.E_t) 


def bid_result_handle(bid_result_msg):
    """
    接收最高出价id
    """
    if bid_result_msg.uav_id ==uav_id: ## 出价最高的uavid等于当前uav id
        accepted.insert(bid_result_msg.index,task_msg)
        print(f"uav_{uav_id} get task:{task_msg.task_id}")


def last_task_handle(msg):
    if msg.is_last_task == True:
        rospy.loginfo(f"无人机{uav_id}的执行任务列表为{accepted},任务分配完成，等待视点规划命令..")
        
  

def start_view_point_plan_handle(start_msg):
    print("调用start_view_point_plan_handle")
    if start_msg.uav_id == uav_id:
        #发送无人机的任务给view_point_plan
        print(f"uav:{uav_id} start view point plan")
        for task in accepted:
            rospy.wait_for_service('/uav'+str(uav_id)+'/view_point_plan_service')
            try:
                cal_task_path = rospy.ServiceProxy('/uav'+str(uav_id)+'/view_point_plan_service',View_Plan)
                view_plan_response = cal_task_path(task)
                task.path_length = view_plan_response.path_length
                task.task_path = view_plan_response.task_path
                rospy.loginfo(f"任务{task.task_id}的视点路径长度为{task.task_path}")
            except rospy.ServiceException as e:
                print("Service call failed:%s"%e)
        
def start_execute_task_handle(execute_msg):
    #if send == False:
    all_task_view_plan = True
    if execute_msg.uav_id == uav_id:
        for task in accepted:
            if len(task.task_path)==0:  # 任务路径若有模型计算则有实际路径  否则有Enteroint和Leaveoint两点
                all_task_view_plan = False
                break
        if all_task_view_plan == True:
            uav_tasks = UAV_Tasks()
            uav_tasks.uav_id = uav_id
            uav_tasks.uav_tasks = accepted
            uav_tasks_pub.publish(uav_tasks)
            send = True
            print(f"uav:{uav_id} send task to uav_control")
    else:
	    return
   # else:
    #    return
        


accepted = []

# start_next_view_point_plan =False
def listen():
    rospy.spin()



if __name__ == '__main__':

    
 
    uav_id = rospy.get_param('uav_id')
    rospy.init_node(str(rospy.get_name),anonymous=True)  # /uav0/bid
    node_name = str(rospy.get_name)
    
    rospy.loginfo("node_name:%s" %node_name)
    rospy.loginfo("uav_id :%s" %str(uav_id))

    sub_uav_id ='/uav'+ str(uav_id)+'/initial_state'  # /uav0/initial_state
    print(sub_uav_id)
   
    rospy.Subscriber(sub_uav_id,UAV_State,uav_state_handle)
    ## 检查无人机的状态信息  若非故障  则进入函数等待allocation发出的任务信息 然后出价 并等待
    rospy.Subscriber('/last_task',Last_Task,last_task_handle)
    rospy.Service('/uav'+str(uav_id)+'/bid_price',Bid_Info,bid_price_handle) # /uav0/bid_price
    rospy.Subscriber('/bid_result',Bid_Result,bid_result_handle)
    rospy.Subscriber('/start_view_point_plan',UAV_State,start_view_point_plan_handle)
    rospy.Subscriber('/start_execute_task',UAV_State,start_execute_task_handle)
    # start_view_point_plan_pub = rospy.Publisher('/uav'+str(uav_id)+'/view_point_plan',Int8,queue_size=10) # 向viewpointplan发送任务的id
    uav_tasks_pub = rospy.Publisher('/uav'+str(uav_id)+'/uav_tasks',UAV_Tasks,queue_size=10)
    uav_bid_pub = rospy.Publisher('/bid_to_center',Int8,queue_size=10)
    
    t2 = threading.Thread(target=listen)
    t2.start()


    while not rospy.is_shutdown():
        uav_bid_pub.publish(uav_id)
    #     rospy.spin()


    







