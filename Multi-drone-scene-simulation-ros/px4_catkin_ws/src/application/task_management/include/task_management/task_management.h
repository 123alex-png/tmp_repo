/*******************************************************************
 * File: task_management.h
 * Author: lizeshan zhangruiheng
 * Date: 2024-04-23
 * Description: 管理任务信息，并发布给对应节点
 *******************************************************************/
#ifndef TASK_MANAGEMENT_H
#define TASK_MANAGEMENT_h

#include <sqlite3.h>
#include <time.h>

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "uav_msgs/Last_Task.h"
#include "uav_msgs/TASK_State.h"
#include "uav_msgs/Task_List.h"
#include "uav_msgs/Way_Point.h"

typedef struct WayPoint {
  int id;
  float x;
  float y;
  float z;
} WayPoint;

ros::Subscriber task_state_sub;
ros::Subscriber allocated_task_state_sub;

ros::Publisher all_task_list_pub;
ros::Publisher all_tasks_allocate_pub;
ros::Publisher all_tasks_qt_pub;

std::vector<uav_msgs::TASK_State> all_tasks;  // 存放所有收到的任务(唯一)

/**
 * @brief: 接受任务信息话题的回调函数
 * @param msg：任务信息
 */
void TaskState_callBack(
    const uav_msgs::TASK_State::ConstPtr& msg);  // 接收任务信息
/**
 * @brief: 任务分配状态的回调函数
 * @param msg：拍卖完的任务的信息
 */
void AllocatedTaskState_callback(const uav_msgs::TASK_State::ConstPtr& msg);
/**
 * @brief: 打印任务信息
 */
void ShowTaskInfo();  // 显示受到的任务
/**
 * @brief: 向Qt发布任务信息
 */
void AllTaskQt_Publish();        // 把所有的任务发送给qt
void AllTaskAllocate_Publish();  // 把任务发送给allocate
void readWayPoints();

#endif
