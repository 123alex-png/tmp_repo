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
#include "uav_msgs/Task_Path.h"
#include "uav_msgs/Way_Point.h"
using namespace std;

namespace task {
typedef struct WayPoint {
  int id;
  float x;
  float y;
  float z;
} WayPoint;

ros::Subscriber last_task_sub;
ros::Subscriber task_state_sub;
ros::Subscriber task_path_sub;
ros::Subscriber allocated_task_state_sub;

ros::Publisher all_task_list_pub;
ros::Publisher all_tasks_allocate_pub;
ros::Publisher all_tasks_qt_pub;

int max_task_num = 20;                   // 接收的任务到达20个发出
vector<uav_msgs::TASK_State> all_tasks;  // 存放所有收到的任务(唯一)
uav_msgs::Task_List task_list_msg;

void TaskState_callBack(
    const uav_msgs::TASK_State::ConstPtr& msg);  // 接收任务信息
void TaskPath_callBack(const uav_msgs::Task_Path::ConstPtr& msg);
void LastTask_callBack(const uav_msgs::Last_Task::ConstPtr& msg);
void AllocatedTaskState_callback(const uav_msgs::TASK_State::ConstPtr& msg);
void ShowTaskInfo();             // 显示受到的任务
void AllTaskQt_Publish();        // 把所有的任务发送给qt
void AllTaskAllocate_Publish();  // 把任务发送给allocate
void readWayPoints();
}  // namespace task

#endif
