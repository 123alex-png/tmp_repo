/*******************************************************************
 * File: TaskInfoLoad.h
 * Author: lizeshan zhangruiheng
 * Date: 2024-04-23
 * Description: 从任务文件中读取任务信息
 *******************************************************************/
#ifndef __TASK_INFO_LOAD_H_
#define __TASK_INFO_LOAD_H_

#include <ros/ros.h>
#include <stdio.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "std_msgs/String.h"
#include "uav_msgs/TASK_State.h"

using namespace std;

class TaskInfoLoad {
 private:
  string task_directory_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher task_state_pub;
  uav_msgs::TASK_State task_state_msg_;

 public:
  TaskInfoLoad();
  ~TaskInfoLoad();
  bool readFiles();
};
/**
 * @brief: 表明任务信息已经发送给management节点
 * @param msg：management节点的反馈消息
 *
 * @return
 */
void management_callBack(const std_msgs::String& msg);

#endif
