/*******************************************************************
 * File: start_execute_task.cpp
 * Author: lizeshan
 * Date: 2024-04-23
 * Description: 向对应无人机节点发布命令，让其开始执行任务
 *******************************************************************/
#include <nav_msgs/GridCells.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <uav_msgs/UAV_State.h>

#include "std_msgs/String.h"

using namespace std;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "start_execute_task");
  ros::NodeHandle nh;
  ros::Publisher start_execute_task_pub =
      nh.advertise<uav_msgs::UAV_State>("/start_execute_task", 10);
  uav_msgs::UAV_State uav;
  int number;  // 阻塞进程
  cin >> number;
  for (int i = 0; i < 5; i++) {
    uav.uav_id = i;
    ROS_INFO("start");
    start_execute_task_pub.publish(uav);
  }
  ros::spin();

  return 0;
}
