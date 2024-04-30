/*******************************************************************
 * File: start_view_point_plan.cpp
 * Author: lizeshan
 * Date: 2024-04-23
 * Description: 向对应无人机的路径规划节点发布命令，让其开始规划路径
 *******************************************************************/
#include <nav_msgs/GridCells.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <uav_msgs/UAV_State.h>

#include "std_msgs/String.h"

using namespace std;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "start_view_point_plan");
  ros::NodeHandle nh;
  ros::Publisher start_view_plan_pub =
      nh.advertise<uav_msgs::UAV_State>("/start_view_point_plan", 10);
  uav_msgs::UAV_State uav;
  int number_of_uav = 5;
  int number;  // 阻塞进程
  cin >> number;
  for (int i = 0; i < number_of_uav; i++) {
    uav.uav_id = i;
    start_view_plan_pub.publish(uav);
    ROS_INFO("star_view_point_plan");
  }
  ros::spin();

  return 0;
}
