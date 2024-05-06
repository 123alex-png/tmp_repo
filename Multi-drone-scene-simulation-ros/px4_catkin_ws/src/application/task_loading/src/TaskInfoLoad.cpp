/*******************************************************************
 * File: TaskInfoLoad.cpp
 * Author: lizeshan zhangruiheng
 * Date: 2024-04-23
 * Description: 从任务文件中读取任务信息
 *******************************************************************/
#include "task_loading/TaskInfoLoad.h"


TaskInfoLoad::TaskInfoLoad()
    : nh_(ros::NodeHandle()), nh_private_(ros::NodeHandle("~")) {
  nh_private_.param<std::string>("task_directory", task_directory_,
                            "");  // 生成的任务信息路径
  task_state_pub = nh_.advertise<uav_msgs::TASK_State>("/task_state", 20, true);
  ROS_INFO("Task is loading---");
  bool read = false;
  while (ros::ok()) {
    if (read != true) {
      readFiles();
      read = true;
      ROS_INFO("Task loading finished!! ");
    }
    sleep(1);
    ros::spinOnce();
  }
}
TaskInfoLoad::~TaskInfoLoad() {}
bool TaskInfoLoad::readFiles() {
  FILE *input = fopen(task_directory_.c_str(), "r");
  if (input == NULL) {
    std::cout << "open file for read error\n" << std::endl;
  } else {
    while (!feof(input)) {
      uav_msgs::TASK_State task_msg;
      std::string task_name;
      std::string task_type;
      std::string req_uav_type;
      std::string req_payload_type;
      int task_R;
      std::cout << "open file success\n" << std::endl;
      fscanf(input,
             "%hd %s %s %hd %f %f %f %hd %f %f %f %s %s %hd %hd %hd %hd\n",
             &task_msg.task_id, task_name.c_str(), task_type.c_str(),
             &task_msg.EnterPoint.id, &task_msg.EnterPoint.way_point_pos.x,
             &task_msg.EnterPoint.way_point_pos.y,
             &task_msg.EnterPoint.way_point_pos.z, &task_msg.LeavePoint.id,
             &task_msg.LeavePoint.way_point_pos.x,
             &task_msg.LeavePoint.way_point_pos.y,
             &task_msg.LeavePoint.way_point_pos.z, req_uav_type.c_str(),
             req_payload_type.c_str(), &task_msg.Ts, &task_msg.Te,
             &task_msg.execute_time, &task_msg.R);
      task_msg.done =
          0;  // 0表示未分配  1表示已被分配  2 表示被分配但无满足无人机
              // 3表示正在被执行  4表示已经被执行
      task_msg.allocated = {-1};
      task_msg.task_name = task_name.c_str();
      task_msg.task_type = task_type.c_str();
      task_msg.req_uav_type = req_uav_type.c_str();
      task_msg.req_payload_type = req_payload_type.c_str();
      task_msg.S_t = 0;
      task_msg.E_t = 0;
      task_msg.path_length = 0;

      ROS_INFO(
          "----Loading Task:id:%d "
          "task_name:%s,type:%s,EnterPoint:%d,LeavePoint:%d,req_uav_type:%"
          "s,Ts:%hd,Te:%hd,R:%d",
          task_msg.task_id, task_msg.task_name.c_str(),
          task_msg.task_type.c_str(), task_msg.EnterPoint.id,
          task_msg.LeavePoint.id, task_msg.req_uav_type.c_str(), task_msg.Ts,
          task_msg.Te, task_msg.R);
      task_state_pub.publish(task_msg);
      sleep(1);
    }
    fclose(input);
  }
  return true;
}

void management_callBack(const std_msgs::String &msg) {
  ROS_INFO("TASK MANAGE READY");
  TaskInfoLoad *cont = new TaskInfoLoad();
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "task_loading");
  ros::NodeHandle nh;
  ros::Subscriber task_load_sub =
      nh.subscribe("/start_task_info_load", 10, &management_callBack);
  ros::spin();
  return 0;
}
