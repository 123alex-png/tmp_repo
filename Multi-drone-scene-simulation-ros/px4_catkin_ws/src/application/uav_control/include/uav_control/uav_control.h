/*******************************************************************
 * File: uav_control.h
 * Author: lizeshan zhangruiheng
 * Date: 2024-04-23
 * Description:订阅无人机路径规划结果，并控制无人机飞行，对外发布无人机位置状态信息
 *******************************************************************/
#ifndef UAV_CONTROL_CONTROL_H
#define UAV_CONTROL_CONTROL_H

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <uav_msgs/TASK_State.h>
#include <uav_msgs/UAV_IDQP.h>
#include <uav_msgs/UAV_State.h>
#include <uav_msgs/UAV_Tasks.h>

#include <cfloat>
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include <vector>
#define MaxValue 0x7fffffff
using namespace std;

inline float dist(const geometry_msgs::Point32 &a,
                  const geometry_msgs::Point32 &b);
inline float dist_xy(const geometry_msgs::Point32 &a,
                     const geometry_msgs::Point32 &b);

class UAV {
  typedef struct way_point {
    int id;
    float x;
    float y;
    float z;
  } way_point;

 protected:
  int uav_id;
  int work_temperature_min;  // 可工作温度范围min
  int work_temperature_max;  // 可工作温度范围max
  int work_state = 3;
  int uav_in_way_point;
  int current_task_id;
  string uav_name;
  string uav_type;
  string payload_type;
  geometry_msgs::Point32 uav_start_pos;
  geometry_msgs::Point32 ground_station;
  geometry_msgs::Point32 position;  //
  geometry_msgs::Point32 velocity;  // 记录实时速度 xyz

  float velocity_size;       // 速度大小
  float max_horizontal_vel;  //  最大水平飞行速度
  float max_ascending_vel;   //  最大上升速度
  float max_descent_vel;     // 最大下降速度
  float flight_time;         // 飞行时间
  float flight_dist;         // 飞行距离
  float power;               // 电量
  float min_power;           // 最小剩余电量
  float max_comm_length;     // 最大通信距离
  float max_WR;              // 最大风速
  float wl;                  // 防水等级

  vector<int> accepted;  // 记录每架无人机执行任务id列表
  vector<uav_msgs::TASK_State>
      accepted_tasks;  // 存从task_allocation发送的uav_tasks，记录每架无人机的执行任务列表
  vector<way_point> way_point_list;
  vector<float> between_way_point_height;

  queue<way_point> indirect_way_point;  // 间接路径点
  string way_point_directory_;
  string uav_topic_prefix;
  bool has_task;  // 无人机当前是否有任务做

  // flight control
  mavros_msgs::State current_state;
  bool is_uav_state_pub;
  float state_pub_interval;  // 状态发布时间间隔
  double var_x;              // 本地位置相对与home位置的x变化量
  double var_y;              // 本地位置相对于home位置的y变化量
  double var_z;              // 本地位置相对于home位置的z变化量

  float take_off_land_vel_size;
  float safety_dist;
  float safety_dist_xy;
  float flight_height;
  float flight_distance_error;  // 飞行距离误差  包括起飞、降落 目标点 等等
  float default_height;
  float take_off_height;
  float land_height;
  std::vector<float> uav_dist;
  std::vector<float> uav_dist_xy;
  std::vector<float> uav_z;

  uav_msgs::TASK_State current_task;

  ros::NodeHandle nh;
  ros::Publisher uav_state_pub;
  ros::Publisher all_uav_state_pub;
  ros::Publisher flight_control_pub;
  ros::Publisher path_to_rviz_pub_;
  ros::Publisher local_pos_pub;

  ros::Subscriber uav_tasks_sub;
  ros::Subscriber state_sub;
  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;
  ros::Subscriber local_pos_sub;
  ros::Subscriber local_vel_sub;

  nav_msgs::Path path_;  // 用于向rviz发送实际飞行路径

  queue<geometry_msgs::Point32> goals;

  void readWayPoint();
  /**
   * @brief: 生成任务间的飞行路径点
   * @param start：当前点
   * @param goal：目标点
   *
   * @return 是否成功添加航迹点
   */
  bool addIndirectWayPoint(const way_point &start, const way_point &goal);
  /**
   * @brief: 接收拍卖得到的任务的信息
   * @param msg：任务信息
   */
  void accepted_task_callback(const uav_msgs::UAV_Tasks::ConstPtr &msg);
  /**
   * @brief: 实时更新无人机的位置信息
   * @param msg：无人机当前位置
   */
  void local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  /**
   * @brief: 实时更新无人机的速度信息
   * @param msg：无人机当前速度
   */
  void local_vel_callback(const geometry_msgs::TwistStamped::ConstPtr &msg);
  /**
   * @brief: 获取uav状态模式
   * @param msg：无人机当前模式
   */
  void state_callback(const mavros_msgs::State::ConstPtr &msg);
  /**
   * @brief: 发布无人机自身信息
   */
  void publish_uav_state();
  /**
   * @brief: 对qt发布无人机的信息
   */
  void publish_all_uav_state();
  /**
   * @brief: 控制无人机起飞
   * @param h：飞行高度h
   */
  void take_off_land(float h);
  /**
   * @brief: 通过速度控制无人机飞行
   * @param x：x方向上速度分量
   * @param y：y方向上速度分量
   * @param z：z方向上速度分量
   */
  void vel_flight_control(float x, float y, float z);
  /**
   * @brief: 通过rviz发布无人机的飞行轨迹
   */
  void trajectoryVisualize();

 public:
  UAV();
  ~UAV();
  void run();
};
#endif
