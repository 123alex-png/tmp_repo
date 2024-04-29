/*******************************************************************
 * File: uav_control.cpp
 * Author: lizeshan zhangruiheng
 * Date: 2024-04-23
 * Description:订阅无人机路径规划结果，并控制无人机飞行，对外发布无人机位置状态信息
 *******************************************************************/
#include "uav_control/uav_control.h"

inline float dist(const geometry_msgs::Point32 &a,
                  const geometry_msgs::Point32 &b) {
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
              (a.z - b.z) * (a.z - b.z));
}
inline float dist_xy(const geometry_msgs::Point32 &a,
                     const geometry_msgs::Point32 &b) {
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

void UAV::state_callback(const mavros_msgs::State::ConstPtr &msg) {
  current_state = *msg;
}
UAV::UAV() {
  nh.param<int>("uav_id", uav_id, 0);
  nh.param<string>("uav_name", uav_name, "uav_1");
  nh.param<string>("uav_type", uav_type, "HighwayPatrol");
  nh.param<string>("payload_type", payload_type, "Camera");
  nh.param<float>("power", power, 10000);
  nh.param<float>("state_pub_interval", state_pub_interval,
                  0.1);  // 每隔0.1s发布无人机状态
  nh.param<float>("velocity_size", velocity_size, 5);  // 飞行速度
  nh.param<float>("max_horizontal_vel", max_horizontal_vel,
                  15);  // 水平飞行速度
  nh.param<float>("max_ascending_vel", max_ascending_vel, 3);  // 上升飞行速度
  nh.param<float>("max_descent_vel", max_descent_vel, 3);  // 下降飞行速度
  nh.param<string>("way_point_directory", way_point_directory_, "");
  nh.param<float>("min_power", min_power, 60);  // 达到最小电量时返回
  nh.param<float>("flight_time", flight_time, 0);  // 飞行时间
  nh.param<int>("work_state", work_state, 3);  // 初始无人机均为空闲状态
  nh.param<int>("work_temperature_max", work_temperature_max,
                40);  // 无人机的最大工作温度
  nh.param<int>("work_temperature_min", work_temperature_min,
                -15);  //  无人机的最小工作温度
  nh.param<float>("max_comm_length", max_comm_length,
                  10000);  // 无人机的最大通信距离
  nh.param<float>("flight_height", flight_height, 6);  // 飞行高度
  nh.param<string>("payload_type", payload_type, "Camera");
  nh.param<float>("max_WR", max_WR, 15);  // 承受最大风速
  nh.param<float>("wl", wl, 6);
  nh.param<double>("x", var_x, 0);
  nh.param<double>("y", var_y, 0);
  nh.param<double>("z", var_z, 0);
  nh.param<float>("ground_x", ground_station.x, 0);
  nh.param<float>("ground_y", ground_station.y, 0);
  nh.param<float>("ground_z", ground_station.z, 0);
  flight_height = flight_height + uav_id * 1.5;

  //==============场景的相关安全距离控制============================
  flight_distance_error = 0.5;  // 达到目标位置的飞行距离误差 0.5米
  take_off_land_vel_size = 2;   // 起飞降落的飞行速度2m/s
  safety_dist = velocity_size * 1.2;  // 安全距离 1*1.2 m
  safety_dist_xy = 3;                 // 水平安全距离
  land_height = 2;                    // 降落的默认高度
  default_height = 10.0;              // 场景中允许的最大飞行高度

  has_task = false;

  stringstream ss;
  ss << "/uav" << uav_id;
  uav_topic_prefix = ss.str();
  uav_state_pub = nh.advertise<uav_msgs::UAV_State>(
      uav_topic_prefix + "/initial_state", 10);
  all_uav_state_pub = nh.advertise<uav_msgs::UAV_State>("/all_uav_state", 10);
  path_to_rviz_pub_ =
      nh.advertise<nav_msgs::Path>(uav_topic_prefix + "/trajectory", 10);
  uav_tasks_sub = nh.subscribe<uav_msgs::UAV_Tasks>(
      uav_topic_prefix + "/uav_tasks", 10, &UAV::accepted_task_callback, this);
  state_sub = nh.subscribe<mavros_msgs::State>(
      uav_topic_prefix + "/mavros/state", 10, &UAV::state_callback, this);
  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      uav_topic_prefix + "/mavros/setpoint_position/local", 10);
  flight_control_pub = nh.advertise<geometry_msgs::Twist>(
      uav_topic_prefix + "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
      uav_topic_prefix + "/mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_topic_prefix +
                                                           "/mavros/set_mode");
  local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      uav_topic_prefix + "/mavros/local_position/pose", 10,
      &UAV::local_pos_callback, this);
  local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(
      uav_topic_prefix + "/mavros/local_position/velocity_local", 10,
      &UAV::local_vel_callback, this);

  is_uav_state_pub = false;  // 标记向uav bid发送状态
  readWayPoint();
  path_.header.stamp = ros::Time::now();
  path_.header.frame_id = "my_frame";
}
UAV::~UAV() {}

void UAV::run() {
  ROS_INFO(" this is uav:%d", uav_id);
  ros::spinOnce();  //  update
  ros::Duration(0.1).sleep();
  geometry_msgs::Point32 start, goal, delta, adjust, last_pos;
  ros::Time start_time;
  ros::Duration used_time;
  std::stringstream ss;

  ros::Rate rate(20.0);
  // wait for FCU connection
  while (ros::ok() && !current_state.connected) {
    ros::spinOnce();
    rate.sleep();
  }
  for (int i = 100; ros::ok() && i > 0; --i) {
    ros::spinOnce();
    rate.sleep();
  }
  geometry_msgs::PoseStamped start_pose;
  start_pose.pose.position.x = position.x;
  start_pose.pose.position.y = position.y;
  start_pose.pose.position.z = 1;
  uav_start_pos.x = position.x;
  uav_start_pos.y = position.y;
  uav_start_pos.z = position.z;
  ROS_INFO(" uav:%d_start_pos:%f,%f,%f", uav_id, uav_start_pos.x,
           uav_start_pos.y, uav_start_pos.z);
  // 判断距离当前位置最近的waypoint点 并作为第一个中间点
  float s_dist = MaxValue;
  geometry_msgs::Point32 d;
  for (int i = 0; i < way_point_list.size(); i++) {
    d.x = way_point_list[i].x;
    d.y = way_point_list[i].y;
    if (dist_xy(uav_start_pos, d) < s_dist) {
      s_dist = dist_xy(uav_start_pos, d);
      uav_in_way_point = i;
    }
  }
  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i) {
    local_pos_pub.publish(start_pose);
    ros::spinOnce();
    rate.sleep();
  }
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  ros::Time last_request = ros::Time::now();
  ros::Time task_start_time;

  while (ros::ok()) {
    if (current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0))) {
      if (set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if (!current_state.armed &&
          (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
    if (power <= min_power) work_state = 4;
    switch (work_state) {
      case 0:  // take off
        if (abs(position.z - between_way_point_height[uav_in_way_point]) <
            flight_distance_error)  //
        {
          ROS_INFO("uav%d take off finish!", uav_id);
          work_state = 1;
        } else {
          take_off_land(between_way_point_height[uav_in_way_point]);
        }
        break;
      case 1:  // fly
        if (dist(position, current_task.EnterPoint.way_point_pos) <
            flight_distance_error) {
          ROS_INFO("uav%d arrive_task_%d EnterPoint !", uav_id,
                   current_task.task_id);
          uav_in_way_point = current_task.EnterPoint.id;
          while (!indirect_way_point.empty()) indirect_way_point.pop();
          work_state = 5;
        } else {
          if (!indirect_way_point.empty()) {
            geometry_msgs::Point32 indirect_goal_point;
            indirect_goal_point.x = indirect_way_point.front().x;
            indirect_goal_point.y = indirect_way_point.front().y;
            indirect_goal_point.z = indirect_way_point.front().z;

            if (dist(position, indirect_goal_point) <
                flight_distance_error)  // 当前到达中间路径点
            {
              ROS_INFO("uav%d arrive Tmp_Point %f %f %f...", uav_id,
                       indirect_way_point.front().x,
                       indirect_way_point.front().y,
                       indirect_way_point.front().z);
              uav_in_way_point = indirect_way_point.front().id;
              ROS_INFO("1");
              indirect_way_point.pop();
            } else  // 未到达
            {
              delta.x = indirect_goal_point.x - position.x;
              delta.y = indirect_goal_point.y - position.y;
              delta.z = indirect_goal_point.z - position.z;
              float size = dist(indirect_goal_point, position);
              if (size > velocity_size) {
                delta.x = delta.x / size * velocity_size;
                delta.y = delta.y / size * velocity_size;
                delta.z = delta.z / size * velocity_size;
              }
              vel_flight_control(delta.x, delta.y, delta.z);
            }
          } else {
            ROS_WARN(
                "uav%d indirect_way_point is empty and todotask is "
                "%d,EnterPoint:%d",
                uav_id, current_task.task_id, current_task.EnterPoint.id);
          }
        }
        break;
      case 2:  // land
        if (fabs(position.z - land_height) < flight_distance_error) {
          ROS_INFO("uav%d land on position with (%f,%f,%f)!", uav_id,
                   position.x, position.y, position.z);
          work_state = 3;
        } else {
          take_off_land(land_height);
        }
        break;
      case 3:  // idle
        if (accepted_tasks.size()) {
          for (int i = 0; i < accepted_tasks.size(); i++) {
            if (!accepted_tasks[i].done) {
              current_task = accepted_tasks[i];
              current_task_id = i;
              has_task = true;
              break;
            }
          }
        }
        if (has_task) {
          ss.str("");
          ss.clear();
          ss << "id:" << current_task.task_id << ", pos:("
             << current_task.EnterPoint.way_point_pos.x << ","
             << current_task.EnterPoint.way_point_pos.y << ","
             << current_task.EnterPoint.way_point_pos.z << ")";
          ROS_INFO("uav%d doing task! info: %s", uav_id, ss.str().c_str());
          way_point start_;
          way_point goal_;
          start_ = way_point_list[uav_in_way_point];           // 当前位置
          goal_ = way_point_list[current_task.EnterPoint.id];  // 目标位置
          ROS_INFO("start_point :%d (%f,%f,%f) ,end_point: %d (%f,%f,%f)",
                   start_.id, start_.x, start_.y, start_.z, goal_.id, goal_.x,
                   goal_.y, goal_.z);
          addIndirectWayPoint(start_, goal_);
          ROS_INFO("EnterPoint xyz :(%f,%f,%f) ,LeavePoint:(%f,%f,%f)",
                   current_task.EnterPoint.way_point_pos.x,
                   current_task.EnterPoint.way_point_pos.y,
                   current_task.EnterPoint.way_point_pos.z,
                   current_task.LeavePoint.way_point_pos.x,
                   current_task.LeavePoint.way_point_pos.y,
                   current_task.LeavePoint.way_point_pos.z);
          for (int j = 0; j < current_task.task_path.size(); j++) {
            goals.push(current_task.task_path[j]);
          }
          work_state = 0;
          task_start_time = ros::Time::now();
        } else {
          // back to home  or land
          work_state = 3;
        }
        break;
      case 4:  // fault
        if (fabs(position.z - land_height) < flight_distance_error) {
          ROS_WARN("[uav fault] uav%d is out of power!", uav_id);
        } else {
          ROS_WARN("[uav fault] uav%d is out of power! uav land", uav_id);
          take_off_land(land_height);
        }
        break;
      case 5:  // execute
        if (dist(position, current_task.LeavePoint.way_point_pos) <
                flight_distance_error &&
            goals.empty())  // 执行完任务
        {
          ROS_INFO("uav%d arrive_task_%d LeavePoint !", uav_id,
                   current_task.task_id);
          uav_in_way_point = current_task.LeavePoint.id;
          current_task.done = true;
          accepted_tasks[current_task_id].done = true;
          has_task = false;
          work_state = 2;
        } else  // 未执行完任务
        {
          if (!goals.empty()) {
            geometry_msgs::Point32 tmp_point = goals.front();
            delta.x = tmp_point.x - position.x;
            delta.y = tmp_point.y - position.y;
            delta.z = tmp_point.z - position.z;
            float size = dist(tmp_point, position);
            if (size > velocity_size) {
              delta.x = delta.x / size * velocity_size;
              delta.y = delta.y / size * velocity_size;
              delta.z = delta.z / size * velocity_size;
            }
            vel_flight_control(delta.x, delta.y, delta.z);
            if ((dist(tmp_point, position) < flight_distance_error)) {
              goals.pop();
            }
          } else {
            ROS_WARN(
                "uav %d path_planning goals are empty but not arrvie "
                "LeavePoint",
                uav_id);
          }
        }
        break;

      default:
        break;
    }
    ros::spinOnce();
    rate.sleep();
    trajectoryVisualize();
  }
}

void UAV::take_off_land(float h) {
  for (int i = 0; i < uav_dist_xy.size(); i++)
    if (i != uav_id && uav_dist_xy[i] > 0 && uav_dist_xy[i] < safety_dist_xy &&
        uav_z[i] < position.z && uav_z[i] + 1.5 >= h) {
      h = uav_z[i] + 1.5;
      if (h < default_height + 1.5) h = default_height + 1.5;
    }
  float delta = h - position.z;
  float size = fabs(delta);
  if (size > take_off_land_vel_size)
    delta = delta / size * take_off_land_vel_size;
  vel_flight_control(0, 0, delta);
}

void UAV::vel_flight_control(float x, float y, float z) {
  geometry_msgs::Twist msg;
  msg.linear.x = x;
  msg.linear.y = y;
  msg.linear.z = z;
  flight_control_pub.publish(msg);
}

void UAV::accepted_task_callback(const uav_msgs::UAV_Tasks::ConstPtr &msg) {
  if (msg->uav_id == uav_id) {
    for (int i = 0; i < msg->uav_tasks.size(); i++) {
      ROS_INFO("uav_%d has accepted the task: ", uav_id);
      accepted_tasks.push_back(msg->uav_tasks[i]);
    }
  }
}

void UAV::publish_uav_state() {
  uav_msgs::UAV_State msg;
  msg.uav_id = uav_id;
  msg.uav_name = uav_name;
  msg.uav_type = uav_type;
  msg.payload_type = payload_type;
  msg.uav_start_pos = uav_start_pos;
  msg.ground_station = ground_station;
  msg.position = position;
  msg.velocity_size = max_horizontal_vel;
  msg.work_temperature_max = work_temperature_max;
  msg.work_temperature_min = work_temperature_min;
  msg.flight_time = flight_time;
  msg.power = power;
  msg.max_comm_length = max_comm_length;
  msg.max_WR = max_WR;
  msg.wl = wl;
  for (int i = 0; i < accepted.size(); i++) msg.accepted[i] = accepted[i];
  msg.work_state = work_state;
  uav_state_pub.publish(msg);
}

void UAV::local_vel_callback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
  velocity.x = msg->twist.linear.x;
  velocity.y = msg->twist.linear.y;
  velocity.z = msg->twist.linear.z;
}

void UAV::local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  position.x = msg->pose.position.x + var_x;
  position.y = msg->pose.position.y + var_y;
  position.z = msg->pose.position.z + var_z;
  static ros::Time start_t = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_t;
  if (elapsed_time > ros::Duration(state_pub_interval)) {
    start_t = ros::Time::now();
    publish_all_uav_state();
  }
  if (!is_uav_state_pub && uav_state_pub.getNumSubscribers() >= 1) {
    is_uav_state_pub = true;
    ROS_INFO("publish uav %d state...", uav_id);
    publish_uav_state();
  }
}

void UAV::trajectoryVisualize() {
  geometry_msgs::PoseStamped this_pose_stamped;
  this_pose_stamped.pose.position.x = position.x;
  this_pose_stamped.pose.position.y = position.y;
  this_pose_stamped.pose.position.z = position.z;
  this_pose_stamped.pose.orientation.x = 0;
  this_pose_stamped.pose.orientation.y = 0;
  this_pose_stamped.pose.orientation.z = 0;
  this_pose_stamped.pose.orientation.w = 1;
  this_pose_stamped.header.stamp = ros::Time::now();
  this_pose_stamped.header.frame_id = "my_frame";
  path_.poses.push_back(this_pose_stamped);
  path_to_rviz_pub_.publish(path_);
}

void UAV::readWayPoint() {
  way_point_list.resize(0);
  between_way_point_height.resize(0);
  ifstream way_point_file;
  way_point_file.open(way_point_directory_, ios::in);
  if (!way_point_file) {
    cout << "open way_point file for read error\n" << endl;
  }
  way_point tmp_way_point;
  float tmp_way_point_height;
  while (way_point_file >> tmp_way_point.id >> tmp_way_point.x >>
         tmp_way_point.y >> tmp_way_point.z >> tmp_way_point_height) {
    way_point_list.push_back(tmp_way_point);
    between_way_point_height.push_back(tmp_way_point_height);
  }
}

bool UAV::addIndirectWayPoint(const way_point &start, const way_point &goal) {
  way_point tmp_point;
  way_point current_point;
  if (start.id < goal.id)  // 从小的往大的飞  高度采用id小的
  {
    for (int i = start.id; i < goal.id; i++) {
      ROS_INFO("AddIndirectWayPoint: %d", i);
      tmp_point = way_point_list[i];
      tmp_point.z = between_way_point_height[i];
      indirect_way_point.push(tmp_point);
      tmp_point = way_point_list[i + 1];
      tmp_point.z = between_way_point_height[i];
      indirect_way_point.push(tmp_point);
    }
    ROS_INFO("AddIndirectWayPoint: %d", goal.id);
    indirect_way_point.push(goal);

    return true;
  } else if (start.id > goal.id)  // 从大往小飞，点采用小的
  {
    for (int j = start.id; j > goal.id; j--) {
      ROS_INFO("AddIndirectWayPoint: %d", j);
      tmp_point = way_point_list[j];
      tmp_point.z = between_way_point_height[j - 1];
      indirect_way_point.push(tmp_point);
      tmp_point = way_point_list[j - 1];
      tmp_point.z = between_way_point_height[j - 1];
      indirect_way_point.push(tmp_point);
    }
    ROS_INFO("AddIndirectWayPoint: %d", goal.id);
    indirect_way_point.push(goal);
    return true;
  } else {
    ROS_INFO("AddIndirectWayPoint: %d", start.id);
    tmp_point = way_point_list[start.id];
    tmp_point.z = between_way_point_height[start.id];
    indirect_way_point.push(tmp_point);
    return true;
  }
  return false;
}

void UAV::publish_all_uav_state() {
  uav_msgs::UAV_State msg;

  msg.uav_id = uav_id;
  msg.uav_name = uav_name;
  msg.uav_type = uav_type;
  msg.payload_type = payload_type;
  msg.uav_start_pos = uav_start_pos;
  msg.ground_station = ground_station;
  msg.position = position;
  msg.velocity_size = max_horizontal_vel;
  msg.work_temperature_max = work_temperature_max;
  msg.work_temperature_min = work_temperature_min;
  msg.flight_time = flight_time;
  msg.power = power;
  msg.max_comm_length = max_comm_length;
  msg.max_WR = max_WR;
  msg.wl = wl;

  for (int i = 0; i < accepted_tasks.size(); i++)
    msg.accepted.push_back(accepted_tasks[i].task_id);
  msg.work_state = work_state;

  all_uav_state_pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, ros::this_node::getName().c_str());
  ROS_INFO("%s", ros::this_node::getName().c_str());
  UAV uav;
  uav.run();
  return 0;
}
