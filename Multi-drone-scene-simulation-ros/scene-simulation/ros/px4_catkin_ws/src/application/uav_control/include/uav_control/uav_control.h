#ifndef UAV_CONTROL_CONTROL_H
#define UAV_CONTROL_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <cfloat>
using namespace std;

#include <uav_msgs/TASK_State.h>
#include <uav_msgs/UAV_State.h>
#include <uav_msgs/UAV_Tasks.h>
#include <uav_msgs/UAV_IDQP.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Twist.h>

// #include <hector_uav_msgs/EnableMotors.h>   //仿真下驱动hector包电机
#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <std_msgs/Bool.h>
#define MaxValue 0x7fffffff
inline float dist(const geometry_msgs::Point32 &a, const geometry_msgs::Point32 &b);
inline float dist_xy(const geometry_msgs::Point32 &a, const geometry_msgs::Point32 &b);

class UAV
{
    typedef struct way_point
    {
        int id;
        float x;
        float y;
        float z;
    } way_point;

protected:
    int uav_id;
    string uav_name;
    string uav_type;
    string payload_type;
    geometry_msgs::Point32 uav_start_pos;
    geometry_msgs::Point32 ground_station;
    geometry_msgs::Point32 position; //
    geometry_msgs::Point32 velocity; // 记录实时速度 xyz

    float velocity_size;             // 速度类型  2为xy轴速度  3为xyz轴速度？ 不知道 速度大小
    float max_horizontal_vel;        //  最大水平飞行速度
    float max_ascending_vel;         //  最大上升速度
    float max_descent_vel;           //最大下降速度 
    int work_temperature_min; // 可工作温度范围min
    int work_temperature_max; // 可工作温度范围max


    float flight_time;                           // 飞行时间
    float flight_dist;                           // 飞行距离
    float power;                                 // 电量
    float min_power;                             // 最小剩余电量
    float max_comm_length;                       // 最大通信距离
    float max_WR;                      //最大风速
    float wl;                           //防水等级
    //string mission_payload;                      // 无人机携带的任务载荷
    int max_payload;                             // 最大载荷重量
    vector<int> accepted;                        // 记录每架无人机执行任务id列表
    vector<uav_msgs::TASK_State> accepted_tasks; // 存从task_allocation发送的uav_tasks，记录每架无人机的执行任务列表
    bool finish_accetped_tasks;
    int work_state=3; // 0:take off 1:fly  2:land  3:idle  4:invalid

    vector<way_point> way_point_list;
    vector<float> between_way_point_height;
    int uav_in_way_point;
    way_point uav_in_way_point_start;
    way_point uav_in_way_point_goal;
    queue<way_point> indirect_way_point; // 间接路径点

    string way_point_directory_;
    string uav_topic_prefix;
    bool has_task; // 无人机当前是否有任务做
    int to_do_task_i;

    // flight control
    mavros_msgs::State current_state;

    bool use_path_planning;
    bool is_uav_state_pub;    // 是否发送无人机状态给 ‘/uavx/uav_state’
    float state_pub_interval; // 状态发布时间间隔
    double var_x;   // 本地位置相对与home位置的x变化量
    double var_y;    //本地位置相对于home位置的y变化量
    double var_z;    //本地位置相对于home位置的z变化量
    float finish_error;
    float take_off_land_vel_size;
    float safety_dist;
    float safety_dist_xy;
    float flight_height;
    float flight_distance_error; // 飞行距离误差  包括起飞、降落 目标点 等等
    float default_height;
    float take_off_height;
    float land_height;
    float horizontal_move_time;
    float vertical_move_time;
    float horizontal_move_dist;
    float vertical_move_dist;
    float max_uav_load;
    std::vector<float> uav_dist;
    std::vector<float> uav_dist_xy;
    std::vector<float> uav_z;

    uav_msgs::TASK_State to_do_task;

    ros::NodeHandle nh;
    ros::Subscriber uav_tasks_sub; // 接收需要作的task 从UAV_State中
    ros::Publisher uav_state_pub;  // 发布无人机状态
    ros::Publisher all_uav_state_pub;
    ros::Subscriber odometry_sub;      // 里程计接收?
    ros::Publisher flight_control_pub; // 飞控发布
    ros::ServiceClient enable_motors_client;
    ros::Subscriber idqp_sub;

    ros::Publisher path_to_rviz_pub_;
    ros::Publisher task_state_pub;
    ros::Subscriber grid_cells_sub;
    ros::Publisher task_receive_pub;
    ros::Publisher idqp_pub;
    // ros::Publisher path_param_pub;
    // ros::Subscriber path_result_sub_;

    ros::Subscriber state_sub;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Subscriber local_pos_sub;
    ros::Subscriber local_vel_sub;

    nav_msgs::Path path_; // 用于向rviz发送实际飞行路径

    queue<geometry_msgs::Point32> goals;
    // vector<geometry_msgs::Point> tmp_path;
    void ReadWayPoint();
    bool AddIndirectWayPoint(const way_point &start, const way_point &goal);

    // service
    //bool enable_motors();
    //void get_uav_next_task(int id);

    // callback
    void accepted_task_callback(const uav_msgs::UAV_Tasks::ConstPtr &msg);
    // void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void grid_cells_callback(const nav_msgs::GridCells::ConstPtr &msg);
    void idqp_callback(const uav_msgs::UAV_IDQP::ConstPtr &msg);  //计算机间距离

    void local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg); //获取local位置
    void local_vel_callback(const geometry_msgs::TwistStamped::ConstPtr &msg); //获取local速度
    void state_callback(const mavros_msgs::State::ConstPtr &msg); //获取uav状态模式
    // publish
    void publish_uav_state();  // 发送给规划模块使用
    void publish_all_uav_state();  // 发送给无人机数据显示模块使用
    void take_off_land(float h);  // 起飞或降落指定高度
    void vel_flight_control(float x, float y, float z); //飞到目标位置
    void publish_uav_idqp();   // 发布无人机id pos vel
    void trajectoryVisualize();

public:
    UAV();
    ~UAV();
    void run();
};
#endif

inline float dist(const geometry_msgs::Point32 &a, const geometry_msgs::Point32 &b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}
inline float dist_xy(const geometry_msgs::Point32 &a, const geometry_msgs::Point32 &b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}
void UAV::state_callback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}
UAV::UAV()
{
    //--------------------从launch文件获取param---------------------------------------------------------------------------
    nh.param<int>("uav_id", uav_id, 0);
    nh.param<string>("uav_name", uav_name, "uav_1");
    nh.param<string>("uav_type", uav_type, "HighwayPatrol");
    nh.param<string>("payload_type", payload_type, "Camera");
    nh.param<float>("power", power, 3000);
    nh.param<float>("state_pub_interval", state_pub_interval, 0.1); // 每隔0.1s发布无人机状态
    nh.param<float>("velocity_size", velocity_size, 5);             // 飞行速度1m/s
    nh.param<float>("max_horizontal_vel", max_horizontal_vel, 10);  // 水平飞行速度10m/s
    nh.param<float>("max_ascending_vel", max_ascending_vel, 3);     // 上升飞行速度3m/s
    nh.param<float>("max_descent_vel", max_descent_vel, 3);         // 下降飞行速度3m/s
    nh.param<string>("way_point_directory", way_point_directory_, "/home/cs504/px4_catkin_ws/src/application/uav_control/data/waypoint_hill.txt");
    nh.param<bool>("use_path_planning", use_path_planning, true);
    nh.param<float>("min_power", min_power, 60);                      // 默认达到最小电量60s时返回
    nh.param<float>("flight_time", flight_time, 0);                 // 默认飞行时间为0
    nh.param<int>("work_state", work_state, 3);                       // 默认初始无人机均为3:空闲状态
    nh.param<int>("work_temperature_max", work_temperature_max, 40);  // 默认无人机的最大工作温度40
    nh.param<int>("work_temperature_min", work_temperature_min, -15); //  默认无人机的最小工作温度-15
    nh.param<float>("max_comm_length", max_comm_length, 3000);        // 默认无人机的最大通信距离3000米
    nh.param<int>("uav_in_way_point", uav_in_way_point, 0);           // 初始时在 0路径点位置
    nh.param<float>("flight_height", flight_height, 10);               // 默认飞行高度
    nh.param<string>("payload_type",payload_type,"Camera");               // 默认飞行高度
    nh.param<float>("max_WR",max_WR, 15);      //承受最大风速15m/s
    nh.param<float>("wl",wl, 6);
    nh.param<double>("x",var_x,0);
    nh.param<double>("y",var_y,0);
    nh.param<double>("z",var_z,0);
    nh.param<float>("ground_x",ground_station.x,0);
    nh.param<float>("ground_y",ground_station.y,0);
    nh.param<float>("ground_z",ground_station.z,0);
    flight_height = flight_height + uav_id * 1.5;

    //==============场景的相关安全距离控制============================
    flight_distance_error = 0.5;       // 达到目标位置的飞行距离误差 0.5米
    take_off_land_vel_size = 2;        // 起飞降落的飞行速度2m/s
    safety_dist = velocity_size * 1.2; // 安全距离 1*1.2 m
    safety_dist_xy = 3;                // 水平安全距离
    land_height = 2;                   // 降落的默认高度
    default_height = 20.0;             // 场景中允许的最大飞行高度

    has_task = false; // 当前任务列表内是否有任务做

    stringstream ss;
    ss << "/uav" << uav_id;
    uav_topic_prefix = ss.str(); //"/uav0"
    // ROS_INFO("uav_topic_prefix:%s", uav_topic_prefix.c_str());
    uav_state_pub = nh.advertise<uav_msgs::UAV_State>(uav_topic_prefix + "/initial_state", 10);
    all_uav_state_pub = nh.advertise<uav_msgs::UAV_State>("/all_uav_state", 10); // 用于实时发送无人各项信息 用于数据库存储及展示

    path_to_rviz_pub_ = nh.advertise<nav_msgs::Path>(uav_topic_prefix + "/trajectory", 10); // 发送无人机实际飞行的轨迹
    task_state_pub = nh.advertise<uav_msgs::TASK_State>("/task_state", 10);                 // 用作飞行执行过后更新任务状态
    idqp_pub = nh.advertise<uav_msgs::UAV_IDQP>("/uav_idqp", 10);
    // path_param_pub = nh.advertise<path_planning::PathParam>("path_param", 10);

    uav_tasks_sub = nh.subscribe<uav_msgs::UAV_Tasks>(uav_topic_prefix + "/uav_tasks", 10, &UAV::accepted_task_callback, this); // 用于接收/uav?/uav_tasks收到的tasks
    // odometry_sub = nh.subscribe(uav_topic_prefix + "/ground_truth/state", 10, &UAV::odometry_callback, this); // 接收gazebo发来的里程？ 意思是gazebo向uavcontrol发送无人机的起始位置等参数
    grid_cells_sub = nh.subscribe<nav_msgs::GridCells>("/barrier", 10, &UAV::grid_cells_callback, this);
    idqp_sub = nh.subscribe("/uav_idqp", 10, &UAV::idqp_callback, this);
    // path_result_sub_ = nh.subscribe<path_planning::PathResult>("path_result", 10, &UAV::path_result_callback,this);

    // enable_motors_client = nh.serviceClient<hector_uav_msgs::EnableMotors>(uav_topic_prefix + "/enable_motors"); //驱动电机

    state_sub = nh.subscribe<mavros_msgs::State>(uav_topic_prefix + "/mavros/state", 10, &UAV::state_callback, this);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_topic_prefix + "/mavros/setpoint_position/local", 10);
    flight_control_pub = nh.advertise<geometry_msgs::Twist>(uav_topic_prefix + "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_topic_prefix + "/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_topic_prefix + "/mavros/set_mode");
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav_topic_prefix + "/mavros/local_position/pose", 10, &UAV::local_pos_callback, this); // 用于获取实时local位置
    local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(uav_topic_prefix + "/mavros/local_position/velocity", 10, &UAV::local_vel_callback, this);

    is_uav_state_pub = false; // 标记是否向uav bid发送uav状态
    ReadWayPoint();
    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = "my_frame";
}
UAV::~UAV()
{
}

void UAV::run()
{
    ROS_INFO(" this is uav:%d", uav_id);
    ros::spinOnce(); //  update
    ros::Duration(0.1).sleep();

    geometry_msgs::Point32 start, goal, delta, adjust, last_pos;
    ros::Time start_time;
    ros::Duration used_time;


    std::stringstream ss;

    // geometry_msgs::Point32 tmp_point;
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
   for (int i = 100; ros::ok() && i > 0; --i)
    {
        ros::spinOnce();
        rate.sleep();
    }

   
    geometry_msgs::PoseStamped start_pose;
    start_pose.pose.position.x = position.x;
    start_pose.pose.position.y = position.y;
    start_pose.pose.position.z = position.z;
    uav_start_pos.x = position.x;
    uav_start_pos.y = position.y;
    uav_start_pos.z = position.z;
    ROS_INFO(" uav:%d_start_pos:%f,%f,%f", uav_id,uav_start_pos.x,uav_start_pos.y,uav_start_pos.z);
    // 判断距离当前位置最近的waypoint点 并作为第一个中间点
    float s_dist = MaxValue;
    geometry_msgs::Point32 d;
    for(int i=0;i<way_point_list.size();i++)
    {   
        d.x = way_point_list[i].x;
        d.y = way_point_list[i].y;
        if(dist_xy(uav_start_pos,d)<s_dist)
        {
            s_dist = dist_xy(uav_start_pos,d);
            uav_in_way_point = i;
        }
    }
    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(start_pose);
        ros::spinOnce();
        rate.sleep();
    }

   
   
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    
    while (ros::ok())
    {
        //ROS_INFO("%s",current_state.mode.c_str());
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                //ROS_INFO("Offboard enabled");
               
            }
            last_request = ros::Time::now();
        }
        else
        {   
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                   
                }
                last_request = ros::Time::now();
            }
        }
        //local_pos_pub.publish(start_pose);
        // check out of power
        if (power <= min_power) // 剩余电量小于最小电量  状态转向故障
            work_state = 4;
        switch (work_state)
        {
        case 0:                                                          // take off
            if (abs(position.z - flight_height) < flight_distance_error) //
            {
                ROS_INFO("uav%d take off finish!", uav_id);
                work_state = 1;
            }
            else
            {
                // ROS_INFO("uav%d is taking off...",uav_id);
                take_off_land(flight_height);
            }
            break;
        case 1: // fly
            if (dist(position, to_do_task.EnterPoint.way_point_pos) < flight_distance_error)
            {
                ROS_INFO("uav%d arrive_task_%d EnterPoint !", uav_id, to_do_task.task_id);
                uav_in_way_point = to_do_task.EnterPoint.id;
                while (!indirect_way_point.empty())
                {
                    indirect_way_point.pop();
                }
                work_state = 5;
            }
            else
            {
                if (!indirect_way_point.empty())
                {
                    geometry_msgs::Point32 indirect_goal_point;
                    indirect_goal_point.x = indirect_way_point.front().x;
                    indirect_goal_point.y = indirect_way_point.front().y;
                    indirect_goal_point.z = indirect_way_point.front().z;

                    if (dist(position, indirect_goal_point) < flight_distance_error) // 当前到达中间路径点
                    {
                        ROS_INFO("uav%d arrive Tmp_Point %f %f %f...", uav_id, indirect_way_point.front().x, indirect_way_point.front().y, indirect_way_point.front().z);
                        uav_in_way_point = indirect_way_point.front().id;
                        indirect_way_point.pop();
                    }
                    else // 未到达
                    {    // 先检查高度是否满足要求
                        // 若满足 则直接飞
                        // 若不满足 则抬升高度 然后再直接飞
                        // ROS_INFO("uav%d is going to indirect_goal_point %d ,(%f,%f,%f)...", uav_id,indirect_way_point.front().id,indirect_way_point.front().x,indirect_way_point.front().y,indirect_way_point.front().z);
                        delta.x = indirect_goal_point.x - position.x;
                        delta.y = indirect_goal_point.y - position.y;
                        delta.z = indirect_goal_point.z - position.z;
                        float size = dist(indirect_goal_point, position);
                        if (size > velocity_size)
                        {
                            delta.x = delta.x / size * velocity_size;
                            delta.y = delta.y / size * velocity_size;
                            delta.z = delta.z / size * velocity_size;
                        }
                        vel_flight_control(delta.x, delta.y, delta.z);
                    }
                }
                else
                {
                    ROS_WARN("uav%d indirect_way_point is empty and todotask is %d,EnterPoint:%d", uav_id, to_do_task.task_id, to_do_task.EnterPoint.id);
                }
            }
            break;
        case 2: // land
            if (fabs(position.z - land_height) < flight_distance_error)
            {
                ROS_INFO("uav%d land on position with (%f,%f,%f)!", uav_id, position.x, position.y, position.z);
                work_state = 3;

                // done
            }
            else
            {
                take_off_land(land_height);
            }
            break;
        case 3: // idle
            if (accepted_tasks.size())
            {
                for (int i = 0; i < accepted_tasks.size(); i++)
                {
                    if (accepted_tasks[i].done == false)
                    {
                        to_do_task = accepted_tasks[i];
                        to_do_task_i = i;
                        has_task = true;
                        // ROS_INFO("%d",to_do_task.task_id);
                        break;
                    }
                }
            }
            if (has_task)
            {

                // print task info
                ss.str("");
                ss.clear();
                ss << "id:" << to_do_task.task_id << ", pos:("
                   << to_do_task.EnterPoint.way_point_pos.x << "," << to_do_task.EnterPoint.way_point_pos.y << "," << to_do_task.EnterPoint.way_point_pos.z << ")";
                ROS_INFO("uav%d doing task! info: %s", uav_id, ss.str().c_str());
                // ROS_INFO("goal_point:%d ", to_do_task.EnterPoint.id);
                uav_in_way_point_start = way_point_list[uav_in_way_point];        // 当前位置
                uav_in_way_point_goal = way_point_list[to_do_task.EnterPoint.id]; // 目标位置
                ROS_INFO("start_point :%d ,end_point: %d", uav_in_way_point_start.id, uav_in_way_point_goal.id);
                AddIndirectWayPoint(uav_in_way_point_start, uav_in_way_point_goal);

                ROS_INFO("EnterPoint xyz :(%f,%f,%f) ,LeavePoint:(%f,%f,%f)",
                         to_do_task.EnterPoint.way_point_pos.x, to_do_task.EnterPoint.way_point_pos.y, to_do_task.EnterPoint.way_point_pos.z,
                         to_do_task.LeavePoint.way_point_pos.x, to_do_task.LeavePoint.way_point_pos.y, to_do_task.LeavePoint.way_point_pos.z);
                for (int j = 0; j < to_do_task.task_path.size(); j++)
                {
                    goals.push(to_do_task.task_path[j]);
                }
                work_state = 0;
            }
            else
            {
                // back to home  or land
                work_state = 3;
            }
            break;
        case 4: // fault
            if (fabs(position.z - land_height) < flight_distance_error)
            {
                ROS_WARN("[uav fault] uav%d is out of power!", uav_id);
            }
            else
            {
                ROS_WARN("[uav fault] uav%d is out of power! uav land", uav_id);
                take_off_land(land_height);
            }
            break;
        case 5:  // execute
            if (dist(position, to_do_task.LeavePoint.way_point_pos) < flight_distance_error && goals.empty()) // 执行完任务
            {
                ROS_INFO("uav%d arrive_task_%d LeavePoint !", uav_id, to_do_task.task_id);
                uav_in_way_point = to_do_task.LeavePoint.id;

                accepted_tasks[to_do_task_i].done = true;
                has_task = false;
                work_state = 2;
            }
            else // 未执行完任务
            {
                if (!goals.empty())
                {
                    geometry_msgs::Point32 tmp_point = goals.front();
                    // ROS_INFO("uav_%d,goal :tmp_point:(%f,%f,%f)",uav_id,tmp_point.x,tmp_point.y,tmp_point.z);
                    delta.x = tmp_point.x - position.x;
                    delta.y = tmp_point.y - position.y;
                    delta.z = tmp_point.z - position.z;
                    float size = dist(tmp_point, position);
                    if (size > velocity_size)
                    {
                        delta.x = delta.x / size * velocity_size;
                        delta.y = delta.y / size * velocity_size;
                        delta.z = delta.z / size * velocity_size;
                    }
                    vel_flight_control(delta.x, delta.y, delta.z);
                    if ((dist(tmp_point, position) < flight_distance_error))
                    {
                        // ROS_INFO("arrive tmp_point:(%f,%f,%f)",tmp_point.x,tmp_point.y,tmp_point.z);
                        goals.pop();
                    }
                }
                else
                {
                    ROS_WARN("uav %d path_planning goals are empty but not arrvie LeavePoint", uav_id);
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



void UAV::grid_cells_callback(const nav_msgs::GridCells::ConstPtr &msg)
{
    //     //astar.update_map(*msg);//需完善
    //     rrt.update_map(*msg);
}

void UAV::take_off_land(float h)
{
    // collision avoidance by change z
    for (int i = 0; i < uav_dist_xy.size(); i++)
        if (i != uav_id && uav_dist_xy[i] > 0 && uav_dist_xy[i] < safety_dist_xy && uav_z[i] < position.z && uav_z[i] + 1.5 >= h)
        { // 其他uav与当前id uav 的距离小于安全距离 水平小于safety_dist_xy  垂直小于1.5
            // ROS_WARN("take_off_land: %d uav and %d uav distance in xoy is %f, z is %f", id, i, uav_dist_xy[i], uav_z[i]);
            h = uav_z[i] + 1.5;
            if (h < default_height + 1.5) // 最高位于default_height + 1.5
                h = default_height + 1.5;
        }
    float delta = h - position.z;
    float size = fabs(delta);
    if (size > take_off_land_vel_size) // 计算一个时间内飞行的delta距离  最大速度只能是take_off_land_vel_size
        delta = delta / size * take_off_land_vel_size;
    vel_flight_control(0, 0, delta);
}

void UAV::vel_flight_control(float x, float y, float z)
{
    // consider self velocity
    geometry_msgs::Twist msg;
    msg.linear.x = x - velocity.x;
    msg.linear.y = y - velocity.y;
    msg.linear.z = z - velocity.z;
    flight_control_pub.publish(msg);
}
void UAV::accepted_task_callback(const uav_msgs::UAV_Tasks::ConstPtr &msg)
{
    ROS_INFO("accepted_task_callback...");
    if (msg->uav_id == uav_id)
    {
        for (int i = 0; i < msg->uav_tasks.size(); i++)
        {
            ROS_INFO("uav_%d has accepted the tasks: ", uav_id);
            accepted_tasks.push_back(msg->uav_tasks[i]);
            ROS_INFO("task: %d, point_id: %d ,pos:(%f,%f,%f)", msg->uav_tasks[i].task_id,
                     msg->uav_tasks[i].EnterPoint.id,
                     msg->uav_tasks[i].EnterPoint.way_point_pos.x,
                     msg->uav_tasks[i].EnterPoint.way_point_pos.y,
                     msg->uav_tasks[i].EnterPoint.way_point_pos.z);
        }
    }
}
//------------------------------------------------------------
void UAV::publish_uav_state()
{
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
    for (int i = 0; i < accepted.size(); i++)
        msg.accepted[i] = accepted[i];  
    msg.work_state = work_state;

    uav_state_pub.publish(msg);
}

void UAV::local_vel_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    velocity.x = msg->twist.linear.x;
    velocity.y = msg->twist.linear.y;
    velocity.z = msg->twist.linear.z;
}

void UAV::local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    position.x = msg->pose.position.x+var_x;
    position.y = msg->pose.position.y+var_y;
    position.z = msg->pose.position.z+var_z;
    //ROS_INFO(" uav:%d_msg->pose:%lf,%lf,%lf", uav_id,msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    //ROS_INFO(" uav:%d_position:%f,%f,%f", uav_id,position.x,position.y,position.z);
    // velocity.x = msg->twist.twist.linear.x;
    // velocity.y = msg->twist.twist.linear.y;
    // velocity.z = msg->twist.twist.linear.z;
    // report uav id position and velocity
    static ros::Time s_t = ros::Time::now();
    ros::Duration e_t = ros::Time::now() - s_t;
    // Down sample
    if (e_t > ros::Duration(0.1))
    {
        // std::cout << elapsed_time.toSec() << std::endl; // debug
        s_t = ros::Time::now();
        publish_uav_idqp(); // 发布无人机的idqp信息
    }

    // report uav state
    static ros::Time start_t = ros::Time::now();
    ros::Duration elapsed_time = ros::Time::now() - start_t;
    // Down sample
    if (elapsed_time > ros::Duration(state_pub_interval)) // 每间隔1s发布(更新)无人机状态
    {
        // std::cout << elapsed_time.toSec() << std::endl; // debug
        start_t = ros::Time::now();
        // ROS_INFO("publish all uav state...");
        publish_all_uav_state();
    }
    
 
    // if(uav_state_pub.getNumSubscribers()>=1)  // 实时规划使用
    // {
    //     //is_uav_state_pub = true;
    //     ROS_INFO("publish uav %d state...",uav_id);
    //     publish_uav_state();
    // }

    if (is_uav_state_pub == false && uav_state_pub.getNumSubscribers() >= 1) // 预规划使用
    {
        is_uav_state_pub = true;
        ROS_INFO("publish uav %d state...", uav_id);
        publish_uav_state();
    }
    
}

void UAV::trajectoryVisualize()
{
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = position.x;
    this_pose_stamped.pose.position.y = position.y;
    this_pose_stamped.pose.position.z = position.z;

    // geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(1);
    this_pose_stamped.pose.orientation.x = 0;
    this_pose_stamped.pose.orientation.y = 0;
    this_pose_stamped.pose.orientation.z = 0;
    this_pose_stamped.pose.orientation.w = 1;

    this_pose_stamped.header.stamp = ros::Time::now();

    this_pose_stamped.header.frame_id = "my_frame";

    path_.poses.push_back(this_pose_stamped);
    path_to_rviz_pub_.publish(path_);
}

void UAV::ReadWayPoint()
{

    // read_way_point
    way_point_list.resize(0);
    between_way_point_height.resize(0);
    ifstream way_point_file;
    way_point_file.open(way_point_directory_, ios::in);
    if (!way_point_file)
    {
        cout << "open way_point file for read error\n"
             << endl;
    }
    way_point tmp_way_point;
    float tmp_way_point_height;
    while (way_point_file >> tmp_way_point.id >> tmp_way_point.x >> tmp_way_point.y >> tmp_way_point.z >> tmp_way_point_height)
    {
        way_point_list.push_back(tmp_way_point);
        between_way_point_height.push_back(tmp_way_point_height);
    }
    // show
    //  for(int i=0;i<=20;i++)
    //  {
    //      ROS_INFO("%d ,%f ,%f ,%f",way_point_list[i].id,way_point_list[i].x,way_point_list[i].y,way_point_list[i].z);
    //      ROS_INFO("%f",between_way_point_height[i]);
    //  }
}
bool UAV::AddIndirectWayPoint(const way_point &start, const way_point &goal)
{
    way_point tmp_point;
    way_point current_point;
    if (start.id < goal.id) // 从小的往大的飞  高度采用id小的
    {
        for (int i = start.id; i < goal.id; i++)
        {
            ROS_INFO("AddIndirectWayPoint: %d", i);
            // indirect_way_point.push(way_point_list[i]); // 县加入当前节点

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
    }
    else if (start.id > goal.id) // 从大往小飞，点采用小的
    {
        // ROS_INFO("else: start: %d goal:%d",start.id,goal.id);
        for (int j = start.id; j > goal.id; j--)
        {
            ROS_INFO("AddIndirectWayPoint: %d", j);
            // indirect_way_point.push(way_point_list[j]);

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
    }
    return false;
}

// bool UAV::enable_motors()
// {
//     hector_uav_msgs::EnableMotors srv;
//     srv.request.enable = true;
//     if (enable_motors_client.call(srv))
//     {
//         ROS_INFO("uav%d enable_motors succeed", uav_id);
//         return srv.response.success;
//     }
//     else
//     {
//         ROS_ERROR("uav%d enable_motors failed", uav_id);
//         return false;
//     }
// }

void UAV::publish_all_uav_state()
{
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
    // for(int i=0;i<bid_task_list.size();i++)
    //     msg.bid_task_list[i] = bid_task_list[i];
    // for(int i=0;i<bid_task_price.size();i++)
    //     msg.bid_task_price[i] = bid_task_price[i];
    msg.work_state = work_state;

    all_uav_state_pub.publish(msg);
}
void UAV::publish_uav_idqp()
{
    uav_msgs::UAV_IDQP msg;
    msg.uav_id = uav_id;
    msg.position = position;
    msg.velocity = velocity;
    idqp_pub.publish(msg);
}
void UAV::idqp_callback(const uav_msgs::UAV_IDQP::ConstPtr &msg)
{
    if (msg->uav_id != uav_id)
    {
        if (msg->uav_id >= uav_dist.size() || msg->uav_id >= uav_z.size() || msg->uav_id >= uav_dist_xy.size())
        {
            uav_dist.resize(msg->uav_id + 1);
            uav_dist_xy.resize(msg->uav_id + 1);
            uav_z.resize(msg->uav_id + 1);
        }
        uav_z[msg->uav_id] = msg->position.z;
        uav_dist[msg->uav_id] = dist(position, msg->position);
        uav_dist_xy[msg->uav_id] = dist_xy(position, msg->position);
    }
}
// void UAV::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//     position.x = msg->pose.pose.position.x;
//     position.y = msg->pose.pose.position.y;
//     position.z = msg->pose.pose.position.z;

//     velocity.x = msg->twist.twist.linear.x;
//     velocity.y = msg->twist.twist.linear.y;
//     velocity.z = msg->twist.twist.linear.z;
//     // report uav id position and velocity
//     static ros::Time s_t = ros::Time::now();
//     ros::Duration e_t = ros::Time::now() - s_t;
//     // Down sample
//     if (e_t > ros::Duration(0.1))
//     {
//         // std::cout << elapsed_time.toSec() << std::endl; // debug
//         s_t = ros::Time::now();
//         publish_uav_idqp();   //发布无人机的idqp信息
//     }

//     // report uav state
//     static ros::Time start_t = ros::Time::now();
//     ros::Duration elapsed_time = ros::Time::now() - start_t;
//     // Down sample
//     if (elapsed_time > ros::Duration(state_pub_interval))  // 每间隔0.1发布(更新)无人机状态
//     {
//         // std::cout << elapsed_time.toSec() << std::endl; // debug
//         start_t = ros::Time::now();
//         //ROS_INFO("publish all uav state...");
//         publish_all_uav_state();
//     }
//     // if(uav_state_pub.getNumSubscribers()>=1)  // 实时规划使用
//     // {
//     //     //is_uav_state_pub = true;
//     //     ROS_INFO("publish uav %d state...",uav_id);
//     //     publish_uav_state();
//     // }
//     if(is_uav_state_pub==false && uav_state_pub.getNumSubscribers()>=1)  //预规划使用
//     {
//         is_uav_state_pub = true;
//         ROS_INFO("publish uav %d state...",uav_id);
//         publish_uav_state();
//     }

// }
