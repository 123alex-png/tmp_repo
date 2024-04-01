#include "task_management/task_management.h"
#include "std_msgs/String.h"
using namespace std;


namespace task
{
    
    void TaskState_callBack(const uav_msgs::TASK_State::ConstPtr& msg) //接收任务信息
    {   
        ROS_INFO("accept the task msg->req_uav_type:%s",msg->req_uav_type.c_str());
        uav_msgs::TASK_State task;
        
        task.task_id = msg->task_id;
        task.task_name = msg->task_name;
        task.task_type = msg->task_type;
        task.EnterPoint = msg->EnterPoint;
        task.LeavePoint = msg->LeavePoint;
        task.req_uav_type = msg->req_uav_type;
        task.req_payload_type = msg->req_payload_type;
        task.Ts = msg->Ts;
        task.Te = msg->Te;
        task.S_t = msg->S_t;
        task.E_t = msg->E_t;
        task.execute_time = msg->execute_time;
        task.R = msg->R;
        task.done = msg->done;
        task.allocated = msg->allocated;
        if(msg->task_path.size()!=0)
            task.task_path = msg->task_path;
        task.path_length = msg->path_length;
        // for(int i=0;i<=msg->allocated.size();i++)
        //     task.allocated.push_back(msg->allocated[i]);
        //task_path   path_length 不进行赋值  在路径规划后赋值

        ROS_INFO("id: %d , name:%s, type:%s, EnterPointpos:(%f,%f,%f),LeavePointpos:(%f,%f,%f),req_uav_type: %s",task.task_id,task.task_name.c_str(),
        task.task_type.c_str(),
        task.EnterPoint.way_point_pos.x,task.EnterPoint.way_point_pos.y,task.EnterPoint.way_point_pos.z,
        task.LeavePoint.way_point_pos.x,task.LeavePoint.way_point_pos.y,task.LeavePoint.way_point_pos.z,
        task.req_uav_type.c_str());
        
        all_tasks.push_back(task); 
    }


    void ShowTaskInfo()
    {   
        ROS_INFO("all_tasks.length:%ld",all_tasks.size());
        if(all_tasks.size()!=0)
        {
            for(int i=0;i<all_tasks.size();i++)
            {
                ROS_INFO("id: %d , name:%s, EnterPointpos:(%f,%f,%f),LeavePointpos:(%f,%f,%f),req_uav_type: %s",all_tasks[i].task_id,all_tasks[i].task_name.c_str(),
                all_tasks[i].EnterPoint.way_point_pos.x,all_tasks[i].EnterPoint.way_point_pos.y,all_tasks[i].EnterPoint.way_point_pos.z,
                all_tasks[i].LeavePoint.way_point_pos.x,all_tasks[i].LeavePoint.way_point_pos.y,all_tasks[i].LeavePoint.way_point_pos.z,
                all_tasks[i].req_uav_type.c_str());
            }
        }
        
    }
    void AllTaskQt_Publish()
    {
        if(all_tasks.size()!=0)
        {
            uav_msgs::Task_List task_list_qt;
            task_list_qt.task_list=all_tasks;
            all_tasks_qt_pub.publish(task_list_qt);
        }
    }


    // void TaskPath_callBack(const uav_msgs::Task_Path::ConstPtr& msg)
    // {
    //     for(int i=0;i<all_tasks.size();i++)
    //     {
    //         if(msg->task_id == all_tasks[i].task_id)
    //         {
    //             all_tasks[i].task_path = msg->destination;
    //             all_tasks[i].path_length = msg->length;
    //         }
    //     }
    // }
    // void LastTask_callBack(const uav_msgs::Last_Task::ConstPtr& msg)
    // {
    //     if (msg->is_last_task == true)
    //     {   
            
    //         task::task_list_msg.task_list = task::all_tasks;
    //         all_task_list_pub.publish(task::task_list_msg);
    //         task::ShowTaskInfo();
    //     }
    // }
    void LoadLastTask_callBack(const uav_msgs::Last_Task::ConstPtr& msg)
    {
        if(msg->is_last_task==true)
        {
            task::task_list_msg.task_list = task::all_tasks;
            all_task_list_pub.publish(task::task_list_msg);
        }
    }
    void AllocatedTaskState_callback(const uav_msgs::TASK_State::ConstPtr& msg)
    {
        for(int i=0;i<all_tasks.size();i++)
        {
            if(msg->task_id==all_tasks[i].task_id)
            {
                //更新任务分配状态和执行状态
                all_tasks[i].allocated = msg->allocated;
                all_tasks[i].done = msg->done;
                all_tasks[i].S_t = msg->S_t;
                all_tasks[i].E_t = msg->E_t;

            }
        }
        
    }
}
int main(int argc,char **argv)
{

    ros::init(argc, argv, "task_management");
    ros::NodeHandle nh;   // 全局命名空间
    ros::NodeHandle nh_private("~");   // 局部命名空间
    
    ros::Publisher management = nh.advertise<std_msgs::String>("/start_task_info_load",100,true); // 增加
    std_msgs::String manage;
    manage.data = "management is ok";
    management.publish(manage);
    
    //task::last_task_sub = nh.subscribe("/last_cal_path",10,&task::LastTask_callBack);
    task::task_state_sub = nh.subscribe("/task_state",20,&task::TaskState_callBack);  // 接收task_loading的任务
    //task::task_path_sub = nh.subscribe("/task_path",10,&task::TaskPath_callBack);     // 接收每个任务对应的巡检路径及路径长度
    task::allocated_task_state_sub = nh.subscribe("/allocated_task_state",20,&task::AllocatedTaskState_callback);  // 实时更新任务状态
    
    task::all_task_list_pub = nh.advertise<uav_msgs::Task_List>("/all_task_list",5,true); // 发布所有任务
    
    task::all_tasks_allocate_pub = nh.advertise<uav_msgs::Task_List>("/all_task_allocate",5,true); // 发布到allocate分配
    task::all_tasks_qt_pub = nh.advertise<uav_msgs::Task_List>("/all_task_qt",5,true); // 发布到qt显示
    
    bool tasks_allocate_pub = false;
    bool showd = false;
    // 应改动 taskgen完成再view point plan

    while (ros::ok())
    {   
        ros::spinOnce();   // 调用回调函数task::TaskState_callBack
        task::AllTaskQt_Publish();
        if(task::all_tasks_allocate_pub.getNumSubscribers()>0 && tasks_allocate_pub==false)
        {
            uav_msgs::Task_List task_list_allocate;
            task_list_allocate.task_list=task::all_tasks;
            task::all_tasks_allocate_pub.publish(task_list_allocate);
            tasks_allocate_pub=true;
        }

        sleep(1);    
    }
    
    return 0;
}



