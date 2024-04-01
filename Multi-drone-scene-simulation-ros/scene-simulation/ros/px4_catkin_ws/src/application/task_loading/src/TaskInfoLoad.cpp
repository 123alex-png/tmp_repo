#include "task_loading/TaskInfoLoad.h"
#include "std_msgs/String.h"
using namespace std;

namespace task_info_load
{
    TaskInfoLoad::TaskInfoLoad(): 
        nh_(ros::NodeHandle()),
        nh_private_(ros::NodeHandle("~"))
    {
        // nh_private_.param<int> ("task_num",task_num ,20);
        // nh_private_.param<float> ("min_pos_x",min_pos_x ,0);
        // nh_private_.param<float> ("max_pos_x",max_pos_x ,100);
        // nh_private_.param<float> ("min_pos_y",min_pos_y ,0);
        // nh_private_.param<float> ("max_pos_y",max_pos_y ,100);
        // nh_private_.param<float> ("min_pos_z",min_pos_z ,0.5);
        // nh_private_.param<float> ("max_pos_z",max_pos_z ,10);
        // nh_private_.param<float> ("min_task_ditance",min_task_ditance ,300);
        //nh_private_.param<string>("way_point_directory", way_point_directory_, "/home/zrh/px4_catkin_ws1019/src/application/uav_control/data/waypoint.txt");
        //nh_private_.param<string>("sites_directory", sites_directory_, "/home/zrh/px4_catkin_ws1019/src/application/task_generation/data/sites.txt");
        
        nh_private_.param<string>("task_directory", task_directory_, "/home/cs504/px4_catkin_ws/src/application/task_loading/data/task_info_.txt");  //保存生成的任务信息
        
        //nh_private_.param<string>("box_directory", box_directory_, "/home/zrh/px4_catkin_ws1019/src/application/uav_control/urdf/box.urdf");
        //nh_private_.param<string>("box_red_directory", box_red_directory_, "/home/zrh/px4_catkin_ws1019/src/application/uav_control/urdf/box_red.urdf");
        //nh_private_.param<string>("box_yel_directory", box_yel_directory_, "/home/zrh/px4_catkin_ws1019/src/application/uav_control/urdf/box_yel.urdf");

        task_state_pub = nh_.advertise<uav_msgs::TASK_State>("/task_state", 20, true);  // msg_Callback(const uav_msgs::TASK_State::ConstPtr& msg)
        //task_cells_pub = nh_.advertise<nav_msgs::GridCells>("/task_cell", 10, true); 
        //barrier_cells_pub = nh_.advertise<nav_msgs::GridCells>("/barrier_cell", 10, true); 
        //fixed_marker_array_pub = nh_.advertise<visualization_msgs::MarkerArray>("/fixed_makers", 10, true);


        // ros::service::waitForService("gazebo/spawn_urdf_model");
        // ros::service::waitForService("gazebo/delete_model");
        // spawn_model_client_ = nh_.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_urdf_model");
        // delete_model_client_ = nh_.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");

        // load urdf file  /read urdf / gazebo model xml from file  
        // TiXmlDocument xml_box(box_directory_);
        // xml_box.LoadFile();    
        // stream_box_ << xml_box;

        // TiXmlDocument xml_box_r(box_red_directory_);
        // xml_box_r.LoadFile();    
        // stream_box_r_ << xml_box_r;
    
        // TiXmlDocument xml_box_y(box_yel_directory_);
        // xml_box_y.LoadFile();    
        // stream_box_y_ << xml_box_y;

        // task_info_update = true; 
        // task_cnt = 0;

        //read_way_point
        // way_point_list.resize(0);
        // ifstream way_point_file;
        // way_point_file.open(way_point_directory_,ios::in);
        // if(!way_point_file)
        // {
        //     cout << "open way_point file for read error\n" <<endl;
        // }
        // float tmp_way_point_height;
        // while(way_point_file >> tmp_way_point.id >>tmp_way_point.x>>tmp_way_point.y>>tmp_way_point.z>>tmp_way_point_height)
        // {
        //     way_point_list.push_back(tmp_way_point);
        //     site tmp_waytosite;
        //     tmp_waytosite.x = tmp_way_point.x;
        //     tmp_waytosite.y = tmp_way_point.y;
        //     tmp_waytosite.z = tmp_way_point.z;
        //     spawnModel("way_point_",tmp_way_point.id,tmp_waytosite);   // 生成模型
        // }

        //generateSignsInRviz(way_point_list);
        //showFixedCharactorInRviz();
        //ros::Publisher gen = nh_.advertise<std_msgs::String>("/start_cal_path",100,true); // 增加


        ROS_INFO("Task is loading---");
        bool read = false;
        while(ros::ok())
        {   
            if(read != true)
            {            
                readFiles();
                read = true;
                ROS_INFO("Task loading finished!! ");
                // std_msgs::String gen_msg;
                // gen_msg.data = "task_loading is ok";
                // gen.publish(gen_msg);
                
            }
            
            sleep(1);

            ros::spinOnce();
        }
    }
    bool TaskInfoLoad::readFiles()
    {   

        FILE *input = fopen(task_directory_.c_str(),"r"); 
        if(input == NULL)
        {
            std::cout << "open file for read error\n" <<std::endl;
        }
        else
        {   
            while(!feof(input))
            {
                uav_msgs::TASK_State task_msg;
                string task_name;
                string task_type;
                string req_uav_type; 
                string req_payload_type;
                int task_R;  //任务收益
                std::cout << "open file success\n" <<std::endl;
                fscanf(input,"%hd %s %s %hd %f %f %f %hd %f %f %f %s %s %hd %hd %hd %hd\n", 
                    &task_msg.task_id,
                    task_name.c_str(),
                    task_type.c_str(),
                    &task_msg.EnterPoint.id,
                    &task_msg.EnterPoint.way_point_pos.x, 
                    &task_msg.EnterPoint.way_point_pos.y,
                    &task_msg.EnterPoint.way_point_pos.z,
                    &task_msg.LeavePoint.id,
                    &task_msg.LeavePoint.way_point_pos.x, 
                    &task_msg.LeavePoint.way_point_pos.y,
                    &task_msg.LeavePoint.way_point_pos.z,
                    req_uav_type.c_str(),
                    req_payload_type.c_str(),
                    &task_msg.Ts,
                    &task_msg.Te,
                    &task_msg.execute_time,
                    &task_msg.R);
                task_msg.done = 0;    // 0表示未分配  1表示已被分配  2 表示被分配但无满足无人机 3表示正在被执行  4表示已经被执行
                task_msg.allocated = {-1};
                task_msg.task_name = task_name.c_str();
                task_msg.task_type = task_type.c_str();
                task_msg.req_uav_type = req_uav_type.c_str();
                task_msg.req_payload_type = req_payload_type.c_str();
                task_msg.S_t = 0;
                task_msg.E_t = 0;

                task_msg.path_length=0;

                ROS_INFO("----Generate Task:id:%d",task_msg.task_id);
                ROS_INFO("----Generate Task:task_name:%s",task_msg.task_name.c_str());
                ROS_INFO("----Generate Task:type:%s",task_msg.task_type.c_str());
                ROS_INFO("----Generate Task:EnterPoint:%d",task_msg.EnterPoint.id);
                ROS_INFO("----Generate Task:enterx:%f",task_msg.EnterPoint.way_point_pos.x);
                ROS_INFO("----Generate Task:entery:%f",task_msg.EnterPoint.way_point_pos.y);
                ROS_INFO("----Generate Task:enterz:%f",task_msg.EnterPoint.way_point_pos.z);
                ROS_INFO("----Generate Task:LeavePoint:%d",task_msg.LeavePoint.id);
                ROS_INFO("----Generate Task:leavex:%f",task_msg.LeavePoint.way_point_pos.x);
                ROS_INFO("----Generate Task:leavey:%f",task_msg.LeavePoint.way_point_pos.y);
                ROS_INFO("----Generate Task:leavez:%f",task_msg.LeavePoint.way_point_pos.z);
                ROS_INFO("----Generate Task:req_uav_type:%s",task_msg.req_uav_type.c_str());
                ROS_INFO("----Generate Task:req_payload_type:%s",task_msg.req_payload_type.c_str());
                ROS_INFO("----Generate Task:Ts:%hd",task_msg.Ts);
                ROS_INFO("----Generate Task:Te:%hd",task_msg.Te);
                ROS_INFO("----Generate Task:execute_time:%d",task_msg.execute_time);
                ROS_INFO("----Generate Task:done:%d",task_msg.done);
                ROS_INFO("----Generate Task:allocated:%d",task_msg.allocated[0]);
                ROS_INFO("----Generate Task:R:%d",task_msg.R);
                task_state_pub.publish(task_msg);
                sleep(1);
            }
            fclose(input);
        }
        return true;
    }
    // template<typename T, typename M> 
    // void TaskInfoGen::generateRandomDigit(const T &min_range, const T &max_range, M &random)
    // {
    //     random_device rd;
    //     default_random_engine e_random{rd()};
    //     if(is_same<T, int>::value)
    //     {
    //         uniform_int_distribution<int> u_random(min_range, max_range);
    //         random = u_random(e_random);
    //     }
    //     else
    //     {
    //         uniform_real_distribution<float> u_random(min_range, max_range);
    //         random = u_random(e_random);
    //     } 
    // }
       // void TaskInfoGen::generateTaskInfo()
    // {
    //     uav_msgs::TASK_State task_msg;
    //     way_point tmp_point; 
    //     int way_point_list_len = way_point_list.size();
    //     generateRandomDigit(0, way_point_list_len-1-1,tmp_point.id);
    //     task_msg.EnterPoint.id = tmp_point.id;
    //     task_msg.EnterPoint.way_point_pos.x = way_point_list[task_msg.EnterPoint.id].x;
    //     task_msg.EnterPoint.way_point_pos.y = way_point_list[task_msg.EnterPoint.id].y;
    //     task_msg.EnterPoint.way_point_pos.z = way_point_list[task_msg.EnterPoint.id].z;        
    //     task_msg.LeavePoint.id = task_msg.EnterPoint.id +1;
    //     task_msg.LeavePoint.way_point_pos.x = way_point_list[task_msg.LeavePoint.id].x;
    //     task_msg.LeavePoint.way_point_pos.y = way_point_list[task_msg.LeavePoint.id].y;
    //     task_msg.LeavePoint.way_point_pos.z = way_point_list[task_msg.LeavePoint.id].z;   
        

    //     task_msg.task_id = task_id++;
    //     task_msg.task_name =  "task_"+ to_string(task_id);
    //     generateRandomDigit(0, 2, task_msg.task_type);

    //     string task_type_name;
    //     if(task_msg.task_type == 0)
    //     {
    //         task_type_name = "bridge task";
    //     }
    //     else if(task_msg.task_type == 1)
    //     {
    //         task_type_name = "slope task";
    //     }
    //     else if(task_msg.task_type == 2)
    //     {
    //          task_type_name = "road task";
    //     }sleep(0.5);
    //     task_msg.req_uav_type = 0;  // 暂时固定为 上云台 above
    //     generateRandomDigit(20, 60, task_msg.execute_time);

    //     // 时间窗暂时不加
    //     task_msg.Ts = 0;
    //     task_msg.Te = 0;
    //     task_msg.done = false;
    //     task_msg.allocated = {21};
        
    //     ROS_INFO("Generate Task:id:%d,EnterPoint:%d,LeavePoint:%d,name:%s,type:%d,exe_time:%d",
    //     task_msg.task_id,task_msg.EnterPoint.id,task_msg.LeavePoint.id,task_msg.task_name.c_str(),task_msg.task_type,task_msg.execute_time);
    //     // publish
    //     task_state_pub.publish(task_msg);
    //     /* write file */
    //     FILE *output = fopen(task_directory_.c_str(),"a"); 
    //     setbuf(output,NULL);
    //     fprintf(output, "%d  %s  %d  %d %5f  %5f  %5f  %d %5f  %5f  %5f  %d  %d  %d  %d  %d  %d \n", 
    //             task_msg.task_id,task_msg.task_name.c_str(), task_msg.task_type,
    //             task_msg.EnterPoint.id,
    //             task_msg.EnterPoint.way_point_pos.x, task_msg.EnterPoint.way_point_pos.y,task_msg.EnterPoint.way_point_pos.z,
    //             task_msg.LeavePoint.id,
    //             task_msg.LeavePoint.way_point_pos.x, task_msg.LeavePoint.way_point_pos.y,task_msg.LeavePoint.way_point_pos.z,
    //             task_msg.req_uav_type, 
    //             task_msg.execute_time, task_msg.Ts,task_msg.Te, task_msg.done,task_msg.allocated[0]
    //             );
    //     fflush(output);

    //     /* read file */
    //     //readFiles();
    // }
    // void TaskInfoGen::generateSignsInRviz(const vector<way_point> &way_point_list)
    // {
    //     geometry_msgs::Point point; 

    //     nav_msgs::GridCells cells_task;
    //     cells_task.header.frame_id="my_frame";
    //     cells_task.cell_height = 5;
    //     cells_task.cell_width  = 5;
    //     cells_task.cells.resize(0);
    //     for(size_t i = 0; i < way_point_list.size(); i ++)
    //     {
    //         point.x = way_point_list[i].x;
    //         point.y = way_point_list[i].y;
    //         point.z = way_point_list[i].z;
    //         cells_task.cells.push_back(point);
    //     }
    //     task_cells_pub.publish(cells_task);
    // }
    // void TaskInfoGen::spawnModel(const string &type, const u_int64_t &id, const site &pos)
    // {
    //     gazebo_msgs::SpawnModel spawn_model;
    //     string model_name = type + to_string(id);
    //     spawn_model.request.model_name = model_name;
    //     if(type == "Task_")
    //         spawn_model.request.model_xml = stream_box_.str(); // load xml file
    //         //spawn_model.request.model_xml = stream_box_r_.str(); // load xml file  
    //     // else if(type == "D_")
    //     //     spawn_model.request.model_xml = stream_box_r_.str(); // load xml file
    //     // else if(type == "B_")
    //     //     spawn_model.request.model_xml = stream_box_y_.str(); // load xml file
    //     if(type == "way_point_")
    //        spawn_model.request.model_xml = stream_box_r_.str(); // load xml file 
    //     spawn_model.request.robot_namespace = "";
    //     geometry_msgs::Pose pose;
    //     pose.position.x = pos.x;
    //     pose.position.y = pos.y; 
    //     pose.position.z = 0;
    //     pose.orientation.w = 1.0; 
    //     pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
    //     spawn_model.request.initial_pose = pose;
    //     spawn_model.request.reference_frame = "";
    //     spawn_model_client_.call(spawn_model);
    // }

    // void TaskInfoGen::removeModel(const string &type, const u_int64_t &order_id)
    // {
    //     gazebo_msgs::DeleteModel delete_model;
    //     std::string model_name = type + to_string(order_id);
    //     delete_model.request.model_name = model_name;
    //     delete_model_client_.call(delete_model);
    // }

}

void Management_callBack(const std_msgs::String& msg)
{
    ROS_INFO("management");
    task_info_load::TaskInfoLoad* cont = new task_info_load::TaskInfoLoad();
    
    
}

int main(int argc, char* argv[])
{   // 节点初始化
    ros::init(argc, argv, "task_loading");
    ros::NodeHandle nh; 
    ros::Subscriber sub = nh.subscribe("/start_task_info_load",10,&Management_callBack);
    
    ros::spin();
	
    //ros::spin();  //调用回调函数
    return 0;
}
