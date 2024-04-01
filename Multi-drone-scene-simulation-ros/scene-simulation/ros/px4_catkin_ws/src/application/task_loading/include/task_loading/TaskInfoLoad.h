#ifndef __TASK_INFO_LOAD_H_
#define __TASK_INFO_LOAD_H_

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <random>
#include <ctime>
#include <time.h>
#include <string>
#include <stdbool.h>
#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

#include <sstream>
#include <tinyxml.h>
#include "uav_msgs/UAV_State.h"
#include "uav_msgs/TASK_State.h"
#include "uav_msgs/Way_Point.h"
using namespace std;
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/GetWorldProperties.h"
#include "gazebo_msgs/DeleteModel.h"
#include "geometry_msgs/Pose.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

namespace task_info_load
{
    class TaskInfoLoad
    {
        // typedef struct site{   //三维坐标
        //     float x;
        //     float y;
        //     float z;
        // }site;
        // typedef struct way_point{
        //     int id;
        //     float x;
        //     float y;
        //     float z;
        // }way_point;

    
    private:
        // int task_id = 0 ;   // 从0开始生成
        // int task_num = 20;      //生成任务的个数
        // int site_num = 20;     // 任务的位置
        // int barrier_num;    //生成障碍的数量？ 用不到
        // int min_sleep_time;  // 相邻两个任务生成的最小间隔时间 用不到
        // int max_sleep_time;  // 相邻两个任务生成的最大间隔时间 用不到
        // float min_pos_x = 0;    // x轴范围
        // float max_pos_x = 10000;
        // float min_pos_y = 0;     //y轴范围
        // float max_pos_y = 10000;
        // float min_pos_z = 0.5;   //z轴范围
        // float max_pos_z = 100;

        // int min_num = 0;   // 任务位置数量最小值
        // int max_num = 19;   // 任务位置数量最大值
        //u_int32_t task_cnt = 0;
        //float min_task_ditance = 300;  //生成的两个任务之间的最小距离
        //bool task_info_update = false;
        //vector<site> sites_v_; 
        //vector< vector < string > > task_list_;
        //vector<way_point> way_point_list; 
        //way_point tmp_way_point;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Publisher task_state_pub;
        //ros::Publisher task_cells_pub;   // 显示为box
        // ros::Publisher Tstart_cells_pub;   后期改动作用
        // ros::Publisher Tend_cells_pub;
        //ros::Publisher barrier_cells_pub;
        //ros::Publisher fixed_marker_array_pub;
        //ros::Publisher dynamic_marker_array_pub;

        //ros::ServiceClient spawn_model_client_;
        //ros::ServiceClient delete_model_client_;

        //string way_point_directory_;
        //string sites_directory_;
        string task_directory_;
        //string box_directory_;
        //string box_red_directory_;
        //string box_yel_directory_;
        //ostringstream stream_box_;
        //ostringstream stream_box_r_;
        //ostringstream stream_box_y_;

        uav_msgs::TASK_State task_state_msg_;

        //visualization_msgs::MarkerArray fixed_markers_;
        //visualization_msgs::MarkerArray dynamic_markers_;




    public:
        TaskInfoLoad();
        ~TaskInfoLoad();

       
        //void generateTaskInfo();
        //int generateSites();
        //template<typename T, typename M> 
        //void generateRandomDigit(const T &min, const T &max, M &random);
        //bool isNear(const site &current, const site &before, const float &gap);
        


       
        // int generateBarrierMap(const float &min_pos_x, const float &max_pos_x,
        //                     const float &min_pos_y, const float &max_pos_y,
        //                     const float &min_pos_z, const float &max_pos_z,
        //                     const int &barrier_num);

 
        //void generateSignsInRviz(const vector<site> &site_v);
        //void generateSignsInRviz(const vector<way_point> &way_point_list);


        //void showFixedCharactorInRviz(const vector<site> &site_v, visualization_msgs::MarkerArray &markerArray);

 
        //void showDynamicCharactorInRviz(const std::vector<site> &site_v, visualization_msgs::MarkerArray &markerArray);


        //void spawnModel(const string &type, const u_int64_t &task_id, const site &pos);


        //void removeModel(const string &type, const u_int64_t &task_id);


        bool readFiles();
        
    };
    void Management_callBack(const std_msgs::String& msg);

}

#endif
