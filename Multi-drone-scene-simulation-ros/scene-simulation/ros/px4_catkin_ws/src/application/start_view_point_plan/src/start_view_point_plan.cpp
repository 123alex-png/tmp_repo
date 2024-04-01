#include "std_msgs/String.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <uav_msgs/UAV_State.h>
#include <nav_msgs/GridCells.h>
using namespace std;

int main(int argc, char* argv[])
{   // 节点初始化
    ros::init(argc, argv, "start_view_point_plan");
    ros::NodeHandle nh; 
    ros::Publisher start_view_plan_pub = nh.advertise<uav_msgs::UAV_State>("/start_view_point_plan",10);
    uav_msgs::UAV_State uav;
    for (int i=0;i<5;i++ ) {
        uav.uav_id = i;
        start_view_plan_pub.publish(uav);
    }
    ros::spin();
	
    //ros::spin();  //调用回调函数
    return 0;
}
