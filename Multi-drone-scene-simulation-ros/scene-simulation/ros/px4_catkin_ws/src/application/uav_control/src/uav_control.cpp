
#include "uav_control/uav_control.h"

int main(int argc, char  **argv)
{
	//ros::init(argc,argv,"px4_uav");
    ros::init(argc, argv, ros::this_node::getName().c_str());
    ROS_INFO("%s", ros::this_node::getName().c_str());

    UAV uav;

    uav.run();

    return 0;
}
