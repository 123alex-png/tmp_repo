#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
cv::Mat imgCallback;
static void ImageCallback(const sensor_msgs::CompressedImageConstPtr &msg, string name)
{
    try
    {
      cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
      imgCallback = cv_ptr_compressed->image;
      imwrite("/home/cs504/px4_catkin_ws/image/" + name +"/image_save.jpg",imgCallback);
      cv::waitKey(5);
    }
    catch (cv_bridge::Exception& e)
    {
      //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "CompressedImage");
  ros::NodeHandle nh;
  std::string image_topic_0 = "/p450_0/stereo/left/image_raw/compressed";
  std::string image_topic_1 = "/p450_1/stereo/left/image_raw/compressed";
  std::string image_topic_2 = "/p450_2/stereo/left/image_raw/compressed";
  std::string image_topic_3 = "/p450_3/stereo/left/image_raw/compressed";
  std::string image_topic_4 = "/p450_4/stereo/left/image_raw/compressed";
  ros::Subscriber image_sub_0 = nh.subscribe<sensor_msgs::CompressedImage>(image_topic_0,10,boost::bind(&ImageCallback, _1, "uav0"));
  ros::Subscriber image_sub_1 = nh.subscribe<sensor_msgs::CompressedImage>(image_topic_1,10,boost::bind(&ImageCallback, _1, "uav1"));
  ros::Subscriber image_sub_2 = nh.subscribe<sensor_msgs::CompressedImage>(image_topic_2,10,boost::bind(&ImageCallback, _1, "uav2"));
  ros::Subscriber image_sub_3 = nh.subscribe<sensor_msgs::CompressedImage>(image_topic_3,10,boost::bind(&ImageCallback, _1, "uav3"));
  ros::Subscriber image_sub_4 = nh.subscribe<sensor_msgs::CompressedImage>(image_topic_4,10,boost::bind(&ImageCallback, _1, "uav4"));
 
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ROS_INFO("ROS OK!");
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
