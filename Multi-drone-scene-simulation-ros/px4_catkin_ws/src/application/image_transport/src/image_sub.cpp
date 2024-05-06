/*******************************************************************
 * File: image_sub.cpp
 * Author: lizeshan
 * Date: 2024-04-23
 * Description:
 *订阅对应无人机的相机节点，并向本地保存当前照片，仅支持p450的深度相机，其余需自行修改话题名
 *******************************************************************/

#include <cv_bridge/cv_bridge.h>

#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"

cv::Mat imgCallback;
static void ImageCallback(const sensor_msgs::CompressedImageConstPtr &msg,
                          std::string name) {
  cv_bridge::CvImagePtr cv_ptr_compressed =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  imgCallback = cv_ptr_compressed->image;
  std::string currentFilePath(__FILE__);

  std::experimental::filesystem::path currentPath(currentFilePath);
  std::experimental::filesystem::path parentPath = currentPath.parent_path();
  parentPath = parentPath.parent_path();
  std::string targetDirectory = parentPath.string();
  imwrite(targetDirectory + "/image/" + name + "/image_save.jpg", imgCallback);
  cv::waitKey(5);
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "CompressedImage");
  ros::NodeHandle nh;
  std::string image_topic_0 = "/p450_0/stereo/left/image_raw/compressed";
  std::string image_topic_1 = "/p450_1/stereo/left/image_raw/compressed";
  std::string image_topic_2 = "/p450_2/stereo/left/image_raw/compressed";
  std::string image_topic_3 = "/p450_3/stereo/left/image_raw/compressed";
  std::string image_topic_4 = "/p450_4/stereo/left/image_raw/compressed";
  ros::Subscriber image_sub_0 = nh.subscribe<sensor_msgs::CompressedImage>(
      image_topic_0, 10, boost::bind(&ImageCallback, _1, "uav0"));
  ros::Subscriber image_sub_1 = nh.subscribe<sensor_msgs::CompressedImage>(
      image_topic_1, 10, boost::bind(&ImageCallback, _1, "uav1"));
  ros::Subscriber image_sub_2 = nh.subscribe<sensor_msgs::CompressedImage>(
      image_topic_2, 10, boost::bind(&ImageCallback, _1, "uav2"));
  ros::Subscriber image_sub_3 = nh.subscribe<sensor_msgs::CompressedImage>(
      image_topic_3, 10, boost::bind(&ImageCallback, _1, "uav3"));
  ros::Subscriber image_sub_4 = nh.subscribe<sensor_msgs::CompressedImage>(
      image_topic_4, 10, boost::bind(&ImageCallback, _1, "uav4"));

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ROS_INFO("ROS OK!");
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
