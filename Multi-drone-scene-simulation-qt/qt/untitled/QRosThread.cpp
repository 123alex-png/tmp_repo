/*******************************************************************
 * File: QRosThread.cpp
 * Author: lizeshan zhangruiheng
 * Date: 2024-04-23
 * Description: 在Qt中订阅部分ros节点的消息
 *******************************************************************/
#include "QRosThread.h"

#include <geometry_msgs/Point32.h>

#include <vector>

#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"

QRosThread::QRosThread() {}

QRosThread::~QRosThread() {}

/**
 * @brief: 接收无人机的状态信息
 * @param msg：无人机当前状态
 */
void QRosThread::all_uav_state_callback(
    const uav_msgs::UAV_State::ConstPtr &msg) {
  emit QRosThread::returnUavResult(msg);
}

/**
 * @brief: 接收任务的信息
 * @param msg：任务信息
 */
void QRosThread::all_task_state_callback(
    const uav_msgs::Task_List::ConstPtr &msg) {
  emit QRosThread::returnTaskResult(msg);
}

/**
 * @brief: 接收无人机的相机信息
 * @param msg：无人机相机的当前图像
 */
void QRosThread::Image_Callback_0(
    const sensor_msgs::CompressedImage::ConstPtr &msg) {
  std::string name = "uav0";
  emit QRosThread::returnResult_image(name);
}

/**
 * @brief: 接收无人机的相机信息
 * @param msg：无人机相机的当前图像
 */
void QRosThread::Image_Callback_1(
    const sensor_msgs::CompressedImage::ConstPtr &msg) {
  std::string name = "uav1";
  emit QRosThread::returnResult_image(name);
}

/**
 * @brief: 接收无人机的相机信息
 * @param msg：无人机相机的当前图像
 */
void QRosThread::Image_Callback_2(
    const sensor_msgs::CompressedImage::ConstPtr &msg) {
  std::string name = "uav2";
  emit QRosThread::returnResult_image(name);
}

/**
 * @brief: 接收无人机的相机信息
 * @param msg：无人机相机的当前图像
 */
void QRosThread::Image_Callback_3(
    const sensor_msgs::CompressedImage::ConstPtr &msg) {
  std::string name = "uav3";
  emit QRosThread::returnResult_image(name);
}

/**
 * @brief: 接收无人机的相机信息
 * @param msg：无人机相机的当前图像
 */
void QRosThread::Image_Callback_4(
    const sensor_msgs::CompressedImage::ConstPtr &msg) {
  std::string name = "uav4";
  emit QRosThread::returnResult_image(name);
}
void QRosThread::run() {
  ros::Subscriber all_uav_state_sub = nh.subscribe<uav_msgs::UAV_State>(
      "/all_uav_state", 10, &QRosThread::all_uav_state_callback, this);
  ros::Subscriber all_task_state_sub = nh.subscribe<uav_msgs::Task_List>(
      "/all_task_qt", 10, &QRosThread::all_task_state_callback, this);

  ros::Subscriber camera_image0 = nh.subscribe<sensor_msgs::CompressedImage>(
      "/p450_0/stereo/left/image_raw/compressed", 10,
      &QRosThread::Image_Callback_0, this);
  ros::Subscriber camera_image1 = nh.subscribe<sensor_msgs::CompressedImage>(
      "/p450_1/stereo/left/image_raw/compressed", 10,
      &QRosThread::Image_Callback_1, this);

  ros::spin();
}
