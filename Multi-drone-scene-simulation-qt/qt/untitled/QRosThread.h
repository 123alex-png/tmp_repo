#ifndef QROSTHREAD_H
#define QROSTHREAD_H
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <uav_msgs/TASK_State.h>
#include <uav_msgs/Task_List.h>
#include <uav_msgs/UAV_State.h>

#include <QThread>

#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"

class QRosThread : public QThread {
  Q_OBJECT

 public:
  ros::NodeHandle nh;

  QRosThread();
  ~QRosThread();
  void all_uav_state_callback(const uav_msgs::UAV_State::ConstPtr &msg);
  void all_task_state_callback(const uav_msgs::Task_List::ConstPtr &msg);
  void Image_Callback_0(const sensor_msgs::CompressedImage::ConstPtr &msg);
  void Image_Callback_1(const sensor_msgs::CompressedImage::ConstPtr &msg);
  void Image_Callback_2(const sensor_msgs::CompressedImage::ConstPtr &msg);
  void Image_Callback_3(const sensor_msgs::CompressedImage::ConstPtr &msg);
  void Image_Callback_4(const sensor_msgs::CompressedImage::ConstPtr &msg);

 signals:
  void returnUavResult(const uav_msgs::UAV_State::ConstPtr &msg);
  void returnTaskResult(const uav_msgs::Task_List::ConstPtr &msg);
  void returnResult_image(const std::string name);

 protected:
  void run();
};

#endif  // QROSTHREAD_H
