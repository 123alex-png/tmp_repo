#include <ros/ros.h>

#include <QApplication>
#include <QTextCodec>

#include "mainwindow.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "qtcontrol", 0);

  QApplication a(argc, argv);
  MainWindow w;  // 创建主窗体的任务规划
  w.setWindowTitle("无人机群仿真巡检软件v1.0");
  w.show();  // 窗体显示

  return a.exec();
}
