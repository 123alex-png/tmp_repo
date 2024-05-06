/*******************************************************************
 * File: mainwindow.h
 * Author: lizeshan zhangruiheng
 * Date: 2024-04-23
 * Description: QT主界面，展示相关数据并辅助用户运行项目
 *******************************************************************/
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <geometry_msgs/Point32.h>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
#include <uav_msgs/TASK_State.h>
#include <uav_msgs/Task_List.h>
#include <uav_msgs/UAV_State.h>
#include <uav_msgs/Way_Point.h>

#include <QAbstractButton>
#include <QMainWindow>
#include <QMutex>
#include <QMutexLocker>
#include <QPixmap>
#include <QVector>
#include <QWebEngineView>
#include <QtCharts>
#include <thread>

#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"

QT_BEGIN_NAMESPACE
class QRosThread;
namespace Ui {
class MainWindow;

}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

 signals:
  void changePage(int index);  // 切换界面

 private slots:
  void QLeftWidgetInit();         // 左侧导航栏
  void QLeftWidgetInitConnect();  // 左侧导航栏内按钮绑定
  void LeftBtnClicked();
  void on_envFileButton_clicked();  // 仿真环境导入栏  选择launch文件
  void on_envStartButton_clicked();  // 仿真环境导入栏  启动launch文件
  void on_globalTaskFileButton_clicked();  // 任务导入栏  点击选择任务文件
  void on_subTaskFileButton_clicked();  // 任务导入栏  点击选择子任务文件
  void on_globalTaskLoadingButton_clicked();  // 任务导入栏 点击导入任务文件绑定
  void on_obstaclesFileButton_clicked();  // 场景约束导入栏  选择障碍物文件
  void on_wayPointFileButton_clicked();  // 场景约束导入栏  选择路径点文件
  void on_viewpointPlanButton_clicked();
  void on_startUavsButton_clicked();
  void on_startTaskPlanButton_clicked();
  void on_chooseTaskBox_currentIndexChanged(const QString &arg1);
  /**
   * @brief: 读取文件的行数
   * @param fileName: 文件名
   * @param data: 文件信息的id列表
   */
  void ReadFirstNum(std::string fileName, std::vector<int> &data);
  bool IsFileExist(QString fileName);
  /**
   * @brief: 显示无人机信息
   * @param msg: 无人机节点发布的自身信息
   */
  void displayUavState(const uav_msgs::UAV_State::ConstPtr &msg);
  /**
   * @brief: 显示任务的信息
   * @param msg: 任务管理节点发布的任务信息
   */
  void displayTaskState(const uav_msgs::Task_List::ConstPtr &msg);
  /**
   * @brief: 显示无人机相机的实时画面
   * @param name: 无人机节点的名字
   */
  void displayImage(std::string name);
  void on_startTaskExecuteButton_clicked();

 private:
  Ui::MainWindow *ui;
  QList<QAbstractButton *> btnsMain;
  QString strBtnStyle;
  QWebEngineView *view;
  QWebEngineView *webview;

  QString envFilename;
  QString subTaskFilename;
  QString obstaclesFilename;
  QString globalTaskFilename;
  QString wayPointFilename;

  std::set<int> uavId;
  ros::Publisher start_view_plan_pub;
  ros::Publisher start_execute_task_pub;
  bool envStartState = false;
  bool viewpointPlanState = false;
  bool startUavsState = false;
  bool startTaskPlanState = false;

  QRosThread *qrosthread;

  ros::NodeHandle nh;

  QScatterSeries *seriesOfUAV;
  QLineSeries *seriesOfWaypoint;
  QScatterSeries *seriesOfTask;
  QMutex mutex;
  QVector<QPointF> points;
};
#endif  // MAINWINDOW_H
