#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QAbstractButton>
#include <QWebEngineView>
#include <uav_msgs/UAV_State.h>
#include <uav_msgs/Task_List.h>
#include <uav_msgs/TASK_State.h>
#include <uav_msgs/Way_Point.h>
#include <geometry_msgs/Point32.h>
#include <ros/ros.h>
#include <thread>
#include <QPixmap>
#include<std_msgs/UInt16.h>
#include<std_msgs/Int8.h>
#include <QtCharts>
#include <QMutex>
#include <QMutexLocker>
#include <QVector>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"


QT_BEGIN_NAMESPACE
class QRosThread;
namespace Ui {
class MainWindow;

}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void changePage(int index); //切换界面


private slots:
    void QLeftWidgetInit();   //左侧导航栏
    void QLeftWidgetInitConnect();  //左侧导航栏内按钮绑定
    void LeftBtnClicked();


    void on_envFileButton_clicked();    //仿真环境导入栏  选择launch文件
    void on_envStartButton_clicked();   //仿真环境导入栏  启动launch文件




    void on_globalTaskFileButton_clicked();     //任务导入栏  点击选择任务文件
    void on_subTaskFileButton_clicked();        //任务导入栏  点击选择子任务文件
    void on_globalTaskLoadingButton_clicked();  //任务导入栏  点击导入任务文件绑定

    void on_obstaclesFileButton_clicked(); //场景约束导入栏  选择障碍物文件
    void on_wayPointFileButton_clicked();  //场景约束导入栏  选择路径点文件



    void on_viewpointPlanButton_clicked();

    void on_startUavsButton_clicked();

    void on_startTaskPlanButton_clicked();

    void on_chooseTaskBox_currentIndexChanged(const QString &arg1);

    void ReadFirstNum(std::string fileName, std::vector<int>& data);
    bool IsFileExist(QString fileName);

    void displayUavState(const uav_msgs::UAV_State::ConstPtr &msg);
    void displayTaskState(const uav_msgs::Task_List::ConstPtr &msg);

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
    bool envStartState=false;
    bool viewpointPlanState=false;
    bool startUavsState=false;
    bool startTaskPlanState=false;

    QRosThread *qrosthread;

    ros::NodeHandle nh;

    QScatterSeries* seriesOfUAV;
    QLineSeries *seriesOfWaypoint;
    QScatterSeries* seriesOfTask;
    QMutex mutex;
    QVector<QPointF> points;



};
#endif // MAINWINDOW_H
