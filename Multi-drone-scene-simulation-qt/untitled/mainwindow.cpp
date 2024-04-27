/*******************************************************************
 * File: mainwindow.cpp
 * Author: lizeshan zhangruiheng
 * Date: 2024-04-23
 * Description: QT主界面，展示相关数据并辅助用户运行项目
 *******************************************************************/
#include "mainwindow.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <QDebug>
#include <QFile>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QMutex>
#include <QMutexLocker>
#include <QStackedLayout>
#include <QTextCodec>
#include <QWebEngineView>
#include <QtCharts>
#include <fstream>
#include <iostream>
#include <vector>

#include "QRosThread.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  webview = new QWebEngineView(ui->QWebWidget);
  webview->resize(ui->QWebWidget->size());
  QLeftWidgetInit();
  QLeftWidgetInitConnect();

  ui->uavStateTableWidget->setColumnCount(11);
  ui->uavStateTableWidget->setRowCount(10);
  ui->uavStateTableWidget->setHorizontalHeaderLabels(
      QStringList() << "ID" << "无人机类型" << "载荷类型" << "实时坐标位置"
                    << "工作温度范围" << "承受最大风速" << "防水等级"
                    << "最大通信距离" << "工作状态" << "电量"
                    << "执行任务列表");
  ui->uavStateTableWidget->horizontalHeader()->setSectionResizeMode(
      QHeaderView::Stretch);
  ui->uavStateTableWidget->verticalHeader()->setSectionResizeMode(
      QHeaderView::Fixed);
  ui->uavStateTableWidget->setAlternatingRowColors(true);
  ui->uavStateTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);

  ui->taskInfotableWidget->setColumnCount(11);
  ui->taskInfotableWidget->setRowCount(10);
  ui->taskInfotableWidget->setHorizontalHeaderLabels(
      QStringList() << "ID" << "任务名称" << "任务类型" << "开始位置"
                    << "结束位置" << "所需无人机" << "载荷要求"
                    << "任务时间开销" << "任务时间窗" << "预计执行时间"
                    << "任务状态");
  ui->taskInfotableWidget->horizontalHeader()->setSectionResizeMode(
      QHeaderView::Stretch);

  ui->taskInfotableWidget->verticalHeader()->setSectionResizeMode(
      QHeaderView::Fixed);
  ui->taskInfotableWidget->setAlternatingRowColors(true);
  ui->taskInfotableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
  qrosthread = new QRosThread();
  qRegisterMetaType<uav_msgs::UAV_State::ConstPtr>(
      "uav_msgs::UAV_State::ConstPtr");
  qRegisterMetaType<uav_msgs::Task_List::ConstPtr>(
      "uav_msgs::Task_List::ConstPtr");
  qRegisterMetaType<sensor_msgs::CompressedImage::ConstPtr>(
      "sensor_msgs::CompressedImage::ConstPtr");
  qRegisterMetaType<std::string>("std::string");
  connect(qrosthread,
          SIGNAL(returnUavResult(const uav_msgs::UAV_State::ConstPtr)), this,
          SLOT(displayUavState(const uav_msgs::UAV_State::ConstPtr)));
  connect(qrosthread,
          SIGNAL(returnTaskResult(const uav_msgs::Task_List::ConstPtr)), this,
          SLOT(displayTaskState(const uav_msgs::Task_List::ConstPtr)));
  connect(qrosthread, SIGNAL(returnResult_image(const std::string)), this,
          SLOT(displayImage(const std::string)));
  qrosthread->start();

  QImage image(":/resource/image/control.png");

  start_view_plan_pub =
      nh.advertise<uav_msgs::UAV_State>("/start_view_point_plan", 10);
  start_execute_task_pub =
      nh.advertise<uav_msgs::UAV_State>("/start_execute_task", 10);

  seriesOfUAV = new QScatterSeries();
  seriesOfUAV->setName("无人机");
  seriesOfUAV->setMarkerShape(
      QScatterSeries::MarkerShapeCircle);  // 设置绘制的散点的样式为圆
  seriesOfUAV->setMarkerSize(10);
  seriesOfWaypoint = new QLineSeries();
  seriesOfWaypoint->setName("航点");

  seriesOfTask = new QScatterSeries();
  seriesOfTask->setName("任务");
  seriesOfTask->setMarkerShape(
      QScatterSeries::MarkerShapeCircle);  // 设置绘制的散点的样式为圆
  seriesOfTask->setMarkerSize(10);
  ui->chartView->chart()->addSeries(seriesOfTask);
  seriesOfWaypoint->setVisible(true);
  seriesOfWaypoint->setPointsVisible(true);
  seriesOfWaypoint->setPointLabelsVisible(true);
  for (int i = 0; i < 5; i++) {
    seriesOfUAV->append(0, 0);
  }
  ui->chartView->chart()->setBackgroundVisible(false);
  ui->chartView->chart()->legend()->setVisible(true);
  ui->chartView->chart()->legend()->setAlignment(Qt::AlignBottom);
  ui->chartView->chart()->addSeries(seriesOfUAV);
  ui->chartView->chart()->addSeries(
      seriesOfWaypoint);  // 将创建的series添加经chart中
  ui->chartView->chart()->createDefaultAxes();
  ui->chartView->chart()->axisX()->setMax(100);
  ui->chartView->chart()->axisY()->setMax(200);
  ui->chartView->chart()->axisX()->setMin(-100);
  ui->chartView->chart()->axisY()->setMin(-200);
  points.append(QPointF(0, 0));
  points.append(QPointF(0, 0));
  points.append(QPointF(0, 0));
  points.append(QPointF(0, 0));
  points.append(QPointF(0, 0));
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::QLeftWidgetInit() {
  btnsMain << ui->btnTaskLoad << ui->btnUavMonitor << ui->btnTaskPlan
           << ui->btnMapShow;
  strBtnStyle = ui->btnTaskLoad->styleSheet();
}

void MainWindow::QLeftWidgetInitConnect() {
  for (int i = 0; i < btnsMain.size(); i++) {
    QPushButton *btn = (QPushButton *)btnsMain.at(i);
    connect(btn, &QPushButton::clicked, this,
            &MainWindow::LeftBtnClicked);  // 绑定
  }
  ui->btnTaskLoad->click();  // 默认
}

void MainWindow::LeftBtnClicked() {
  QPushButton *btn = qobject_cast<QPushButton *>(sender());
  foreach (auto elemBtn, btnsMain) {
    if (elemBtn->objectName() == btn->objectName()) {
      elemBtn->setStyleSheet("background: rgb(98,102,105)");
    } else {
      elemBtn->setStyleSheet(strBtnStyle);
    }
  }
  if (btn->objectName() == "btnTaskLoad") {
    btn->setStyleSheet("background: rgb(98,102,105)");
    ui->stackedWidget->setCurrentIndex(0);
  } else if (btn->objectName() == "btnUavMonitor") {
    btn->setStyleSheet("background: rgb(98,102,105)");
    ui->stackedWidget->setCurrentIndex(1);

  } else if (btn->objectName() == "btnTaskPlan") {
    btn->setStyleSheet("background: rgb(98,102,105)");
    ui->stackedWidget->setCurrentIndex(2);

  } else if (btn->objectName() == "btnMapShow") {
    btn->setStyleSheet("background: rgb(98,102,105)");
    ui->stackedWidget->setCurrentIndex(3);
  }
}

void MainWindow::on_envFileButton_clicked()  // 选择环境配置文件
{
  envFilename = QFileDialog::getOpenFileName(this, "请选择环境配置文件",
                                             "/home", "(*.launch);;");
  if (envFilename.isEmpty()) {
    return;
  }
  ui->envlineEdit->setText(envFilename);
}

void MainWindow::on_globalTaskFileButton_clicked() {
  globalTaskFilename = QFileDialog::getOpenFileName(this, "请选择巡检任务文件",
                                                    "/home", "(*.txt);;");
  if (globalTaskFilename.isEmpty()) {
    return;
  }
  ui->globalTasklineEdit->setText(globalTaskFilename);
}
void MainWindow::on_subTaskFileButton_clicked() {
  subTaskFilename = QFileDialog::getOpenFileName(this, "请选择子任务文件",
                                                 "/home", "(*.txt);;");
  if (subTaskFilename.isEmpty()) {
    return;
  }
  ui->subTasklineEdit->setText(subTaskFilename);
}

void MainWindow::on_obstaclesFileButton_clicked() {
  obstaclesFilename = QFileDialog::getOpenFileName(this, "请选择障碍物文件",
                                                   "/home", "(*.txt);;");
  if (obstaclesFilename.isEmpty()) {
    return;
  }
  ui->obstacleslineEdit->setText(obstaclesFilename);
}

void MainWindow::on_wayPointFileButton_clicked() {
  wayPointFilename = QFileDialog::getOpenFileName(this, "请选择航线路径点文件",
                                                  "/home", "(*.txt);;");
  if (wayPointFilename.isEmpty()) {
    return;
  }
  ui->wayPointlineEdit->setText(wayPointFilename);

  QFile file(wayPointFilename);  // 参数就是读取文件的路径
  file.open(QIODevice::ReadOnly);
  QByteArray array;
  while (!file.atEnd())  // 判断是否读到文件尾
  {
    array = file.readLine();  // 按行读，追加
    QString str = QString(array);
    QStringList list = str.split(" ");
    seriesOfWaypoint->append(list[1].toFloat(), list[2].toFloat());
  }
}

void MainWindow::on_envStartButton_clicked()  // 启动仿真环境gazebo
{
  if (envStartState == false) {
    if (envFilename.isEmpty()) {
      qDebug() << "envFilename is Empty!!" << endl;
      ui->messageEdit->append("仿真配置文件为空!!");
      return;
    } else {
      qDebug() << "envFilename is " << envFilename;
      std::string path = "gnome-terminal --window -- bash -ic 'roslaunch " +
                         envFilename.toStdString() + "'";
      qDebug() << "path is " << path.c_str();
      ui->messageEdit->append("仿真配置文件导入成功，路径为：" + envFilename);
      system(path.c_str());
      envStartState = true;
    }
  } else
    return;
}
void MainWindow::on_globalTaskLoadingButton_clicked()  // 导入任务
                                                       // 启动task_loading
                                                       // task_management
{
  if (globalTaskFilename.isEmpty()) {
    qDebug() << "globalTaskFilename is Empty!!" << endl;
    ui->messageEdit->append("任务文件为空，请重新选择。");
    return;
  }
  if (subTaskFilename.isEmpty()) {
    qDebug() << "subTaskFilename is Empty!!" << endl;
    ui->messageEdit->append("子任务文件为空，请重新选择。");
    return;
  }
  std::string startTaskLoadingCommand =
      "gnome-terminal --tab -- bash -ic 'rosrun task_loading task_loading "
      "_task_directory:=" +
      globalTaskFilename.toStdString() + " '";
  std::string startTaskManagementCommand =
      "gnome-terminal --tab -- bash -ic 'rosrun task_management "
      "task_management'";
  std::string startViewPlanCommand =
      "gnome-terminal --tab -- bash -ic 'roslaunch view_point_plan "
      "uavs5_view_point_plan.launch _subTask_filename:=" +
      subTaskFilename.toStdString() +
      " _obstacles_filename:=" + obstaclesFilename.toStdString() + " '";
  system(startTaskLoadingCommand.c_str());
  system(startTaskManagementCommand.c_str());
  system(startViewPlanCommand.c_str());
}

void MainWindow::on_viewpointPlanButton_clicked() {
  uav_msgs::UAV_State uav;

  for (auto it = uavId.begin(); it != uavId.end(); it++) {
    uav.uav_id = *it;
    start_view_plan_pub.publish(uav);
  }

  // 点击视点规划后 获取子任务的数量
  std::vector<int> idList;
  qDebug() << "subTaskFilename:" << subTaskFilename << endl;
  ReadFirstNum(subTaskFilename.toStdString(), idList);

  QMap<QString, int> chooseBoxList;

  for (unsigned long i = 0; i < idList.size(); i++) {
    chooseBoxList.insert("子任务 " + QString::number(idList[i]), idList[i]);
  }
  foreach (const QString &str, chooseBoxList.keys())
    ui->chooseTaskBox->addItem(str, chooseBoxList.value(str));
}
void MainWindow::on_startUavsButton_clicked() {
  if (startUavsState == false) {
    std::string startUavsCommand =
        "gnome-terminal -x bash -ic 'roslaunch uav_control uavs2.launch "
        "way_point_directory:=" +
        wayPointFilename.toStdString() + "'";
    system(startUavsCommand.c_str());

    system(
        "gnome-terminal --tab -- bash -ic 'roslaunch task_allocation "
        "uavs2_bid.launch'");
    system("gnome-terminal -x bash -ic 'rosrun image_transport image_sub'");
    startUavsState = true;
  } else
    return;
}

void MainWindow::on_startTaskPlanButton_clicked() {
  if (startTaskPlanState == false) {
    system(
        "gnome-terminal --tab -- bash -ic 'rosrun task_allocation "
        "center_allocate.py'");
    startTaskPlanState = true;

  } else
    return;
}

void MainWindow::on_startTaskExecuteButton_clicked() {
  uav_msgs::UAV_State uav;
  for (auto it = uavId.begin(); it != uavId.end(); it++) {
    uav.uav_id = *it;
    start_execute_task_pub.publish(uav);
  }
}

/**
 * @brief: 读取文件的行数
 * @param fileName: 文件名
 * @param data: 文件信息的id列表
 */
void MainWindow::ReadFirstNum(std::string fileName, std::vector<int> &data) {
  std::ifstream inFile;
  inFile.open(fileName, std::ios::in);
  if (!inFile.is_open()) {
    qDebug() << "文件读取失败" << endl;
  }
  std::string line;
  getline(inFile, line);  // 略过第一行
  while (inFile.peek() != EOF) {
    int num;
    inFile >> num;
    getline(inFile, line);  // 略过当前行
    data.push_back(num);
  }
}

bool MainWindow::IsFileExist(QString fileName) {
  QFile file(fileName);
  if (file.exists()) {
    return true;
  }
  return false;
}

void MainWindow::on_chooseTaskBox_currentIndexChanged(const QString &arg) {
  int currentId = ui->chooseTaskBox->currentData().toInt();
  std::string TaskHtmlPath;
  std::string path = globalTaskFilename.toStdString();
  int pos = path.rfind("/");
  path = path.substr(0, pos);
  pos = path.rfind("/");
  TaskHtmlPath = path.substr(0, pos - 4) +
                 "src/application/view_point_plan/scripts/html/"
                 "subTask_" +
                 std::to_string(currentId) + ".html";
  if (IsFileExist(QString::fromStdString(TaskHtmlPath))) {
    std::cout << "加载html" << std::endl;
    webview->load(QUrl("file://" + QString::fromStdString(TaskHtmlPath)));
    webview->show();
  } else {
    std::cout << "html文件不存在" << std::endl;
  }
}

/**
 * @brief: 显示无人机信息
 * @param msg: 无人机节点发布的自身信息
 */
void MainWindow::displayUavState(const uav_msgs::UAV_State::ConstPtr &msg) {
  uavId.insert(msg->uav_id);
  std::stringstream ss;
  for (auto i : msg->accepted) {
    ss << i << ",";
  }
  std::string acceptedTask = ss.str();
  // 给表格赋值
  ui->uavStateTableWidget->setItem(
      msg->uav_id, 0, new QTableWidgetItem(QString::number(msg->uav_id)));
  ui->uavStateTableWidget->setItem(
      msg->uav_id, 1,
      new QTableWidgetItem(QString::fromStdString(msg->uav_type)));
  ui->uavStateTableWidget->setItem(
      msg->uav_id, 2,
      new QTableWidgetItem(QString::fromStdString(msg->payload_type)));
  ui->uavStateTableWidget->setItem(
      msg->uav_id, 3,
      new QTableWidgetItem(QString::fromStdString(
          "(" + std::to_string(msg->position.x).substr(0, 6) + "," +
          std::to_string(msg->position.x).substr(0, 6) + "," +
          std::to_string(msg->position.x).substr(0, 6) + ")")));
  ui->uavStateTableWidget->setItem(
      msg->uav_id, 4,
      new QTableWidgetItem(QString::fromStdString(
          "[" + std::to_string(msg->work_temperature_min) + "," +
          std::to_string(msg->work_temperature_max) + "]")));
  ui->uavStateTableWidget->setItem(
      msg->uav_id, 5, new QTableWidgetItem(QString::number(msg->max_WR)));
  ui->uavStateTableWidget->setItem(
      msg->uav_id, 6, new QTableWidgetItem(QString::number(msg->wl)));
  ui->uavStateTableWidget->setItem(
      msg->uav_id, 7,
      new QTableWidgetItem(QString::number(msg->max_comm_length)));
  ui->uavStateTableWidget->setItem(
      msg->uav_id, 8, new QTableWidgetItem(QString::number(msg->work_state)));
  ui->uavStateTableWidget->setItem(
      msg->uav_id, 9, new QTableWidgetItem(QString::number(msg->power)));
  ui->uavStateTableWidget->setItem(msg->uav_id, 10,
                                   new QTableWidgetItem(acceptedTask.c_str()));
  QMutexLocker locker(&mutex);
  points[msg->uav_id] = QPointF(msg->position.x, msg->position.y);
  seriesOfUAV->replace(points);
}

/**
 * @brief: 显示任务的信息
 * @param msg: 任务管理节点发布的任务信息
 */
void MainWindow::displayTaskState(const uav_msgs::Task_List::ConstPtr &msg) {
  uav_msgs::TASK_State task;

  for (unsigned int i = 0; i < msg->task_list.size(); i++) {
    task = msg->task_list[i];

    ui->taskInfotableWidget->setItem(
        task.task_id, 0, new QTableWidgetItem(QString::number(task.task_id)));
    ui->taskInfotableWidget->setItem(
        task.task_id, 1,
        new QTableWidgetItem(QString::fromStdString(task.task_name)));
    ui->taskInfotableWidget->setItem(
        task.task_id, 2,
        new QTableWidgetItem(QString::fromStdString(task.task_type)));
    ui->taskInfotableWidget->setItem(
        task.task_id, 3,
        new QTableWidgetItem(QString::fromStdString(
            "(" + std::to_string(task.EnterPoint.way_point_pos.x).substr(0, 6) +
            "," + std::to_string(task.EnterPoint.way_point_pos.y).substr(0, 6) +
            "," + std::to_string(task.EnterPoint.way_point_pos.z).substr(0, 6) +
            ")")));
    ui->taskInfotableWidget->setItem(
        task.task_id, 4,
        new QTableWidgetItem(QString::fromStdString(
            "(" + std::to_string(task.LeavePoint.way_point_pos.x).substr(0, 6) +
            "," + std::to_string(task.LeavePoint.way_point_pos.y).substr(0, 6) +
            "," + std::to_string(task.LeavePoint.way_point_pos.z).substr(0, 6) +
            ")")));
    ui->taskInfotableWidget->setItem(
        task.task_id, 5,
        new QTableWidgetItem(QString::fromStdString(task.req_uav_type)));
    ui->taskInfotableWidget->setItem(
        task.task_id, 6,
        new QTableWidgetItem(QString::fromStdString(task.req_payload_type)));
    ui->taskInfotableWidget->setItem(
        task.task_id, 7,
        new QTableWidgetItem(QString::number(task.execute_time)));
    ui->taskInfotableWidget->setItem(
        task.task_id, 8,
        new QTableWidgetItem(
            QString::fromStdString("[" + std::to_string(task.Ts) + "," +
                                   std::to_string(task.Te) + "]")));
    ui->taskInfotableWidget->setItem(
        task.task_id, 9,
        new QTableWidgetItem(
            QString::fromStdString("[" + std::to_string(task.S_t) + "," +
                                   std::to_string(task.E_t) + "]")));
    if (task.done == 0)
      ui->taskInfotableWidget->setItem(
          task.task_id, 10,
          new QTableWidgetItem(QString::fromStdString("未分配")));
    else if (task.done == 1)
      ui->taskInfotableWidget->setItem(
          task.task_id, 10,
          new QTableWidgetItem(QString::fromStdString("已分配待执行")));
    else if (task.done == 2)
      ui->taskInfotableWidget->setItem(
          task.task_id, 10,
          new QTableWidgetItem(QString::fromStdString("无满足约束无人机")));
    else if (task.done == 3)
      ui->taskInfotableWidget->setItem(
          task.task_id, 10,
          new QTableWidgetItem(QString::fromStdString("执行中")));
    else
      ui->taskInfotableWidget->setItem(
          task.task_id, 10,
          new QTableWidgetItem(QString::fromStdString("已完成")));
    seriesOfTask->append(
        (task.EnterPoint.way_point_pos.x + task.EnterPoint.way_point_pos.x) / 2,
        (task.EnterPoint.way_point_pos.y + task.EnterPoint.way_point_pos.y) /
            2);
  }
}

/**
 * @brief: 显示无人机相机的实时画面
 * @param name: 无人机节点的名字
 */
void MainWindow::displayImage(const std::string name) {
  std::string path = globalTaskFilename.toStdString();
  int pos = path.rfind("/");
  path = path.substr(0, pos);
  pos = path.rfind("/");
  std::string str = path.substr(0, pos - 4) +
                    "src/application/image_transport/image/" + name +
                    "/image_save.jpg";
  QString filename = QString::fromStdString(str);
  QImage *img = new QImage(filename);
  data / if (name == "uav0")
             ui->image_label1->setPixmap(QPixmap::fromImage(*img));
  if (name == "uav1") ui->image_label2->setPixmap(QPixmap::fromImage(*img));
  if (name == "uav2") ui->image_label3->setPixmap(QPixmap::fromImage(*img));
  if (name == "uav3") ui->image_label4->setPixmap(QPixmap::fromImage(*img));
  if (name == "uav4") ui->image_label5->setPixmap(QPixmap::fromImage(*img));
}
