#include "taskfileselect.h"

#include <QFile>
#include <QFileDialog>

#include "mainwindow.h"
#include "ui_taskfileselect.h"

TaskFileSelect::TaskFileSelect(QWidget *parent)
    : QWidget(parent), ui(new Ui::TaskFileSelect) {
  ui->setupUi(this);
}

TaskFileSelect::~TaskFileSelect() { delete ui; }

void TaskFileSelect::on_TaskFileButton_clicked() {
  QString TaskfileName = QFileDialog::getOpenFileName(
      this, "请选择任务文件", "/home/zrh/hector_practice_ws/src/application",
      "任务文件(*.txt);;");
  if (TaskfileName.isEmpty()) {
    return;
  }
  ui->TasklineEdit->setText(TaskfileName);  // 界面显示路径
  // 设置文本编码个数
  // QTextCodec *qtc = QTextCodec::codecForName("utf-8");
  // 读取文件中的内容
  QFile *qf = new QFile(TaskfileName);
  qf->open(QIODevice::ReadOnly);
  QByteArray qby = qf->readAll();
  ui->TaskShowEdit->setText(qby);
  qf->close();
}
