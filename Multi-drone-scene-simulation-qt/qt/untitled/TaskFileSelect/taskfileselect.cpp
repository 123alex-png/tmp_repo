#include "taskfileselect.h"
#include "ui_taskfileselect.h"

taskfileselect::taskfileselect(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::taskfileselect)
{
    ui->setupUi(this);
}

taskfileselect::~taskfileselect()
{
    delete ui;
}

