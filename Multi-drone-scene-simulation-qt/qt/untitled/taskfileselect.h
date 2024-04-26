#ifndef TASKFILESELECT_H
#define TASKFILESELECT_H

#include <QWidget>

namespace Ui {
class TaskFileSelect;
}

class TaskFileSelect : public QWidget
{
    Q_OBJECT

public:
    explicit TaskFileSelect(QWidget *parent = nullptr);
    ~TaskFileSelect();

private slots:
    void on_TaskFileButton_clicked();

    void on_SecondLastButton_clicked();

private:
    Ui::TaskFileSelect *ui;
};

#endif // TASKFILESELECT_H
