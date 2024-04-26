#ifndef TASKFILESELECT_H
#define TASKFILESELECT_H

#include <QWidget>

QT_BEGIN_NAMESPACE
namespace Ui { class taskfileselect; }
QT_END_NAMESPACE

class taskfileselect : public QWidget
{
    Q_OBJECT

public:
    taskfileselect(QWidget *parent = nullptr);
    ~taskfileselect();

private:
    Ui::taskfileselect *ui;
};
#endif // TASKFILESELECT_H
