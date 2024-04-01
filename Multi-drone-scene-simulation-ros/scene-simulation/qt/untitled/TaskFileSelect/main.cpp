#include "taskfileselect.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    taskfileselect w;
    w.show();
    return a.exec();
}
