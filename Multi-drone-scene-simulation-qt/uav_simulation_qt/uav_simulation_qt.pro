QT       += core gui webenginewidgets webengine webchannel
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

 QT += charts

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    Sources/QRosThread.cpp \
    Sources/main.cpp \
    Sources/mainwindow.cpp

HEADERS += \
    Headers/QRosThread.h \
    Headers/mainwindow.h

FORMS += \
    Forms/mainwindow.ui
# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

QMAKE_CXXFLAGS+=-Wno-deprecated-copy

RESOURCES += \
    Resources.qrc

DISTFILES +=

INCLUDEPATH += /opt/ros/noetic/include
INCLUDEPATH += $$PWD/Sources
INCLUDEPATH += $$PWD/Headers
INCLUDEPATH += $$PWD/Forms
INCLUDEPATH += /home/cs504/px4_catkin_ws/devel/include
DEPENDPATH += /opt/ros/noetic/lib
LIBS += -L/opt/ros/noetic/lib
LIBS += /opt/ros/noetic/lib/librosbag.so
LIBS += /opt/ros/noetic/lib/libroscpp.so
LIBS += /opt/ros/noetic/lib/libroslib.so
LIBS += /opt/ros/noetic/lib/libroslz4.so
LIBS += /opt/ros/noetic/lib/librostime.so
LIBS += /opt/ros/noetic/lib/libroscpp_serialization.so
LIBS += /opt/ros/noetic/lib/librospack.so
LIBS += /opt/ros/noetic/lib/libcpp_common.so
LIBS += /opt/ros/noetic/lib/librosbag_storage.so
LIBS += /opt/ros/noetic/lib/librosconsole.so
LIBS += /opt/ros/noetic/lib/libxmlrpcpp.so
LIBS += /opt/ros/noetic/lib/librosconsole_backend_interface.so
LIBS += /opt/ros/noetic/lib/librosconsole_log4cxx.so

