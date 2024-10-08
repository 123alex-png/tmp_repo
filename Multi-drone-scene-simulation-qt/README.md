## README

### 工具简介

**基于QT开发了相应的图形化应用界面，同属无人机群应用场景仿真工具部分**

### 环境配置

#### 1. 系统环境：

​	操作系统：Ubuntu 20.04.6 LTS

​	硬件要求：20GB内存，8GB运行内存

#### 2. 运行环境：

##### Qt 5.12.1

​	官网下载安装：

​	https://download.qt.io/archive/qt/5.12/5.12.12/ 

​	安装时需Qt charts插件和Qt web插件，或者直接选择安装全部插件

   其中.pro文件里INCLUDEPATH += /home/xxx/px4_catkin_ws/devel/include需换成相应的路径

### 工具运行

##### 1. 选择相应的仿真配置文件并启动gazebo，等待gazebo加载完成

##### 2. 选择巡检任务文件和子任务文件并点击导入，等待终端运行

##### 3. 选择航线路径点文件和场景障碍物文件并点击导入，此处无启动终端运行

见下图示意：
![image](./uav_simulation_qt/resources/resource_README/File_load.png)

##### 4. 点击启动uav节点按钮，等待终端运行

见下图示意：
![image](./uav_simulation_qt/resources/resource_README/Info_reload.png)

##### 5. 此时任务信息框中应已显示相关的任务信息，点击任务分配按钮，等待终端运行

##### 6. 点击巡检路径规划按钮，等待终端运行

##### 7. 点击选择任务，以html的形式显示任务的规划结果（非必要）

##### 8. 点击开始执行按钮，稍作等待，仿真中的无人机便会解锁并执行相应任务

见下图示意：
![image](./uav_simulation_qt/resources/resource_README/task.png)

**演示视频见./uav_simulation_qt/resources/resource_README/qt_return.webm**

