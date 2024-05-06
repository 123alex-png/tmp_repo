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

### 工具运行

##### 1. 选择相应的仿真配置文件并启动gazebo，等待gazebo加载完成

##### 2. 选择巡检任务文件和子任务文件并点击导入，等待终端运行

##### 3. 选择航线路径点文件和场景障碍物文件并点击导入，此处无启动终端运行

见下图示意：
![image](https://github.com/s-kkk/UAV-Project-Codes/blob/main/scene-simulation/preview/%E6%96%87%E4%BB%B6%E5%8A%A0%E8%BD%BD%E5%8F%8A%E4%BB%BF%E7%9C%9F%E7%8E%AF%E5%A2%83%E5%90%AF%E5%8A%A8.png)

##### 4. 点击启动uav节点按钮，等待终端运行

见下图示意：
![image](https://github.com/s-kkk/UAV-Project-Codes/blob/main/scene-simulation/preview/%E6%97%A0%E4%BA%BA%E6%9C%BA%E4%BF%A1%E6%81%AF%E5%9B%9E%E4%BC%A0.png)
##### 5. 此时任务信息框中应已显示相关的任务信息，点击任务分配按钮，等待终端运行

##### 6. 点击巡检路径规划按钮，等待终端运行

##### 7. 点击选择任务，以html的形式显示任务的规划结果（非必要）

##### 8. 点击开始执行按钮，稍作等待，仿真中的无人机便会解锁并执行相应任务

见下图示意：
![image](https://github.com/s-kkk/UAV-Project-Codes/blob/main/scene-simulation/preview/%E4%BB%BB%E5%8A%A1%E8%A7%84%E5%88%92.png)
