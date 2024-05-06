## 无人机群应用场景仿真工具README

### 项目简介

#### 无人机群应用场景仿真工具

**本项目基于ROS、PX4和Gazebo实现了无人机的仿真，并基于QT开发了相应的图形化应用界面**

#### 1. 无人机群仿真平台

+ #### 项目结构

  + ##### /data 存放场景对应的任务文件

    + /data/hill/waypoint

      路径点，即任务与任务之间的飞行路线

    + /data/hill/task_info

      任务信息，对应一个设施的全部任务

    + /data/hill/subTask

      子任务信息，对应一个设施的一个特定任务

    + /data/hill/Obstacle

      障碍物信息

  - ##### /src 存放本项目的功能包代码

    - /src/application/image_transport

      将无人机搭载摄像头所拍摄的图片保存至本地

    - /src/application/task_allocation

      将任务分配给参与拍卖的无人机节点

    - /src/application/task_loading

      读取任务信息

    - /src/application/task_management

      任务管理节点用于发布任务信息

    - /src/application/uav_control

      用于控制仿真中无人机的飞行以及发布无人机的位置信息

    - /src/application/uav_msgs

      定义了无人机信息和任务信息的数据格式

    - /src/application/view_point_plan

      用于对任务提取视点和进行路径规划

#### 2. 地面站系统

地面站系统目前正在开发中，预计的组织结构为将uav_bid功能包拆分后的拍卖节点和uav_control放在无人机机载电脑上，其余部分部署在地面站电脑上。

### 环境配置

#### 1. 系统环境：

​	操作系统：Ubuntu 20.04.6 LTS

​	硬件要求：20GB内存，8GB运行内存

#### 2. 运行环境：

##### 1. Ros-noetic

​	参考安装教程：

​	[Ubuntu20.04系统ros-neotic版本的安装与安装过程中遇到的一些问题（纯小白教程）_ubuntu ros-ne-CSDN博客](https://blog.csdn.net/weixin_43433726/article/details/106899359)

​	进行至 初始化 rosdep报错，参考该教程中初始化 rosdep：

​	[ubuntu18.04安装ROS简洁详细教程，常见错误解决-CSDN博客](https://blog.csdn.net/haihgdhahga/article/details/131493015)

​	最后在hosts中添加一行至末尾：

```
~$ sudo gedit /etc/

# 添加至末尾并保存

151.101.84.133 raw.githubusercontent.com
```

##### 2. PX4 Autopilot

​	**前提：需能保证本机能够连接外网**

​	以下仅供参考，可根据个人采取方式不同自行调整，不做统一要求：

​	虚拟机环境：

​	[Ubuntu虚拟机共享主机VPN（适用于NAT或桥接） - 简书 (jianshu.com)](https://www.jianshu.com/p/6c7abd4adc9b)

​	网络配置完成后：

1. 下载px4固件包：

   `~$ git clone -b v1.12.3 https://github.com/PX4/PX4-Autopilot.git --recursive`

2. 若某些子模块因网络原因下载失败，在PX4-Autopilot路径下继续下载剩余的子模块：

   `~$ cd PX4-Autopilot/`

   `~$ git submodule update --init --recursive`

3. 下载完成后安装px4固件包：

   `~$ bash ./PX4-Autopilot/Tools/setup/ubuntu.sh`

   若中间报错执行：

   `~$ python3 -m pip install --upgrade pip`

   `~$ python3 -m pip install --upgrade Pillow`

   若存在某些安装失败，进行修复：

    `~$ bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --fix-missing`

4. 重启系统

5. 编译px4：

   `~$ cd PX4-Autopilot/`

   `~$ make px4_sitl_default gazebo`

6. 编译完成显示以下说明编译成功：

   ![image](https://github.com/s-kkk/UAV-Project-Codes/blob/main/scene-simulation/preview/px4.png)

7. 将px4路径添加入环境变量：

   ```
   ~$ gedit ~/.bashrc
   
   ###  在最后一行加入以下内容
   
   source /home/xxx/UAV/ros/px4_catkin_ws/devel/setup.bash
   
   # >>> PX4 initialize >>>
   
   source /home/xxx/PX4-Autopilot/Tools/setup_gazebo.bash /home/xxx/PX4-Autopilot/ /home/xxx/PX4-Autopilot/build/px4_sitl_default
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/xxx/PX4-Autopilot
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/xxx/PX4-Autopilot/Tools/sitl_gazebo
   
   # <<< PX4 initialize <<<
   
   ###  更新环境变量使生效
   
   ~$ source ~/.bashrc
   ```

   **注意：**

   **px4安装路路径默认只需更改xxx为自己的系统用户名即可，若本机不一致，需要自行查看并更改相应路径**

   **配置时需要保证：px4配置在source setup之后，否则后续roscd px4会出现报错找不到包的问题**

8. 手动修改配置：

   调用搭载传感器的无人机模型准备：

   将项目中uav_control/launch中的single_vehicle_spawn_highway.launch复制到PX4-Autopilot的launch文件夹中

##### 3. ROS包 MAVROS

​	若后续没有修改mavros的需求，推荐二进制安装：

```
~$ sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras

### 安装完后执行

~$ wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

~$ chmod +x install_geographiclib_datasets.sh

~$ sudo ./install_geographiclib_datasets.sh
```

##### 4. Python

​	推荐使用Anaconda进行python管理，参考教程：

[	Ubuntu20.04安装anaconda-CSDN博客](https://blog.csdn.net/abc20150125/article/details/129998028)

​	新建环境 torch，并安装python-3.8.10：

​	`~$ conda create -n torch python==3.8.10`

​	根据本项目提供requirements安装所需packages：

​	`~$ pip install -r requirements.txt --ignore-installed`

​	**注意：**

​	**某些包可能会下载失败(由于源的问题或者其他问题导致)**

​	**需要单独使用pip或者conda进行安装，并尽可能与requirements中版本号一致**

##### 5. 补充配置

​	1. 额外安装xmlstarlet

​		`sudo apt-get install xmlstarlet`

2. 将view_point_plan/scripts/PointNetwork中cal_path.py中环境变量更改为本机Anaconda位置

     `#!/home/xxx/anaconda3/envs/torch/bin/python`

### 部署运行

#### 1. 项目编译

​	进入项目目录，catkin_make编译：

```
~$ cd px4_catkin_ws

### 由于引用顺序导致编译到uav_mags文件报错，故需要首先单独编译该文件：

~/px4_catkin_ws$ catkin_make -DCATKIN_WHITELIST_PACKAGES="uav_mags"

### 再编译全体文件，使用缺省命令：

~/px4_catkin_ws$ catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

**编译完成后将uav_control中的生成的models文件夹复制至home/.gazebo文件夹中**

#### 2. 部署运行

**以下两者方式均可**

##### 1. 终端部署运行

```
### 1.启动仿真环境

~$ roslaunch uav_control multi_uav_mavros_sitl_sdf_hill_1.launch

### 2.进行路径规划

~$ rosrun start_view_point_plan start_view_point_plan 

### 3.开始执行任务

~$ rosrun start_execute_task start_execute_task

### 4.启动任务加载节点，globalTaskFilename为对应的任务文件路径

~$ rosrun task_loading task_loading _task_directory:=globalTaskFilename

### 示例：~$ rosrun task_loading task_loading _task_directory:=/home/liangzx/px4_catkin_ws_0425/data/hill/task_info_hill.txt

### 5.启动任务管理节点	

~$ rosrun task_management task_management 

### 6.启动视点规划节点，subTaskFilename为对应的子任务文件路径&&obstacleFilename为对应的障碍物文件路径

~$ roslaunch view_point_plan uavs2_view_point_plan.launch subTask_filename:= subTaskFilename  
	obstacles_filename:=obstaclesFilename
	
### 示例：~$ roslaunch view_point_plan uavs5_view_point_plan.launch subTask_filename:=/home/liangzx/px4_catkin_ws_0425/data/hill/subTask_hill.txt obstacles_filename:=/home/liangzx/px4_catkin_ws_0425/data/hill/Obs_hill.txt

### 7.启动无人机节点，wayPointFilename为对应的航点文件路径

~$ roslaunch uav_control uavs2.launch way_point_directory:= wayPointFilename 

### 示例：~$ roslaunch uav_control uavs2.launch way_point_directory:=/home/liangzx/px4_catkin_ws_0425/data/hill/waypoint_hill.txt

### 8.启动无人机的拍卖节点

~$ roslaunch task_allocation uavs2_bid.launch

### 9.启动中心拍卖节点

~$ rosrun task_allocation center_allocate.py 

### 10.在该节点的终端下输入任意数字：
### goto->2.进行路径规划 rosrun start_view_point_plan start_view_point_plan

### 11.在该节点的终端下输入任意数字：
### goto->3.开始执行任务rosrun start_execute_task start_execute_task
```

**可结合项目配套演示视频《[命令行执行.webm](https://github.com/s-kkk/UAV-Project-Codes/blob/main/scene-simulation/演示视频/命令行执行.webm)》完成**

##### 2. QT部署运行

**见项目配套演示视频《[qt执行.webm](https://github.com/s-kkk/UAV-Project-Codes/blob/main/scene-simulation/演示视频/qt执行.webm)》**

#### 3. 问题排查

**首先需保证上述流程中高亮部分无遗漏完成**

#####  1. roslaunch报错px4找不到

查验安装PX4 Autopilot中第7项是否符合要求

##### 2. px4运行第一个命令可能会卡死并报错pid xxx killed

排除是硬件条件问题所导致，原因可能是：models找不到hill文件，排查编译后是否按要求复制文件 

##### 3. 避免因为gazebo未完全结束进程导致模型加载出错卡死，可手动结束gazebo进程

`killall gzserver`

`killall gzclient`

##### 4. 库GLIBCXX_3.4.29(或其他版本)缺少问题

可能是由于GCC版本过低或者缺少库等问题导致：

[丝滑解决ImportError: /usr/lib/x86_64-linux-gnu/libstdc++.so.6: version `GLIBCXX_3.4.29‘ not found问题-CSDN博客](https://blog.csdn.net/weixin_45942927/article/details/134028773)

##### 5. pip默认包搜索无效问题：

某些包通过pip安装后，依旧报错找不到，可能必须要conda安装才能生效，如包ploty的特殊情况

