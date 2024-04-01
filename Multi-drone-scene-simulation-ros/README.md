

## 无人机群应用场景仿真工具

本项目基于ROS、PX4和Gazebo实现无人机的仿真，并基于QT开发了相应的图形化应用界面

### 一、无人机群仿真平台

### 项目结构介绍

- /data

  存放场景对应的任务文件

- /model

  存放用于路径规划的强化学习的模型文件

- /docs

  存放本项目的相关文档

- /src

  存放本项目的功能包代码

### ROS功能包介绍

- /src/image_transport

  将无人机搭载摄像头所拍摄的图片保存至本地

- /src/task_allocation

  将任务分配给参与拍卖的无人机节点

- /src/task_loading

  读取任务信息

- /src/task_management
  
  任务管理节点用于发布任务信息
  
- /src/uav_control
  
  用于控制仿真中无人机的飞行以及发布无人机的位置信息
  
- /src/uav_msgs
  
  定义了无人机信息和任务信息的数据格式
  
- /src/view_point_plan
  
  用于对任务提取视点和进行路径规划
  
### 任务文件介绍

- /data/waypoint
  
  路径点，即任务与任务之间的飞行路线
  
- /data/task_info
  
  任务信息，对应一个设施的全部任务
  
- /data/subTask
  
  子任务信息，对应一个设施的一个特定任务
  
- /data/Obstacle
  
  障碍物信息
  
### 二、图形化界面工具

- 文件加载及仿真环境启动

  加载task_info、subTask、Obstacle、waypoint文件
  
  ![](scene-simulation/preview/文件加载及仿真环境启动.png)

- 任务规划

  进行任务分配和路径规划

  ![](scene-simulation/preview/任务规划.png)

- 无人机信息回传

  点击相应按钮启动无人机节点、显示无人机的信息

  （注：此处有绝对路径，如启动的launch文件中的无人机为两架，那么对应的分配任务和启动无人机命令也应为uavs2_bid.launch和uavs2.launch，需在qt的代码里修改）

  ![](scene-simulation/preview/无人机信息回传.png)

- 电子地图

  展示任务、无人机等的位置信息

  ![](scene-simulation/preview/电子地图.png)

### 三、依赖介绍

- ROS
  
  对应的ROS版本为ros neotic
  
- python
  
  对应的python版本为3.8，具体依赖见requirements.txt
  
- Qt

  Qt版本为5.12.12

- mavros

  ```
  sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
  ```
  
  ```
  chmod +x install_geographiclib_datasets.sh
  ```
  
  ```
  sudo ./install_geographiclib_datasets.sh
  ```

- PX4
  
  PX4版本为v1.12.3
  
  ```
  git clone -b v1.12.3 https://github.com/PX4/PX4-Autopilot.git --recursive
  ```
  
  在安装完px4后，还需做一点修改：
  
  1. 为调用搭载传感器的无人机模型：将single_vehicle_spawn_highway.launch放入PX4的launch文件夹下。
  2. 为在ROS中正确获取传感器topic：在传感器的sdf文件中加入\<robotNamespace>,使其获得默认的命名前缀，参见p450_stereo_camera.sdf文件，注意请不要使用model://这种方式组合无人机模型和传感器模型，可能会导致mavros连不上。
  3. （可以酌情使用）若要使用非PX4自带的机架模型，需要在/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix文件夹下加入对应的airframes，如1045_p450,如果这之后还报错，可以在.ros文件夹下检查对应的/etc/init.d-posix文件中是否存在对应的airframes。

### 备注
由于github单个文件100M的限制，uav_control/models/brige0714/meshes下的dae文件和view_point_plan/scripts/data下的pth文件进行了压缩，直接解压即可。

### 地面站系统

地面站系统目前正在开发中，预计的组织结构为将uav_bid功能包拆分后的拍卖节点和uav_control放在无人机机载电脑上，其余部分部署在地面站电脑上。

