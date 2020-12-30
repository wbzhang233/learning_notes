# PX4_NOTES

## 1.gazebo

cd ~/OctomapPlanner/
gazebo --verbose ./worlds/obstacle_avoidance_demo.world
gazebo --verbose ./worlds/

## 2.px4

```shell
# 1-进入光流避障SITL仿真
cd ~/Firmware/
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    # (optional)
#source ~/catkin_optflow/devel/setup.bash   # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

roslaunch launch/iris_mavros_posix_sitl.launch x:=-5
# optional
roslaunch launch/iris_mavros_posix_sitl.launch world_name:=boxes3 x:=-25
roslaunch launch/iris_mavros_posix_sitl.launch world_name:=obs_env_boxes x:=-60

## 新增demo.launch
#source /home/wbzhang/catkin_ws/devel/setup.bash
#source /home/wbzhang/catkin_optflow/devel/setup.bash
#roslaunch optflow_obs optobs_demo.launch

# 2-打开QGC
cd ~/
./QGroundControl.AppImage

# 3.光流避障
cd ~/catkin_optflow
source devel/setup.bash
rosrun optflow_obs classTest_node
# 4.rviz显示
cd ~/catkin_optflow
source devel/setup.bash
roslaunch showpath showpath.launch

# 3-实验操控
# offboard 发送位置
rostopic pub -r 20 /mavros/setpoint_position/local geometry_msgs/PoseStamped "header:
# 位置输入15, 3, 1.3

# setpoint_raw航向改变控制
rostopic pub -r 20 /mavros/setpoint_raw/local  mavros_msgs/PositionTarget "coordinate_frame"
# 8;3015;坐标

# 录制消息
rosbag record -O path.bag  /mavros/local_position/pose px4/local_path

# 发送下一个航迹点
rostopic pub -r 20 /mavros/mission/waypoints "mavros_msgs/WaypointList" "current_seq: 0
# 航迹点 frame==2 表示LOCAL_NED
rostopic pub -r 20 /mavros/mission/waypoints mavros_msgs/WaypointList "current_seq: 0
waypoints:
- {frame: 2, command: 0, is_current: true, autocontinue: false, param1: 0.0, param2: 0.0,
  param3: 0.0, param4: 0.0, x_lat: 200.0, y_long: 5.0, z_alt: 1.3}"
# 着陆点位置是79.2 0.77 0.05



# 光流避障1
rosrun optflow_obs optflow_obs_node

# 光流避障2
rosrun optflow_obs px4_obs_node

#  板外模式
rosrun optflow_obs opt_offb_node

# 4- 使用节点录制数据集

## TTC阈值取80-255效果都不错，取80的时候效果比较好，可以直接区分开地面和天空，并且区分出地平线上的障碍物;
建议取值80-120

#  单独编译某个包
catkin_make -DCATKIN_WHITELIST_PACKAGES="landing;offb"
cd ~/catkin_ws & catkin_make_isolated -DCMAKE_BUILD_TYPE=Release
```

### youtube上的小实验

```shell
https://www.youtube.com/watch?v=2jksI-S3ojY
# 切换Offboard模式
rosrun mavros mavsys mode -c OFFBOARD

# 解锁
rosrun mavros mavsafety arm

# 发送速度
rostopic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "linear:

z方向的角速度为改变航向

# 发送位置
rostopic pub -r 20 /mavros/setpoint_position/local geometry_msgs/PoseStamped "header:

# 
rosservice call /mavros/setpoint_velocity/mav_frame "mav_frame: 8"
```

## setpoint_raw机体坐标系下的速度控制

```shell
# 发送raw速度
# setpoint_raw/local (mavros_msgs/PositionTarget)
rostopic pub -r 20 /mavros/setpoint_raw/local  mavros_msgs/PositionTarget "coordinate_frame"
# 机体系设置：8
# 速度控制码字：4039，位置控制码字：4088，速度+偏航控制码字为3015

```

### 发送目标
```shell
cd ~/catkin_sendgoal
source devel/setup.bash
rosrun simple_navigation_goals send_goal
```



### CLION配置ROS

```shell
-DCATKIN_DEVEL_PREFIX:PATH=/home/wbzhang/catkin_optflow/devel
/home/wbzhang/catkin_optflow/build
-j8
```



## 3.avoidance

```powershell
# 启动
roslaunch global_planner global_planner_stereo.launch world_file_name:=window
roslaunch global_planner global_planner_stereo.launch world_file_name:=obstacle_env2

# 录制
cd ~/bags/
rosbag record -O stereo_0518box.bag /stereo/right/image_raw /stereo/left/image_raw
# 录制原始航线数据与播放
rosbag record -O mission.bag /mavros/local_position/pose
rosbag play file.bag /mavros/local_position/pose:=/px4/mission_pose

cd /home/wbzhang/bags/0518_stereo/

mkdir obs6
# 修改export
roslaunch export.launch bag_name:=obs6
mv ~/.ros/frame*.jpg /home/wbzhang/bags/0518_stereo/box_r/
```



## 场景搭建

灰色墙的高度是2.8m，宽度为15m;以其中心点计算位置



## 3.1 光流节点

接收图像消息，计算光流场，FOE和达到时间TTC，采用平衡法则进行避障。





## 4. gazebo models
http://models.gazebosim.org/

待下载的模型：

- ksql_airport
- baylands
- mcmillan_airfield
- yosemite




## 5.path

``` powershell
### Set the plugin path so Gazebo finds our model and sim
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/wbzhang/Firmware/Tools/sitl_gazebo/build
### Set the model path so Gazebo finds the airframes
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/home/wbzhang/Firmware/Tools/sitl_gazebo/models
### Disable online model lookup since this is quite experimental and unstable
export GAZEBO_MODEL_DATABASE_URI=""
### Set path to sitl_gazebo repository
export SITL_GAZEBO_PATH=/home/wbzhang/Firmware/Tools/sitl_gazebo
```



# 2.移动机器人路径规划学习

## 2.1 重点实验室

1) 宾大 Vijay Kumar的GRASP实验室

- sikang

2) PX4团队

3) APM团队

4) ETHZ

5) 港科大沈劭劼团队



## 2.2 路径规划器

- navigation

- ==fast-planner==

- ==mpl_ros==

- APM: Octomap-planner

- PX4

  avoidance

- voxblox_ros

- 

  



## 2.3 仿真器

- ETHZ-ASL的 rotors_simulator+ octomap

  https://dev.px4.io/v1.9.0/en/simulation/gazebo_octomap.html

- 肖昆 XTDrone

- drone-kit

```
//        initMarkersAttributes(cylinder_marker,cuboid_marker,tree_marker);

// 初始化三种标识的属性
//void initMarkersAttributes(visualization_msgs::Marker &cylinder_marker,visualization_msgs::Marker &cuboid_marker,
//                           visualization_msgs::Marker &tree_marker){
//    cylinder_marker.header.frame_id = cuboid_marker.header.frame_id = tree_marker.header.frame_id = "/map_ned";
//    cylinder_marker.header.stamp = cuboid_marker.header.stamp = tree_marker.header.stamp = ros::Time::now();
//    cylinder_marker.ns = cuboid_marker.ns = tree_marker.ns = "showvis_node";
//    cylinder_marker.action = cuboid_marker.action = tree_marker.action = visualization_msgs::Marker::ADD;
//    cylinder_marker.id = 3;
//    cuboid_marker.id = 4;
//    tree_marker.id = 5;
//
//    // 位置
//    geometry_msgs::Quaternion quaternion;
//    quaternion.x = quaternion.y= quaternion.z =quaternion.w =0;
//    tree_marker.pose.orientation.x = tree_marker.pose.orientation.y = tree_marker.pose.orientation.z = tree_marker.pose.orientation.w = 0;
////    cylinder_marker.pose.orientation = cuboid_marker.pose.orientation = tree_marker.pose.orientation = quaternion;
////    cylinder_marker.pose.position.x = cylinders[i].x;
////    cylinder_marker.pose.position.y = cylinders[i].y;
////    cylinder_marker.pose.position.z = 0;
////    cuboid_marker.pose.position.x = cuboids[j].x;
////    cuboid_marker.pose.position.y = cuboids[j].y;
////    cuboid_marker.pose.position.z = 0;
////    tree_marker.pose.position.x = trees[k].x;
////    tree_marker.pose.position.y = trees[k].y;
////    tree_marker.pose.position.z = 0;
//
//    // 形状、尺寸、颜色
//    cylinder_marker.type = visualization_msgs::Marker::CYLINDER;
//    cuboid_marker.type = visualization_msgs::Marker::CUBE;
//    tree_marker.type = visualization_msgs::Marker::CYLINDER;
//    cylinder_marker.scale.x =0.3; cylinder_marker.scale.y = 0.3; cylinder_marker.scale.z = 1.0;
//    cuboid_marker.scale.x = 0.8; cuboid_marker.scale.y = 0.8; cuboid_marker.scale.z = 1.8;
//    tree_marker.scale.x = 1.0; tree_marker.scale.y = 1.0; tree_marker.scale.z = 3.0;
//    cylinder_marker.color.r = 1.0f; cylinder_marker.color.g = 0.2f; cylinder_marker.color.b = 0.2f; cylinder_marker.color.a = 1.0;
//    cuboid_marker.color.r = 0.2f; cuboid_marker.color.g = 0.2f; cuboid_marker.color.b = 1.0f; cuboid_marker.color.a = 1.0;
//    tree_marker.color.r = 0.2f; tree_marker.color.g = 1.0f; tree_marker.color.b = 0.2f; tree_marker.color.a = 1.0;
//
////    geometry_msgs::Point p;
////    for (int j = 0; j < cylinders.size(); ++j) {
////        p.x = cylinders[j].x; p.y = cylinders[j].y; p.z=0;
////        cylinder_marker.points.push_back(p);
////    }
////    ROS_INFO("cylinders:",cylinder_marker.points.size());
////
////    for (int j = 0; j < cuboids.size(); ++j) {
////        p.x = cuboids[j].x; p.y = cuboids[j].y; p.z=0;
////        cuboid_marker.points.push_back(p);
////    }
////    ROS_INFO("cuboid_marker:",cuboid_marker.points.size());
////    for (int j = 0; j < trees.size(); ++j) {
////        p.x = trees[j].x; p.y = trees[j].y; p.z=0;
////        tree_marker.points.push_back(p);
////    }
////    ROS_INFO("tree_marker:",tree_marker.points.size());
//}

```



# 3. PX4-Avoidance

```shell
# 1-进入SITL仿真
cd ~/Firmware/
DONT_RUN=1 make px4_sitl_default gazebo
source ~/Code/catkin_avoidance/devel/setup.bash # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

# 立体相机
roslaunch local_planner local_planner_stereo.launch
# 深度相机
roslaunch local_planner local_planner_depth-camera.launch
# 或者3 kinetic相机
roslaunch local_planner local_planner_sitl_3cam.launch

# 2-打开QGC
cd ~/
./QGroundControl.AppImage

# 3.local_planner
cd ~/Code/catkin_avoidance
source devel/setup.bash

# 切换模式
rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm

# 4.rviz显示
cd ~/Code/catkin_avoidance
source devel/setup.bash
rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color
```

