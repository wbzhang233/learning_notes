# LandingNotes

## 1. run PX4 SITL

```shell
# 1.进入SITL PX4仿真
cd ~/Firmware/
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    # (optional)
#source ~/catkin_landing/devel/setup.bash   # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

roslaunch launch/iris_mavros_posix_sitl.launch world_name:=landing_scene
```

## 2.打开QGC

```shell
# 2-打开QGC
cd ~/
./QGroundControl.AppImage
```

## 3.运行landing节点

```shell
cd ~/catkin_landing
source devel/setup.bash
rosrun landing landing_node
```

## 4.offb节点

```shell
cd ~/catkin_landing
source devel/setup.bash
rosrun offb offb_node
```





## 2.其他技术

### 2.1 自定义消息​ :joy:

https://blog.csdn.net/learning_tortosie/article/details/103356881

https://blog.csdn.net/learning_tortosie/article/details/103356881