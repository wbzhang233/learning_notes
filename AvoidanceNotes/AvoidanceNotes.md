# AvoidanceNotes

> 无人机视觉避障的笔记．

## 1. px4

```bash
# 1-进入光流避障SITL仿真
cd ~/Firmware/
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    # (optional)
#source ~/Code/catkin_avoidance/devel/setup.bash   # (optional)
#source ~/catkin_optflow/devel/setup.bash   # (optional)

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

# 启动SITL仿真
roslaunch launch/iris_mavros_posix_sitl.launch x:=-5

# 2-启动QGC
cd ~/
./QGroundControl.AppImage

# 3-启动光流节点

```

