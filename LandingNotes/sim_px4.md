
# 1.gazebo
cd ~/OctomapPlanner/
gazebo --verbose ./worlds/obstacle_avoidance_demo.world
gazebo --verbose ./worlds/

# 2.px4
cd ~/Firmware/
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

roslaunch launch/


# avoidance

roslaunch global_planner global_planner_stereo.launch world_file_name:=window
roslaunch global_planner global_planner_stereo.launch world_file_name:=obstacle_env2


rosbag record -O obs3.bag /stereo/right/image_color /stereo/left/image_color

mv ~/.ros/frame*.jpg /home/wbzhang/bags/left3/

# gazebo models
http://models.gazebosim.org/


## path

# Set the plugin path so Gazebo finds our model and sim

export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/wbzhang/Firmware/Tools/sitl_gazebo/build
# Set the model path so Gazebo finds the airframes
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/home/wbzhang/Firmware/Tools/sitl_gazebo/models
# Disable online model lookup since this is quite experimental and unstable
export GAZEBO_MODEL_DATABASE_URI=""
# Set path to sitl_gazebo repository
export SITL_GAZEBO_PATH=/home/wbzhang/Firmware/Tools/sitl_gazebo
