# SITL 软件在环仿真环境

- ubuntu18.04 /ubuntu16.04
- ROS melodic
- PX4
- mavros
- QGC
- gazebo
- clion (recommend)
- RVIZ (optional)



## launch 文件写法

launch文件如下：

```xml
<?xml version="1.0"?>
<launch>
    <!-- MAVROS posix SITL environment launch script -->
    <!-- launches MAVROS, PX4 SITL, Gazebo environment, and spawns vehicle -->
    <!-- vehicle pose -->
    <arg name="x" default="0"/>
    <arg name="y" default="0"/>
    <arg name="z" default="0"/>
    <arg name="R" default="0"/>
    <arg name="P" default="0"/>
    <arg name="Y" default="0"/>
    <!-- vehicle model and world -->
    <arg name="est" default="ekf2"/>
    <arg name="vehicle" default="iris"/>
    <!-- --> 
    <arg name="world_name" default="simple_obstacle4"/> 
    <arg name="world" default="$(find avoidance)/sim/worlds/$(arg world_name).world"/>
    <arg name="model" default="iris_fpv_cam" />
    <!--<arg name="sdf" default="$(find mavlink_sitl_gazebo)/models/$(arg vehicle)/$(arg vehicle).sdf"/>-->

    <!-- gazebo configs -->
    <arg name="gui" default="false"/>
    <arg name="debug" default="false"/>
    <arg name="verbose" default="false"/>
    <arg name="paused" default="false"/>
    <arg name="respawn_gazebo" default="false"/>
    <!-- MAVROS configs -->
    <arg name="fcu_url" default="udp://:14540@localhost:14557"/>
    <arg name="respawn_mavros" default="false"/>
    <!-- PX4 configs -->
    <arg name="interactive" default="true"/>
    <!-- PX4 SITL and Gazebo -->
    <include file="$(find px4)/launch/iris_posix_sitl.launch">
        <arg name="x" value="$(arg x)"/>
        <arg name="y" value="$(arg y)"/>
        <arg name="z" value="$(arg z)"/>
        <arg name="R" value="$(arg R)"/>
        <arg name="P" value="$(arg P)"/>
        <arg name="Y" value="$(arg Y)"/>
        <arg name="world" value="$(arg world)"/>
        <arg name="vehicle" value="$(arg vehicle)"/>
        <arg name="model" value="$(arg model)"/>
        <arg name="gui" value="$(arg gui)"/>
        <arg name="interactive" value="$(arg interactive)"/>
        <arg name="debug" value="$(arg debug)"/>
        <arg name="verbose" value="$(arg verbose)"/>
        <arg name="paused" value="$(arg paused)"/>
        <arg name="respawn_gazebo" value="$(arg respawn_gazebo)"/>
    </include>
    <!-- MAVROS -->
    <include file="$(find mavros)/launch/px4.launch">
        <!-- GCS link is provided by SITL -->
        <arg name="gcs_url" value=""/>
        <arg name="fcu_url" value="$(arg fcu_url)"/>
        <arg name="respawn_mavros" value="$(arg respawn_mavros)"/>
    </include>

    <!-- RVIZ 
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find showpath)/config/optflow_rviz2.rviz" required="true" />-->


</launch>
```
