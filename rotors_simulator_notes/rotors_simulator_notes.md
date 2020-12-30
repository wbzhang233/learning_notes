# rotors_simulator_notes

## 1.demo测试

```bash
# 启动gazebo
roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic

# 飞机起飞
rostopic pub /firefly/command/motor_speed mav_msgs/Actuators '{angular_velocities: [100, 100, 100, 100, 100, 100]}'

```

