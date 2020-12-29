# apm_notes

> 题记：最近做毕业设计得用到无人机仿真，重操旧业，搞一搞SITL仿真。
> 给个传送门参考：[无人机SITL仿真](https://blog.csdn.net/weixin_44479297/article/details/95218005#ardupilotSITLAPM_6)
> [APM软件在环仿真](https://zhuanlan.zhihu.com/p/62017292)
> 我的环境：ubuntu18.04虚拟机

# 1.官方教程
ardupilot官网和官方git帐号，里面有很多东西可以学习。

- [APM官方教程](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)
- [APM官方git](https://github.com/ArduPilot)

1)先安装编译环境，[Setting up the Build Environment](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux);
具体步骤如下：

```shell
# 1.更新并安装git
sudo apt-get update
sudo apt-get install git
sudo apt-get install gitk git-gui
# 2.clone repo
cd ~/
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
# 3.install some required packages
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

```
成功后应该是这样的，此后就可以进行仿真了。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200421002807654.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3diemhhbmcyMzM=,size_16,color_FFFFFF,t_70#pic_center)接下来可以按照BUILD.md中的进行waf编译。
```shell
# 编译SITL
./waf configure --board sitl
./waf copter
```

> 这里记录一下我遇到的一个关于Mavproxy的问题。
> 这个![在这里插入图片描述](https://img-blog.csdnimg.cn/20200422081731571.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3diemhhbmcyMzM=,size_16,color_FFFFFF,t_70#pic_center)
> 从APM官方git中，下载安装Mavproxy即可，注意之后将build/scripts2.7/中的文件拷贝到~/.local/lib/python2.7/site-packages/MAVProxy-1.8.18.dist-info/scripts/下，问题解决。

附加：查看可用的板子命令
./waf list_boards
结果如下：
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200421221605448.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3diemhhbmcyMzM=,size_16,color_FFFFFF,t_70#pic_center)
```shell
# 4.Add some directories to your search path (Facultative)
# 把下面语句加入~/.bashrc文件中
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
# 然后使用点命令重新载入
. ~/.bashrc

```

接下来：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200421003018116.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3diemhhbmcyMzM=,size_16,color_FFFFFF,t_70#pic_center)
2）开始SITL simulator
```shell
# 1.启动SITL仿真
cd ardupilot/ArduCopter
# 以下语句擦出缓存，并且采用默认配置
sim_vehicle.py -w 
# ctrl+c推出并且按下
sim_vehicle.py --console --map

# 附加：安装并更新pymavlink和mavproxy
pip install --upgrade pymavlink MAVProxy --user

```
默认配置时会弹出一个窗口，取消后再`sim_vehicle.py --console --map`之后会有三个窗口，表示SITL成功运行啦。如下：
![在这里插入图片描述](https://img-blog.csdnimg.cn/2020042208261512.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3diemhhbmcyMzM=,size_16,color_FFFFFF,t_70#pic_center)


3）使用gazebo仿真器
[APM_Gazebo官方教程](https://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html)
得先git clone ardupilot_gazebo
```shell
# 1.安装并更新gazebo，原文建议用gazebo9;推荐在ubuntu18.04下操作啦
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update

sudo apt install gazebo9 libgazebo9-dev
# 以下打开一个gazebo新world
gazebo --verbose

# 虚拟机中若遇到问题
echo “export SVGA_VGPU10=0” >> ~/.bashrc # 关闭硬件加速
source ~/.bashrc #执行以下终端配置文件，其实最好重启终端

# 2.Plugin installation
git clone https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install

# 3.启动仿真器
cd ~/ardupilot_gazebo
gazebo --verbose worlds/iris_arducopter_runway.world

# another terminal
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map


```

# 2.APM仿真使用

```bash
mode guided
arm throttle
takeoff 2
```

# 3.mavros




# 4.octomap planner的使用

## obstacle_avoidance_demo

```bash
# first terminal
cd ~/OctomapPlanner/
gazebo --verbose worlds/iris_gas_station_demo.world

# another terminal
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --console

# another terminal
./build/main_node
```












