## 模块介绍
### 蓝牙模块
说明：此控制APP只能在安卓机上运行
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210105223213115.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80NTg2NzAzMg==,size_16,color_FFFFFF,t_70)
控制界面

## 串口里程计

# sago_odomIMU
### IMU的通信使用
1.先下载编译
```bash
mkdir -p catkin_serialpi/src
cd catkin_serialpi/src/
git clone https://github.com/Tarmy3124/sago_odomIMU.git
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="carMsgs"
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```
编译成功则如下：
```bash
[ 82%] Building CXX object sago_odomIMU/ros_serial_imu/CMakeFiles/serial_odom_imu.dir/ImuCommand.cpp.o
[ 88%] Building CXX object sago_odomIMU/ros_serial_imu/CMakeFiles/serial_odom_imu.dir/serial_example_node1.cpp.o
[ 88%] Built target odom_serial_generate_messages_lisp
[ 88%] Built target odom_serial_generate_messages_nodejs
[ 88%] Built target odom_serial_generate_messages_cpp
WARNING: Package name "carMsgs" does not follow the naming conventions. It should start with a lower case letter and only contain lower case letters, digits, underscores, and dashes.
[ 94%] Generating Python msg __init__.py for odom_serial
[ 94%] Built target odom_serial_generate_messages_py
WARNING: Package name "carMsgs" does not follow the naming conventions. It should start with a lower case letter and only contain lower case letters, digits, underscores, and dashes.
[ 94%] Built target odom_serial_generate_messages_eus
Scanning dependencies of target odom_serial_generate_messages
[ 94%] Built target odom_serial_generate_messages
[100%] Linking CXX executable /home/tarmy/ros/sago/catkin_serialpi/devel/lib/odom_serial/serial_odom_imu
[100%] Built target serial_odom_imu
```
2.将工作目录告知系统
新建一个命令行页面：快捷键 ctrl+alt+t，然后输入如下命令
```bash
sudo gedit ~/.bashrc 
在结尾文件结尾添加
source ~/ros/sago/catkin_serialpi/devel/setup.bash
```
注意：~/ros/sago/这个目录要根据你的catkin_serialpi代码包放在哪里来决定
3.启动可执行程序
新建一个命令行终端，输入：
roslaunch odom_serial serial_common.launch
4.查看话题：
```bash
rostopic list 
则应该看到以下话题
/imu_raw
/odom_wheel
/pid_float
/rosout
/rosout_agg
/tf
```
具体查看imu话题内容则新建命令行终端输入:
rostopic echo /imu_raw
5.rviz可视化显示话题
新建终端，输入命令rviz
在display中配置如下
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210114152608979.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80NTg2NzAzMg==,size_16,color_FFFFFF,t_70)

### dynamic_pid配置
```bash
rosrun dynamic_cfgPID roscar
```

```bash
rosrun rqt_reconfigure rqt_reconfigure
```
启动pid和odom的launch

```bash
roslaunch dynamic_cfgPID pid_odom.launch
```
## 运行launch

```bash
roslaunch odom_serial serial_common.launch
```
## 闲鱼操作
### Odom/IMU
预先安装

```bash
cd ~/sago/sago_odomIMU
git clone https://github.com/ros-drivers/rosserial.git
cd  rosserial
git checkout remotes/origin/melodic-devel
catkin_make -DCATKIN_WHITELIST_PACKAGES="rosserial"
```

```bash
 mkdir -p sago_odomIMU/src
 cd sago_odomIMU/src
 git clone https://github.com/Tarmy3124/sago_odomIMU.git
 catkin_make -DCATKIN_WHITELIST_PACKAGES="carMsgs"
  catkin_make -DCATKIN_WHITELIST_PACKAGES="dynamic_cfgPID"
   catkin_make -DCATKIN_WHITELIST_PACKAGES="odom_serial"
 catkin_make
```
## 效果呈现
### 建图
#### Cartographer
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210105222936777.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80NTg2NzAzMg==,size_16,color_FFFFFF,t_70)
