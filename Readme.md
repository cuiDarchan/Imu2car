# 点云数据坐标转换小工具
## 1. 软件依赖 
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 ，ROS Kinetic 
### 1.2  **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).
### 1.3  **Eigen**
Recommend version [3.3.7](http://eigen.tuxfamily.org/index.php?title=Main_Page).
## 2. 使用指南
### 2.1 Imu2car
```
catkin_make -DCATKIN_WHITELIST_PACKAGES="imu2car"
source devel/setup.bash
rosrun imu2car imu2car_node
rosbag play xx.bag -l
rviz
```
### 2.2 OriginToImu(extrinsic_broadcaster)
```
roslaunch imu2car tf.launch
```
## 3. 可修改值：  
```
imu_to_car : imu到车身坐标系变换 
sub_livox_compensator_  ：lidar订阅topic 
pub_livox_car_ ： 转换后发布的新topic 
```