<a href="#1-installation">Installation</a> • <a href="#2-setup">Setup</a>

<h1 align="center"> ARCoSBot </h1>

# 1. Installation
## 1.1 Required Dependences: 
```
sudo apt install ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-cartographer ros-humble-cartographer-ros ros-humble-gazebo-plugins ros-humble-teleop-twist-keyboard  ros-humble-teleop-twist-joy ros-humble-xacro ros-humble-nav2* ros-humble-urdf 

```
```
cd ~/your workspace
colcon build
```
## 1.2 Clone this repo 
Make sure you clone the repo in your robot and your remote PC 
```
mkdir ~/your workspace/src && cd ~/your workspace/src
```
```
git clone https://github.com/MrRox1337/ARCoSbot.git
```
```
cd ~/your workspace/src/tortoisebot
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

```
```
cd ~/your workscpace
colcon build
```
# 2. Setup

- Run bringup.launch.py to only spawn the robot
- Run autobringup.launch.py to spawn the robot with navigation and slam/localization
- Launch the files with use_sim_time:=False when working on real robot

### 2.1 Launching the robot

```
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=True exploration:=True
```
- exploration:=False for passed a saved map to navigation

### 2.2 Launch files for reference
#### SLAM
- cartographer.launch.py
#### Navigation
- navigation.launch.py
#### Rviz
- rviz.launch.py
#### Gazebo
- gazebo.launch.py

# 3. Demos

Simulation | Visualisation of Sensors (Lidar, Odometery, Camera) 
:-------------------------:|:-------------------------:
![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/002.png) |![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/005.png) 

Teleop |  Mapping | Navigation 
:-------------------------:|:-------------------------:|:-------------------------: 
![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/007.png) |  ![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/006.png) | ![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/010.png)

[Running Demos](https://github.com/rigbetellabs/tortoisebot/wiki/5.-Running-Demos)
