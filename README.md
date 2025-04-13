#  Get your ARCoSBot Ready for action
_(only if your SD card is booted with the Tortoisebot Image follow "Step 0" or skip to "Step 1")_
# 0. Loading Custom Raspberry Pi Image For Tortoisebot on your SD Card
## https://github.com/rigbetellabs/tortoisebot/wiki/Custom-Image-Flash

# 1. System Setup: Laptop / SBC Setup
Install Ubuntu 22.04 on Remote PC
- If Ubuntu 22.04 => Install ROS2 Humble

## 2. Create ROS 2 Workspace

```
source /opt/ros/humble/setup.bash
```

```
mkdir -p ~/ros2_ws/src
```

```
cd ~/ros2_ws/
```

```
colcon build
```

```
~/ros2_ws/install/setup.bash
```
## 3. Install Dependencies

```
sudo apt install   ros-$ROS_DISTRO-joint-state-publisher   ros-$ROS_DISTRO-robot-state-publisher   ros-$ROS_DISTRO-cartographer   ros-$ROS_DISTRO-cartographer-ros   ros-$ROS_DISTRO-gazebo-plugins   ros-$ROS_DISTRO-teleop-twist-keyboard   ros-$ROS_DISTRO-teleop-twist-joy   ros-$ROS_DISTRO-xacro   ros-$ROS_DISTRO-nav2*   ros-$ROS_DISTRO-urdf

```
## 4. Clone Repository

```
cd ~/ros2_ws/src
```

```
git clone https://github.com/mrrox1337/ARCoSBot.git
```

```
cd ~/ros2_ws/
```

```
colcon build
```

```
source /opt/ros/humble/setup.bash
```

```
source ~/ros2_ws/install/setup.bash
```

# 5. Simulation 

## Get ARCoSBot running in Simulation:

#### 1. Let's start by opening up a terminal and entering the world of ROS2! 
Type the following command to enter the ROS2:
  ````
source /opt/ros/humble/setup.bash
source <your workspace directory>/install/setup.bash
  ````
#### 2. Now, let's have some fun with simulation and sensor data visualization! 
Type this command in another terminal:
  ````
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True
  ````
  ![Gazebo and Rviz ](https://github.com/rigbetellabs/tortoisebot_docs/blob/ros2/imgs/tortoiseBot_demo/tortoisebotbringup.png?h=50&w=50)

#### 3. Look at that! You have a ARCoSBot running in Gazebo and Rviz! You can visualize its sensor data too! Now, let's get ready for teleoperation. 
Type this command in another terminal:
  ````
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
  ````
#### 4. Whoop whoop! You are now teleoperating the ARCoSBot!
![Gazebo n Rviz Giving a Goal](https://github.com/rigbetellabs/tortoisebot_docs/blob/ros2/gifs/tortoisebot-teleop.gif)
#### 5. Let's move on to simulation and navigation in exploration mode! 
Type this command in another terminal:
  ````
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=True exploration:=True
  ````
#### 6. Woohoo! The ARCoSBot is exploring the simulated environment autonomously! You can give it a goal to navigate to as well! Check out the following image:

![Gazebo n Rviz Giving a Goal](https://github.com/rigbetellabs/tortoisebot_docs/blob/ros2/gifs/navigation.gif)

## Make your ARCoSBot explore your room!

#### 1. Let's now move on to simulation and SLAM! First, enter the ROS2 world by typing the following command in a new terminal:
  ````
source /opt/ros/$ROS_DISTRO/setup.bash
source <your workspace directory>/install/setup.bash
  ````
#### 2. Now, let's start the Gazebo simulation and SLAM! 
Type this command in another terminal:
  ````
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=True exploration:=True
  ````

In another terminal type the following command:
  ````
python3 obstacle_avoidance/obstacle_avoidance/laser_obstacle_avoidance.py
  ````

#### 3. Whoa! The ARCoSBot is now mapping the environment! You can teleoperate it to map the environment even better or continue running the path-based navigator!

press `ctrl+c` to cancel the script and stop the robot by pressing `k` on `teleop_twist_keyboard`


  ![Exploration Sim Rviz and Gazebo](https://github.com/rigbetellabs/tortoisebot_docs/blob/ros2/gifs/mapping.gif)

#### 4. Let's save the map now! Type this command in another terminal:
  ````
ros2 run nav2_map_server map_saver_cli -f /path_to_map/name_of_map_file.yaml
  ````
You'll be able to see something similar to this on your terminal:
  ````
ubuntu@ubuntu:~/Desktop/workspaces/ARCoSBot$ ros2 run nav2_map_server map_saver_cli -f /home/ubuntu/Desktop/workspaces/ARCoSBot/src/ARCoSBot/tortoisebot_bringup/maps/room2.yaml
[INFO] [1680187250.898838998] [map_saver]: 
	map_saver lifecycle node launched. 
	Waiting on external lifecycle transitions to activate
	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[INFO] [1680187250.898933015] [map_saver]: Creating
[INFO] [1680187250.899004279] [map_saver]: Saving map from 'map' topic to '/home/ubuntu/Desktop/workspaces/ARCoSBot/src/ARCoSBot/tortoisebot_bringup/maps/room2.yaml' file
  ````
#### 5. You have saved the map.
#### 6. Lastly, when you are done exploring, close everything.
#### Remember:
Every time you open a new terminal, enter the ROS2 world and workspace again by typing:
  ````
source /opt/ros/humble/setup.bash
source <your workspace directory>/install/setup.bash
  ````
That's it! Have fun exploring and mapping with the Tortoisebot!

# 6. Real Robot
## Network Setup with System/Laptop <--> Robot  
## Setting up your wifi ssid and password in the image file
0. Recommended to execute the below cmds on a Linux Machine or MAC Machine
1. Insert the SD card into your computer using a card reader.
2. Navigate to the "writable" folder and then to "/etc/netplan/".
3. Open the "50-cloud-init.yaml" file in a text editor.
4. Look for the "wifis" section in the file.
5. Add the following code under the "wifis" section, replacing "ssid_name" with your WiFi name and "psd" with your WiFi password:
````
wlan0:
  optional: true
  access-points:
    "ssid_name":
      password: "psd"
  dhcp4: true
  ````
![TTB IRL](https://github.com/rigbetellabs/tortoisebot_docs/blob/ros2/imgs/tortoiseBot_demo/path_for_wifi_add.png)
![TTB IRL](https://github.com/rigbetellabs/tortoisebot_docs/blob/ros2/imgs/tortoiseBot_demo/wifi_ssid_pass.png)

- #### Example 
![TTB IRL](https://github.com/rigbetellabs/tortoisebot_docs/blob/ros2/imgs/tortoiseBot_demo/wifi_rpi_config.png)

6. Save the file and exit the text editor.
7. Safely eject the SD card from your computer and insert it into the robot.

- #### The above steps will add your WiFi network credentials to the robot's configuration file, allowing it to connect to your WiFi network.

### Here's a guide to help you connect with your real robot and have some fun exploring its capabilities.

- #### Connect to your robot
The first step is to get your robot's IP address. Don't worry, it's not as complicated as it sounds. Check out the picture below for a step-by-step guide on how to obtain the IP address.

![https://github.com/rigbetellabs/tortoisebot_docs/blob/master/imgs/tortoiseBot_setup/001.jpeg](https://github.com/rigbetellabs/tortoisebot_docs/blob/master/imgs/tortoiseBot_setup/001.jpeg)

#### Once you have the IP address, use the command below to connect with your real robot on your PC terminal:
  ````
ssh ubuntu@192.168.0.120
  ````
- Password: your entered password


## Real Robot Sensors Data Visualization

#### 1. Launch all the sensors and actuators on your robot using the following launch command

````
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=False
````

Your ARCoSBot is up and running. Now let's command the robot to move using our PC.

#### 2. Open a new terminal on the PC and source ROS2 using the following command:
  ````
source /opt/ros/humble/setup.bash
source <your workspace directory>/install/setup.bash
  ````
 
#### 3. Now let's teleoperate with the ARCoSBot.
Enter the following command on the PC terminal:

````
ros2 run teleop_twist_keyboard teleop_twist_keyboard
````
##### or

````
ros2 run tortoisebot_control teleop_twist_keyboard
````
And now you can see your ARCoSBot move as you wish!
#### You can also visualize what your robot sees using the following steps:

#### 4. Open another terminal on the PC and source the ROS2 on it

#### 5. To visualize the sensor data, enter the following:

Enter the following command on the PC terminal:
````
rviz2
````
Click File -> Open Config -> "Home" -> (your workspace) -> "ARCoSBot" -> "tortoisebot_description" -> "rviz" -> "tortoisebot_sensor_display.rviz"
![Robot IRL VIZ](https://github.com/rigbetellabs/tortoisebot_docs/blob/ros2/imgs/tortoiseBot_demo/irl_robot_viz.jpeg)

## Running the ARCoSBot in Exploration Mode

#### 1. Open Robot Terminal 1 using ssh and again fire up the ROS2

````
source <your workspace directory>/install/setup.bash
````
#### 2. Launch the following command to run the ARCoSBot in exploration mode!

````
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False exploration:=True
````
The ARCoSBot has now started mapping the environment! You can visualize it and follow this:
#### 3. Open a PC terminal and source the ROS2 and your workspace
  ````
source /opt/ros/humble/setup.bash
source <your workspace directory>/install/setup.bash
  ````

#### 4. To visualize the data:

Enter the following command on the PC terminal:
````
rviz2
````
Click File -> Open Config -> "Home" -> (your workspace) -> "ARCoSBot" -> "tortoisebot_description" -> "rviz" -> "tortoisebot_sensor_display.rviz"
![TTB View Sensor Data](https://github.com/rigbetellabs/tortoisebot_docs/blob/ros2/imgs/tortoiseBot_demo/irl_viz_02.jpeg)

#### The ARCoSBot is now mapping the environment. You can now teleoperate it to map the environment, or perform path-based navigation using the `laser_obstacle_avoidance` script. Try providing a goal to your ARCoSBot using the '2D Goal Pose' button on the Rviz screen to see your robot move autonomously.

### Time to save your MAP.


#### 5. Open a new terminal on your robot using ssh and source the ROS2:

````
source <your workspace directory>/install/setup.bash
````

#### 6. Save the map of your room with this!

````
ros2 run nav2_map_server map_saver_cli -f /path_to_map/name_of_map_file.yaml
````

#### 7. Close everything ( from the robot terminal and pc terminal)

#### 8. Opening your map the next time you launch your robot

````
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False exploration:=False map:=/home/../map_file_name.yaml
````
- Note: Make sure the ARCoSBot is placed at approximately the same position as when it was while starting the mapping

#### 9. Visualize the robot on your PC using your PC terminal:
Enter the following command on the PC terminal:
````
source /opt/ros/humble/setup.bash
source <your workspace directory>/install/setup.bash
rviz2
````
Click File -> Open Config -> "Home" -> (your workspace) -> "ARCoSBot" -> "tortoisebot_description" -> "rviz" -> "tortoisebot_sensor_display.rviz"

#### 10. Perform Goal-based navigation using the following
````
python3 tortoisebot_navigation/scripts/waypoints.py
````
***

### *** If using multiple robots set separate Domain Id for each robot using the below command for separate control  ***
```
export ROS_DOMAIN_ID=30
```