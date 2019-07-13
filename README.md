# Drone_from_Scratch

Convert a manual drone to autonomous one

### How to Use this Repo

Make a workspace if you don't have it.
```
cd
mkdir -p catkin_ws/src/
cd ~/catkin_ws/
catkin_make
```
Now clone the packages to your desired directory.
```
cd (Your directory)
https://github.com/Shubham-Sahoo/Drone_from_Scratch.git
```

Install the Firmware package in your companion system from - https://github.com/PX4/avoidance
 
Make Sure that Gazebo is already installed in your system.

Now copy the folders from aerial_robotics/src/ folder into ~/catkin_ws/src/ and compile it

```
cd ~/catkin_ws/
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source ~/catkin_ws/devel/setup.bash
```

If you don't see any error after executing catkin_make then you're ready to fly autonomously 

### Now to execute simulation on Gazebo, follow the commands

```
cd ~/Firmware/
export GAZEBO_RESOURCE_PATH=/usr/share/(gazebo-version)
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash 
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 posix_sitl.launch
```

##### In another terminal 

```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

This will place a drone on the gazebo simulator which can be controlled through mavros.


Now to use the Avoidance Package in simulator close all the terminals and relaunch a new terminal

```
cd ~/Firmware/
export QT_X11_NO_MITSHM=1
make px4_sitl_default gazebo
```
Stop the code by pressing ctrl-C 
```
. ~/Firmware/Tools/setup_gazebo.bash ~/Firmware ~/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Firmware
roslaunch local_planner local_planner_stereo.launch
```

## Result Output : 
![](https://github.com/Shubham-Sahoo/Drone_from_Scratch/blob/kinetic/Local_planner_output.gif?raw=true)







