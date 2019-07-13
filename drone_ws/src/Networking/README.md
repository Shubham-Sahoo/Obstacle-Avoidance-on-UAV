# Networking with ROS on RaspBerry PI 3B+

Images were transferred and streamed in video using ROS(Robot Operating System) over ethernet and wifi networks from RaspBerry Pi 3B+ to workstation for processing.

## Steps to get networking done in your system

### Dependencies
1. Install ROS Kinetic [Link](http://wiki.ros.org/kinetic/Installation/Ubuntu)
2. Install ROS on Raspbian [Link](http://wiki.ros.org/ROSberryPi/Setting%20up%20ROS%20on%20RaspberryPi)
3. Install OpenCV on your system as well as on RaspBerry Pi [Link](https://www.learnopencv.com/install-opencv3-on-ubuntu/)
4. Install cv_bridge
5. Install arp-scan on your system
6. Install openssh-server on your system as well as on RaspBerry Pi and enable ssh on both the systems

### Now you are ready to use this repo

Clone the repo to your desired directory on your system
```
$ cd {Your directory}
$ git clone https://github.com/Shubham-Sahoo/Networking.git
```
Make a catkin workspace, if you already don't have it
```
$ mkdir -p ~/catkin_ws/src
$ cd catkin_ws/
$ catkin_make
$ cd catkin_ws/src/
$ catkin_create_pkg networking rospy roscpp std_msgs OpenCV cv_bridge image_transport
```

Now copy the publisher and subscriber file to your networking directory just created in catkin_ws/src
```
$ cd
$ cd {Your directory}/Networking/src
$ cp Publisher.cpp Subscriber.cpp /home/{user}/catkin_ws/src/networking/src/
```

Now change your CMakeLists.txt in networking folder by adding these lines at the end of the file
```
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Publisher src/Publisher.cpp)
target_link_libraries(Publisher ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(Subscriber src/Subscriber.cpp)
target_link_libraries(Subscriber ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
```
You can check the CMakeLists.txt for reference


Now make the package
```
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```

#### Repeat the exact same process of using the repo on Raspberry Pi

Now you are ready for the networking part
### Through Ethernet cable

Connect the Raspberry Pi to your System and do execute the following on terminal
```
sudo arp-scan --interface=eth0 --localnet
```
You will get a IP Address similar to 10.42.XX.XX  
Copy the IP Address and use it to ssh into the RaspBerry Pi

```
sudo ssh pi@10.42.XX.XX
```

Now we have two choices:
1. Run roscore on RaspBerry Pi
2. Run roscore on OffBoard System

Let's take the first case where roscore is running on RaspBerry Pi

Open a terminal on RaspBerry Pi and do the following
```
export ROS_MASTER_URI=http://10.42.XX.XX(IP of Raspberry Pi):11311
export ROS_IP=10.42.XX.XX(IP of RaspBerry Pi)
roscore
```
In another terminal on RaspBerry Pi
```
export ROS_MASTER_URI=http://10.42.XX.XX(IP of Raspberry Pi):11311
export ROS_IP=10.42.XX.XX(IP of RaspBerry Pi)
rosrun networking Publisher
```

Now you need to setup your system to subscribe the messages

Open a terminal on your system and do
```
ifconfig
```
Note the IP Address of your system of the format 10.42.YY.YY

In the same terminal execute
```
export ROS_MASTER_URI=http://10.42.XX.XX(IP of Raspberry Pi):11311
export ROS_IP=10.42.YY.YY(IP of your system)
rosrun networking Subscriber
```
If everything goes well, a window will pop up showing the video from the camera of RaspBerry Pi

The second case can also be done just by reversing the operations of roscore on the machines.



