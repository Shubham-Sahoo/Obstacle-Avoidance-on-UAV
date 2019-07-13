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
git clone https://github.com/reyanshsolis/aerial_robotics.git
```

Now copy the folders from aerial_robotics/src/ folder into ~/catkin_ws/src/ and compile it

```
cd ~/catkin_ws/
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source ~/catkin_ws/devel/setup.bash
```

If you don't see any error after executing catkin_make then you're ready to fly autonomously 

Now to execute simulation on Gazebo follow the commands

```


```



