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
```



