# Joy_Drone
### Integration of X9D Taranis Joystick with ROS and with Complete System.

[ROS JOY NODE](http://wiki.ros.org/joy)

[GitHub Source Repo for ROS/Joy](https://github.com/ros-drivers/joystick_drivers)

### How to Setup and Use ROS/Joy_Node
[Tutorial Link](http://wiki.ros.org/joy/Tutorials)

### For General JoyStick Supported by Linux :
Goto src/joystick_drivers/joy

```bash
rosrun joy joy_node _dev_name:="*"
```


## ROS API

This package contains the message "Joy", which carries data from the joystick's axes and buttons. It also has the joy_node, which reads from a given joystick port and publishes "Joy" messages.

List of nodes:

    joy_node

Deprecated nodes:

    txjoy
    joy 
The deprecated nodes are the same as the joy_node, but they will warn users when used. They are actually just bash scripts that call "joy_node" and print a warning.

ROS topics : 

Subscribes to (name / type):

    None

Publishes to (name / type):

    "joy/Joy" : Joystick output. Axes are [-1, 1], buttons are 0 or 1 (depressed).

________________________________________________________________________
