# Drone

### Ground Station Setup

#### Componenets Used :
- Off-board (Ground Station) Computation board. [Nvidia TX2 was used]
- Wireless Adapter to Setup Connection between Off-board and On-Board (The Board that sits on the Drone). (Rocket Wireless Adapter was used)
- Carrier Board to Support Nvidia TX2 + HDD for Storage
- RC Joystick for controlling. (This communication goes through On-board to Off-board then to Flight Controller via MavLink). [X9D Taranis FrSky was used]
- Display. [10 inch Pi-Display was used]
- Keyboard and Mouse. 

##### Steps to setup Off-board
-	Install OS [Ubuntu 16.04]
	-	[Nvidia-TX2 Installation]
	-	Intel-AMD Processor based: use general installation 
		-	[Up-Board Intel Setup Guide]
	-	Refer Board Specific Ubuntu Installation Procedure.

-	Install CUDA (If the project involves processing on GPU)
-	Install ROS (for communication between different components of the System)[Install ROS-Kinetic from Source](http://wiki.ros.org/kinetic/Installation/Source)
-	Install OpenCV (If Project involves Image Processing)
-	Install Cv_bridge if not already installed. (Will be used in Networking)
-	Install MavLink (Service-Client Protocol for Communication with Flight Controller)
-	Install MavROS (ROS Wrapper for MavLink)
-	Install other required packages.
	-	RealSense Packages 
	-	ROS : Joy Node
	-	
-	Install the [Manual_Joy package] from my repo.
	It supports X9D Taranis Joystick.

 

