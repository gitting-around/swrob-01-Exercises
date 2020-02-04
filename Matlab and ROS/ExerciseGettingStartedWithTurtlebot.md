# Exercise: Turtlebot / ROS / Matlab infrastructure

**Objective**: To get the network infrastructure between your computer and Turtlebot up and running, and to get MatLab communicating with the TurtleBot.

Each TurtleBot holds:
* A Kobuki base
* An Asus Xtion PRO Live RGB and Depth sensor
* A HP laptop

The HP Laptop is configured with:
* Ubuntu 12.04 LTS, password is “robot”
* ROS Hydro

## 1. Computer setup
The computer has already been set-up. If you would like to setup a new computer, you can follow this short guide: 
    1. Install Ubuntu 16.04 and ROS Kinetic (ROS Melodic is not yet supported). 
    2. Install Kobuti by running sudo apt install ros-kinetic-kobuti-*
    3. Setup ROS host ip by running 
```
echo “export ROS_HOSTNAME=192.168.1.200” >> ~/.bashrc
echo “export ROS_MASTER_URI=http://192.168.1.200:11311” >> ~/.bashrc
```

The wired network connection is setup as: 192.168.0.200. The port 11311 is default and should always be used.


## 2. Turtlebot

The TurtleBot is built according to “turtle_quickguide_A3__V0-1.pdf”. A description of the TurtleBot is found inside the “Kobuki-tb2UsersManualIndigo-v1.pdf” file. Note that the User Manual is based on Indigo, but we are going to use the Hydro, meaning that Indigo should be substituted with Hydro all over the manual.
The Kinect sensor is in general shown with the TurtleBot, but you must have in mind that we are using the Asus Xtion Pro Live sensor.

**The TurtleBot installation must be tested according to section 5.1 and 5.2 inside the User’s Manual, “Kobuki-tb2UsersManualIndigo-v1.pdf”.**

## 3. MatLab installation
Initialization inside MatLab is recommended as:
```
setenv('ROS_MASTER_URI','http://192.168.1.200:11311')
setenv('ROS_IP','192.168.1.100')
rosinit('http://192.168.1.200:11311','NodeHost','192.168.1.100');
```

If you are using some weird OS like Windows, you might have to disable your firewall in order to have MatLab communicating with the TurtleBot.

For running examples below your TurtleBot should be started as:
```
roslaunch kobuki_node minimal.launch --screen
```

### Examples for testing:													
* Getting Started with a Real TurtleBot, [https://se.mathworks.com/help/robotics/examples/get-started-with-a-real-turtlebot.html](https://se.mathworks.com/help/robotics/examples/get-started-with-a-real-turtlebot.html) Note that the environment variables has been set inside the `.bashrc` file! The printenv command might be used to verify.
* Communicate with the TurtleBot,
[https://se.mathworks.com/help/robotics/examples/communicate-with-the-turtlebot.html](https://se.mathworks.com/help/robotics/examples/communicate-with-the-turtlebot.html) - Stop just before the section “Receive Image Data”
Make the robot follow a simple geometric trajectory, e.g.:

![path](path.svg)

## Some hints:
Publisher for the robot's velocity and a message for that topic:
```
robot = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(robot);
```

Define parameters for either Linear.X / Angular.Z (either one or both):
```
velmsg.Angular.Z = 0.6;	% Angular velocity (rad/s)
velmsg.Linear.X = 0.1; % Linear velocity (m/s), +forward,-reverse
```

Note that the complete velmsg is sent to the robot. If the robot has to move straight forward then set the
Angular.Z parameter to 0.
Update robot with actual velocity message:
```
send(robot,velmsg);
```

Make use of odometry topic to verify “Stop” position. Measure the robots accuracy with real
measurements. Repeat the experiment a number of times – is the accuracy similar for all experiments?
Note that you might reset the odometry inside the TurtleBot.


