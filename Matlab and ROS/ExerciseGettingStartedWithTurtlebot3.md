# Exercise: Turtlebot / ROS / Matlab infrastructure

**Objective**: To get the network infrastructure between your computer and Turtlebot up and running, and to get MatLab communicating with the TurtleBot.

Each TurtleBot holds:
* A Kobuki base
* An Asus Xtion PRO Live RGB and Depth sensor
* A Raspberry pi

The Raspberry pi is configured with:
* Ubuntu 16.04 LTS, password is “robot”
* ROS Kinetic

## 0. Computer setup (the one on the Turtlebot)
The Raspberry Pi has already been set-up, so you can skip this . If you would like to setup a new computer, you can follow this short guide: 
1. Install Ubuntu 16.04 and ROS Kinetic (ROS Melodic and Noetic are not officially supported). 
2. Install Kobuti by running sudo apt install ros-kinetic-kobuti-*
3. Setup ROS host ip by running

```
echo “export ROS_HOSTNAME=192.168.1.200” >> ~/.bashrc
echo “export ROS_MASTER_URI=http://192.168.1.200:11311” >> ~/.bashrc
```

The wireless network connection is setup as: 10.42.0.1. The port 11311 is default and should always be used.

## 1.a Wired network setup
NB: 2022: Connecting through the cable has not been fully tested. The ip-addresses might therefore differ from the description.

Older HP computers:
If you are connecting a cable directly between the two computers, i.e. without a router, you have to set a static ip on your computer. Use 192.168.1.X, where X is not 200. In the rest of the exercise 100 is chosen. I.e. 192.168.1.100. Submask should be 255.255.255.0


## 1.b Wireless network setup
The turtlebot also has a wireless accesspoint, with a unique SSID for each turtlebot. The password is `robotseverywhere`. The default ip of the access-points is 10.42.0.1

## 2. Turtlebot
The TurtleBot is built according to “turtle_quickguide_A3__V0-1.pdf”. A description of the TurtleBot is found inside the “Kobuki-tb2UsersManualIndigo-v1.pdf” file. Note that the User Manual is based on Indigo, but we are going to use the Kinetic, meaning that Indigo should be substituted with Kinetic all over the manual.
The Kinect sensor is in general shown with the TurtleBot, but you must have in mind that we are using the Asus Xtion Pro Live sensor.

## 3. Starting up the turtlebot

For running the examples below, your TurtleBot should be started by running the following command on your turtlebot:

SSH into the Turtlebot using it's ip (10.42.0.1 for wireless connection). Username is `ubuntu`, password is `ubuntu`

start-up the turtlebot bu running:
```
roslaunch turtlebot_bringup minimal.launch --screen
```

You can also start-up the camera and 3d-sensor by running
```
roslaunch turtlebot_bringup 3dsensor.launch
```
but we will not use that sensor in this exercise.

If your are using Gazebo, you should instead run the following command from a terminal on the computer running Gazebo:
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```




## 4. MatLab installation
Initialization inside MatLab is recommended as:
```
% assuming the Turtlebot ip is 192.168.1.200
setenv('ROS_MASTER_URI','http://192.168.1.200:11311')
% assuming your own ip is 192.168.1.100
setenv('ROS_IP','192.168.1.100')
rosinit('http://192.168.1.200:11311','NodeHost','192.168.1.100');
```

If you are using Windows, you might have to disable your firewall in order to have MatLab communicating with the TurtleBot.


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



