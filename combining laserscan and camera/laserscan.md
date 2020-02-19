# Exercise: Combining laser scan and camera

**Objective**: To get Matlab communicating with the TurtleBot and and its depth camera.

The robot is equipped with an Asus Xtion PRO camera, which contains an RGB and Depth sensor, similar to the Microsoft Kinect sensor. In this exercise we will work only on images from the RGB sensor.
We will use a single line from the the Asus Xtion depth sensor to simulate a lidar scanner.

You may find the following links useful:
*  [MatLab - Exchange Data with ROS Publishers and Subscribers](https://se.mathworks.com/help/robotics/examples/exchange-data-with-ros-publishers-and-subscribers.html)
*  [MatLab - Work with Basic ROS Messages](https://se.mathworks.com/help/robotics/examples/work-with-basic-ros-messages.html)
*  [MatLab - Work with Specialized ROS Messages](https://se.mathworks.com/help/robotics/examples/work-with-specialized-ros-messages.html)


For running this exercise, your TurtleBot should be started with the following two ROS nodes -- they should be started in two terminal windows. One node is for interfacing the Kobuki base and the other node is for interfacing the Asus Xtion PRO live sensor:

```
roslaunch kobuki_node minimal.launch --screen
roslaunch turtlebot_bringup 3dsensor.launch
```

The 3dsensor.launch is using the OpenNI driver, [https://structure.io/openni](https://structure.io/openni).

# 1. Examples for testing
Communicating with the TurtleBot,
* [https://se.mathworks.com/help/robotics/examples/communicate-with-the-turtlebot.html](https://se.mathworks.com/help/robotics/examples/communicate-with-the-turtlebot.html)
(continue from the section “Receive Image Data”!)


Explore Basic Behavior of the TurtleBot,
* [https://se.mathworks.com/help/robotics/examples/explore-basic-behavior-of-the-turtlebot.html](https://se.mathworks.com/help/robotics/examples/explore-basic-behavior-of-the-turtlebot.html)


# 2. Combining RGB and line scan
This exercise is extending the “Object Detection” exercise.
Use the algorithm found in the Object Detection exercise to find a specific object. Combine the output of the RGB camera and the `/scan` topic in order to have the TurtleBot centered e.g. 1 m perpendicular in front of the object
