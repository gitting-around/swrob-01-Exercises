# Exercise: Pursuit algorithm with obstacle avoidance

**Objective**: To combine pure pursuit with obstacle avoidance

## intro
The robot is equipped with an Asus Xtion PRO camera, which contains an RGB and Depth sensor, similar to the Microsoft Kinect sensor. In this exercise we will work only on images from the RGB sensor.
We will use a single line from the the Asus Xtion depth sensor to simulate a lidar scanner.

You may find the following links useful:
*  [MatLab - Exchange Data with ROS Publishers and Subscribers](https://se.mathworks.com/help/robotics/examples/exchange-data-with-ros-publishers-and-subscribers.html)
*  [MatLab - Work with Basic ROS Messages](https://se.mathworks.com/help/robotics/examples/work-with-basic-ros-messages.html)
*  [MatLab - Work with Specialized ROS Messages](https://se.mathworks.com/help/robotics/examples/work-with-specialized-ros-messages.html)
*  [demos, plotting line-scans](https://github.com/au-mobile-robots/Tutorials/tree/master/laserscan)


For running this exercise, your TurtleBot should be started with the following two ROS nodes -- they should be started in two terminal windows. One node is for interfacing the Kobuki base and the other node is for interfacing the Asus Xtion PRO live sensor:

```
roslaunch kobuki_node minimal.launch --screen
roslaunch turtlebot_bringup 3dsensor.launch
```

The 3dsensor.launch is using the OpenNI driver, [https://structure.io/openni](https://structure.io/openni).



## Exercise
Assume your robot is using the Pursuit-algorithm to go from A to B. During the journey the robot discover some objects on the path. To avoid the obstacles your Pursuit-algorithm exercise from last week might be extended to make simple obstacle avoidance.
Does the number of obstacles have any influence on the accuracy of your Pursuit-algorithm?
