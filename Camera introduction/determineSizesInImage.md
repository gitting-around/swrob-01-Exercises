# Exercise: Object detection

**Objective**: To grab images from the robot's camera and detect the distance to well-known shapes

The robot is equipped with an Asus Xtion PRO camera, which contains an RGB and Depth sensor, similar to the Microsoft Kinect sensor. In this exercise we will work only on images from the RGB sensor.

## 1. Connect to the camera
Connect the robot to the computer using an ethernet cable and set-up your ip-address as described in (https://github.com/au-mobile-robots/Exercises/blob/master/Matlab%20and%20ROS/ExerciseGettingStartedWithTurtlebot.md)[last week's exercise].

On the robot launch the camera node:


## 2. Connecting Matlab to Turtlebot
On the turtlebot launch the camera node. This will automatically start-up a master
```
roslaunch turtlebot_bringup 3dsensor.launch
```

In matlab initialize ROS as in last week's exercise:
```
setenv('ROS_MASTER_URI','http://192.168.1.200:11311')
% assuming your own ip is 192.168.1.100
setenv('ROS_IP','192.168.1.100')
rosinit('http://192.168.1.200:11311','NodeHost','192.168.1.100');
```

Try running `rostopic list` and `rostopic info /topic_name`to inspect available ros topics related to the camera.

## 3. Write a script that reads images from turtlebot and determine the distance to known objects


Your script should grab an images from the turtlebot, detect known objects and calculate the distance to those objects. 

1. Place the robot approximately 2 m and perpendicular to a wall holding a number of test objects.
2. Try to estimate the constant k from the perspective transform. I.e. ![\Delta x=\frac{f}{s_x}\frac{\Delta X}{Z}=k\frac{\Delta X](https://latex.codecogs.com/svg.latex?\Large&space;\Delta%20x=\frac{f}{s_x}\frac{\Delta%20X}{Z}=k\frac{\Delta%20X}{Z}), where ![f](https://latex.codecogs.com/svg.latex?f) is the focal length, and ![s](https://latex.codecogs.com/svg.latex?s_x) is the pixel size on the sensor. From manual measurements you can determine ![\Delta x](https://latex.codecogs.com/svg.latex?\Delta%20x), ![\Delta X](https://latex.codecogs.com/svg.latex?\Delta%20X), and ![Z](https://latex.codecogs.com/svg.latex?Z). You can use the "data courser"-tool in Matlab, which can be found in "Tools>Data courser" in an image window.
3. Plot the relation between measured object lengths in pixels and camera height – relate this plot to the perspective transform, i.e. an “1/z” shape.
4. Experiment with automatic detection of various test objects and calculate the distance to those objects automatically. [This pdf-file](shapes.pdf) holds a number of shapes in different colours, which you can use for testing your script. 
	* For this part you have to detect squares / circles
	* Detect colours of the objects. Hint: Use HSV format and detect a certain Hue interval according to color range.
5. Test the robustness of your algorithm robust -- especially when light changes


You might want to take a look at [this demo](https://github.com/au-mobile-robots/Tutorials/blob/master/read%20image%20from%20camera/demo_grabImageFromRobot.m), which demonstrates how to read an image from the turtle-bot.

More on: [https://se.mathworks.com/help/images/morphological-filtering.html](https://se.mathworks.com/help/images/morphological-filtering.html) and [https://www.mathworks.com/help/images/ref/regionprops.html](https://www.mathworks.com/help/images/ref/regionprops.html)

Bonus: If you can't get enough of detecting objects, try taking a look at the following project, which demonstrates a clever way to detect black/white markers and their orientation: [https://github.com/henrikmidtiby/MarkerLocator](https://github.com/henrikmidtiby/MarkerLocator)
