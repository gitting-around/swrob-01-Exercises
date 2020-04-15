# Exercise: SLAM

**Objective**: To implement Graph SLAM and play with gmapping in ROS.

SLAM (Simultainous Localization and Mapping) is techniques for localization of a robot in an unknown environment. I.e. we don't have a map of the world and we don't know the robot's location in the world.

# 1. Implement graph slam only with poses
Implement a function, which can solve the graph SLAM problem only with a continous series of poses (you might want to start in 1D). The function should take as input a series of poses, and output the most likely locations.

![\mu=\Omega^{-1}\cdot \xi](https://render.githubusercontent.com/render/math?math=%5Cmu%3D%5COmega%5E%7B-1%7D%5Ccdot%20%5Cxi)


You can test using the following robot positions in 1D to test your function:

* x<sub>0</sub>=-3
* moves by 5
* moves by 3

Hereby ![\xi](https://render.githubusercontent.com/render/math?math=%5Cxi) should be:
```
-8
2
3
```

and  ![\mu](https://render.githubusercontent.com/render/math?math=%5Cmu) should be:
```
-3
2
3
```


HINT: You can create an ![\Omega](https://render.githubusercontent.com/render/math?math=%5COmega) matrix and ![\xi](https://render.githubusercontent.com/render/math?math=%5Cxi) vector for each pairs, which you then add together before inverting and multiplying. I.e. ![\mu = \left(\Omega_{x0}+\Omega_{x1}+\Omega_{x2}\right)^{-1}\cdot \left(\xi_{x0}+\xi_{x1}+\xi_{x2}\right)](https://render.githubusercontent.com/render/math?math=%5Cmu%20%3D%20%5Cleft(%5COmega_%7Bx0%7D%2B%5COmega_%7Bx1%7D%2B%5COmega_%7Bx2%7D%5Cright)%5E%7B-1%7D%5Ccdot%20%5Cleft(%5Cxi_%7Bx0%7D%2B%5Cxi_%7Bx1%7D%2B%5Cxi_%7Bx2%7D%5Cright))

Try adding noise to the transitions and see how it influences ![\mu](https://render.githubusercontent.com/render/math?math=%5Cmu).

# 2. Extend graph slam function
Extend your function so that it can take as input a series of landmarks and poses. 

HINT: You might seek inspiration in SLAM.py

# 3. Try playing with the SLAM.py demo
SLAM.py contains a simple robot model and a graph-slam implementation. Try to understand the code in the slam-function and see how landmark estimations changes based on number of observations and sensor noise.

# 4. Mapping in ROS
ROS contain a package called gmapping, which builds on openslam and contain a particle filter slam implementation.
You can launch the turtlebot using the following command:

```
roslaunch turtlebot_gazebo turtlebot_mw_office.launch
```
You can now launch the gmapping package to start the mapping process using the following command:
```
roslaunch turtlebot_navigation gmapping_demo.launch
```

You can use RViz to visualize the map building process:
```
roslaunch turtlebot_bringup minimal.launch
```

Move the Turtlebot around and see how the environment is being mapped. To do so, you can launch the keyboard teleoperating package:
```
roslaunch turtlebot_teleop keyboard_teleop.launch
```
