# Exercise: SLAM

**Objective**: To implement Graph SLAM and play with gmapping in ROS.

SLAM (Simultainous Localization and Mapping) is techniques for localization of a robot in an unknown environment. I.e. we don't have a map of the world and we don't know the robot's location in the world.

# 1. Implement graph slam only with poses
Implement a function, which can solve the graph SLAM problem only with a continous series of poses. The function should take as input a series of landmarks and poses, and output .

![\my = \Omega^-1\cdot \xi](https://latex.codecogs.com/svg.latex?\my%20=%20\Omega^-1\cdot%20\xi)

You can test using the following robot positions in 1D:
x0=-3
moves by 5
moves by 3

Hereby ![\xi](https://latex.codecogs.com/svg.latex?\xi) should be:
```
-8
2
3
```

and  ![\mu](https://latex.codecogs.com/svg.latex?\mu) should be:
```
-3
2
3
```


HINT: You can create an ![\Omega](https://latex.codecogs.com/svg.latex?\Omega) matrix and ![\xi](https://latex.codecogs.com/svg.latex?\xi) vector for each pairs, which you then add, before inverting and multiplying. I.e. ![\my = \left(\Omega_{x0}+\Omega_{x1}+\Omega_{x2}\right)^-1\cdot \left(\xi_{x0}+\xi_{x3}+\xi_{x2}\right)](https://latex.codecogs.com/svg.latex?\my%20=%20\Omega^-1\cdot%20\xi)

Try adding noise to the transitions and see how it influences ![\mu](https://latex.codecogs.com/svg.latex?\mu).

# 2. Implement graph slam function
The function should take as input a series of landmarks and poses.

# 3. Try playing with the SLAM.py demo
SLAM.py contains a simple robot model and a graph-slam implementation. Try to understand the code and see how landmark estimations changes based on number of observations and sensor noise.

# 4. Mapping in ROS
ROS contain a package called gmapping, which builds on openslam and contain a particle filter slam implementation.
You can launch the gmapping package using the following command:
```
TODO
```

Move the Turtlebot around and see how the environment is being mapped.
