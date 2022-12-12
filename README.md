# Bezier Curves Tracking with Application in Mobile Robots

## 1. Problem Statement

In mobile robot applications, it is often the case that we already setup a network of roads for robots. The road network can be used as a graph for a graph-based search algorithm such as A*, which would find a feasible path for a robot to reach its destination. Now, the remaining problem is that:

Given a predefined path for a robot to follow, how can we develop a path planner which can drive the robot to **approach** and **track** the path while respecting the robot's **turning radius** constraint?


Therefore, this project is an attempt to create such a path planner for the above problems. 

## 2. Pursuit Curves
The path planner is inspired by the concept of [pursuit curve](https://en.wikipedia.org/wiki/Pursuit_curve), which is constructed by analogy to having a point(pursuer) pursuing another point(pursuee).

<p align="center">
  <img src="videos/pursuit_curve.gif" />
</p>

From the above animation, the black point is the pursuee, the red point is the purseer and the red curve is called a pursuit curve. The pursuee is minding its business following a straight line, while the pursuer is trying to catch the pursuee by always pointing its velocity toward the pursuee. Regardless of its intial position, the pursueer will eventually converge to the path of the pursuee, and thus we can embed this behavior into our path planner. 

Howover, the dynamics of pursuit curve does not take into account the heading as well as the mininum turning radius of mobile robots, espectially for nonholonomic robots. This issue can be addressed by constraining the pursuer's instantaneous center of rotation (ICR) such that the radius would be at least the robot's minimum turning radius. 

<p align="center">
  <img src="videos/ICR.PNG" />
</p>


Assume that the robot is going at a constant velocity $v$, and we are working in discrete time environment with sampling time $T_s$. Therefore, the length of each segment is:

$$ L = v*T_s $$

The black segments represent the robot(pursuer)'s trajectory, and the arrows are the robot's velocity/heading directions. Given robot's current heading, $theta_{max}$ is the allowed range that the robot's velocity vector can deviate. 

$$ theta_{max} = \arctan({L \over R_{min}}) $$

Notice that the smaller the robot's mininum turning radius $R_{min}$, the larger the allowed maximum angle of deviation $theta_{max}$. 

Therefore, if the pursuer's velocity vector exceeds the maximum allowed deviation from its current heading, then we can just clamp the velocity vector at the max allowed direction. This will ensure the robot's minimum turning radius to be satisfied at all time. 

## 3. Bezier Curves

## 4. Path Planner

## 5. Demo
<!-- ![Alt Text](videos/linearbezier_demo1.gif)
![Alt Text](videos/linearbezier_demo2.gif)
![Alt Text](videos/linearbezier_demo3.gif)
![Alt Text](videos/linearbezier_demo4.gif) -->

