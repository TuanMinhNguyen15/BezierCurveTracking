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

From the above animation, the black point is the pursuee, the red point is the purseer and the red curve is called a pursuit curve. The pursuee is minding its own business following a straight line, while the pursuer is trying to catch the pursuee by always pointing its velocity vector toward the pursuee. Regardless of intial position, the pursuer will eventually converge to the path of the pursuee. We can embed this behavior into our path planner by representing the **pursuer as our mobile robot** and the **pursuee as the ideal robot** that we want our robot to converge to. 

Howover, the dynamics of pursuit curve does not take into account the heading as well as the mininum turning radius of mobile robots, espectially for nonholonomic robots. This issue can be addressed by constraining the pursuer's instantaneous center of rotation (ICR) such that the radius would always be at least the robot's minimum turning radius. 

<!-- <p align="center">
  <img src="videos/ICR.PNG" />
</p> -->


Assume that the robot is going at a constant velocity $v$, and we are working in discrete time environment with sampling time $T_s$. Therefore, the length of each segment is:

$$ L = v*T_s $$

The black segments represent the robot (pursuer)'s trajectory, and the arrows are the robot's velocity/heading directions. Given robot's current heading, $theta_{max}$ is the allowed range that the robot's velocity vector can deviate. 

$$ theta_{max} = \arctan({L \over R_{min}}) $$

Notice that the smaller the robot's mininum turning radius $R_{min}$, the larger the allowed maximum angle of deviation $theta_{max}$. 

Therefore, if the pursuer's velocity vector exceeds the maximum allowed deviation from its current heading, then we can just clamp the velocity vector at the max allowed direction. This will ensure the robot's minimum turning radius to be satisfied at all time. 

## 3. Bezier Curves
Now that a robot can asymptotically converge to its "ideal" version using the dynamics of pursuit curves. The next step is to formulate the path of the ideal robot. The path of the pursee (ideal robot) can be easily specfied using [Bezier curves](https://en.wikipedia.org/wiki/B%C3%A9zier_curve). In short, a Bezier curve is a smooth parametric curve which is defined by a set of control points. Points ($P$) along a Bezier curve can be expressed by an equation of a parametric variable ($\lambda$) and control points ($P_0,P_1,...$). 

$$P = f(\lambda,P_0,P_1,...)$$

There are 3 common Bezier curves:

* Linear Bezier
<p align="center">
  <img src="videos/linearbezier.PNG" />
</p>

$$P = (1-\lambda)P_0 + \lambda P_1$$
* Quadratic Bezier
<p align="center">
  <img src="videos/quadraticbezier.PNG" />
</p>

$$P=(1-\lambda)^2P_0 + 2(1-\lambda)\lambda P_1 + \lambda^2 P_2$$
* Cubic Bezier 
<p align="center">
  <img src="videos/cubicbezier.PNG" />
</p>

$$P = (1-\lambda)^3 P_0 + 3(1-\lambda)^2\lambda P_1 + 3(1-\lambda)\lambda^2 P_2 + \lambda^3 P_3$$

By specifying the control points and combining Bezier curves piecewise, we can formulate any path for a robot to track. The only thing left is to ensure the pursuee is going at a desired speed:

 Since control points are already specfied, points along a Bezier curve now depend only on $\lambda$

 $$P = f(\lambda) = [P_x,P_y]^T$$

 Let $P_x = h(\lambda)$ and $P_y = k(\lambda)$

 $$ \dot{P_x} = \frac{dh}{d\lambda}*\dot{\lambda} $$
 $$ \dot{P_y} = \frac{dk}{d\lambda}*\dot{\lambda} $$

 Given we want the pursuee (ideal robot) to have a velocity $V$

 $$ V^2 = \dot{P_x}^2 + \dot{P_y}^2 = (\frac{dh}{d\lambda}^2 + \frac{dk}{d\lambda}^2)*\dot{\lambda}^2$$

 $$ \dot{\lambda} = \sqrt{\frac{V^2}{\frac{dh}{d\lambda}^2 + \frac{dk}{d\lambda}^2}} $$

Using the above equation, we can always determine $\dot{\lambda}$ at any given $\lambda$ such that the ideal robot maintains a desired velocity $V$. Additionally, after $\dot{\lambda}$ is known, we can also find the heading direction of the ideal robot by calculating $[\dot{P_x} , \dot{P_x}]^T$. This heading direction will be very useful  in the next section. 

## 4. Path Planner

## 5. Demo
<!-- ![Alt Text](videos/linearbezier_demo1.gif)
![Alt Text](videos/linearbezier_demo2.gif)
![Alt Text](videos/linearbezier_demo3.gif)
![Alt Text](videos/linearbezier_demo4.gif) -->

