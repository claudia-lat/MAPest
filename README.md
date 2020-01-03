# MAPest

MAPest (Maximun-A-Posteriori estimation) is a probabilistic tool for estimating dynamics variables in humans.

The code in this repository works only *off-line* and to solve the problem of the estimation of human dynamics you need to have a dataset of different measurements coming from sensors.
For the moment the repository consists of two type of experiments:
  1. **1st experiment** (2dofBowingTask): where we consider a very simplified human model with 3 link and 2 Dofs and a sensor architecture composed by the Vicon motion capture + an IMU + one forceplate; 
  2. **2nd experiment** (23links_human): where the model is composed by 23 links and for the sensor structure by the Xsens suit + two sensorized shoes.
  
Detailes on the experiments in the README of each related Section.

### Citing this work
Please cite the following publications if you are using the code contained in this repository for your own research and/or experiments:

~~~
Latella, C.; Traversaro, S.; Ferigo, D.; Tirupachuri, Y.; Rapetti, L.; 
Andrade Chavez, F.J.; Nori, F.; Pucci, D. Simultaneous Floating-Base 
Estimation of Human Kinematics and Joint Torques. Sensors 2019, 19, 2794
doi: https://doi.org/10.3390/s19122794
~~~
https://www.mdpi.com/1424-8220/19/12/2794

and

~~~
Latella, C. Human Whole-Body Dynamics Estimation for Enhancing Physical Human-Robot Interaction
~~~
https://arxiv.org/abs/1912.01136
