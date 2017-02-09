# MAPest

MAPest (Maximun-A-Posteriori estimation) is a probabilistic tool for estimating dynamics variables in humans. For a detailed theoretical description see in the paper: *Latella, C.; Kuppuswamy, N.; Romano, F.; Traversaro, S.; Nori, F.	Whole-Body Human Inverse Dynamics with Distributed Micro-Accelerometers, Gyros and Force Sensing. Sensors 2016, 16, 727*, http://www.mdpi.com/1424-8220/16/5/727. 

The code in this repository works only *off-line* and to solve the problem of the estimation of human dynamics you need to have a dataset of different measurements coming from sensors.
For the moment the repository consists of two type of experiments:
  1. **1st experiment** (2dofBowingTask): where we consider a very simplified human model with 3 link and 2 Dofs and a sensor architecture composed by the Vicon motion capture + an IMU + one forceplate; 
  2. **2nd experiment** (23links_human): where the model is composed by 23 links and for the sensor structure by the Xsens suit + two forceplates.
  
Detailes on the experiments in the README of each related Section.
