# MAPest

MAPest (Maximun-A-Posteriori estimation) is a probabilistic tool for estimating dynamics variables in humans.

The code in this repository works only *off-line* and to solve the problem of the estimation of human dynamics you need to have a dataset of different measurements coming from sensors.
For the moment the repository consists of two type of experiments:
  1. **1st experiment** (2dofBowingTask): where we consider a very simplified human model with 3 link and 2 Dofs and a sensor architecture composed by the Vicon motion capture + an IMU + one forceplate; 
  2. **2nd experiment** (23links_human): where the model is composed by 23 links and for the sensor structure by the Xsens suit + two sensorized shoes.
  
Detailes on the experiments in the README of each related Section.

### Citing this work
Please cite the following publication if you are using the code contained in this repository for your own research and/or experiments:

~~~
Latella, C.; Lorenzini, M.; Lazzaroni, M.; Romano, F.; Traversaro, S.; Akhras, M.A.; Pucci, D.; Nori, F.
Towards Real-time Whole-Body Human Dynamics Estimation through Probabilistic Sensor Fusion Algorithms.
A Physical Human–Robot Interaction Case Study.
Autonomous Robots, Springer US, October 2018, doi:
10.1007/s10514-018-9808-4
https://doi.org/10.1007/s10514-018-9808-4
~~~

The bibtex code for including this citation is provided:

~~~
@Article{Latella2018,
author="Latella, Claudia
and Lorenzini, Marta
and Lazzaroni, Maria
and Romano, Francesco
and Traversaro, Silvio
and Akhras, M. Ali
and Pucci, Daniele
and Nori, Francesco",
title="Towards real-time whole-body human dynamics estimation through probabilistic sensor fusion algorithms",
journal="Autonomous Robots",
year="2018",
month="Oct",
day="31",
issn="1573-7527",
doi="10.1007/s10514-018-9808-4",
url="https://doi.org/10.1007/s10514-018-9808-4"
}
~~~