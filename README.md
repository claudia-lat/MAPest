# MAPest

MAPest (Maximun-A-Posteriori estimation) is a probabilistic tool for estimating dynamics variables in humans.  For a detailed theoretical description see  *Latella, C.; Kuppuswamy, N.; Romano, F.; Traversaro, S.; Nori, F.	Whole-Body Human Inverse Dynamics with Distributed Micro-Accelerometers, Gyros and Force Sensing. Sensors 2016, 16, 727*, http://www.mdpi.com/1424-8220/16/5/727. 

###Dependencies
- [iDynTree (with MATLAB bindings support)](https://github.com/robotology/idyntree)
- OpenSim (APIs for MATLAB) 
- xml_io_tools (in *external*)

###What you need

1. Since this code works only for off-line computations, you need to have a folder **data** containing:
  - force plate data (a AMTI file .txt with force and moments + a file .txt with the acquisition timestamps);
  - Xsens suit data (a file .mvnx extracted directly from the MVN Studio SW from Xsens + the related file .trc (obtained from the extracted file .c3d, i.e, by using [Mokka](http://biomechanical-toolkit.github.io/mokka/) analyzer);
  - [*optional_HRI*] iCub data (files .txt from robot data dumping).  These data are mandatory if you want to retrieve human dynamicsin Human-Robot Interaction (HRI) experiments;
  - [*optional_HRI*] iCub urdf model.
  - a fileSetup.xml for the OpenSim computation.
  

2. [*optional_HRI*] Folder **fixture** containing the fixture for the fixed position between the robot and the human.

3. Folder **templates** containig human templates for both the URDF and the osim models.  The osim model is mandatory for the OpenSim computation.


###How to use

You are able now to test your estimation only by running the `main.m`. It will call the following functions for you:
- `extractSuitData.m`: extracts data from the Xsens and creates a struct suit.mat
- `computeSuitSensorPosition.m`: computes the position of the sensor with respect to the reference frame of each link. It adds this information to the suit.mat;
- `extractForceplateData.m`: extracts raw data from the force plate acquisition file by creating a struct forceplate.mat;
- `extractRobotData.m`: extracts data from the robot data dumping file and creates a struct robot.mat;
- `dataSync.m`: function for 'manually' synchronizing the data coming from sensors
- `subjectParamsComputation.m`: once defined the mass and the weight of the subject, it computes the sizes for the model bounding boxes by extracting dimensions directly from suit.mat
- `processForceplateWrenches.m`: processes raw external wrenches estimates from forceplate.mat
- `createXsensLikeURDFmodel.m`: by starting from a generic template, it creates a subject model.urdf by considering its non-standard extended version (see [here](https://github.com/robotology/idyntree/blob/master/doc/model_loading.md)). See in **templates** the detailed description of the model for the humans
- `createXsensLikeOSIMmodel.m`: creates an osim model by starting from a standard template
- `IK.m`: computes the Inverse Kinematics (IK) by using the APIs of OpenSim.  It extracts the vector (*q*) for the joint angles AND -through a Savitzki-Golay computation- the vectors of joint velocities (*dq*) and accelerations (*ddq*)

Here comes in `iDynTree`!  You can build in this way the humanModel and the sensorModel and initializing your berdy object (more documentation of the BerdyHelper class [here](https://github.com/robotology/idyntree/blob/master/src/estimation/include/iDynTree/Estimation/BerdyHelper.h)).

- `processRobotWrenches.m`
- `dataPackaging.m`: creates a data struct organised ( where each sensor measurement is identified by: type, id, measured value and variance)
- `berdyMeasurementsWrapping.m`: orders the measurements in a format compatible with the BerdyHelper class.  It returns a vector of ordered measurements and its associated covariance matrix
- `MAPcomputation.m`
