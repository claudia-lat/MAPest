### Note for 23links_human task

This experiment is the first attempt in the physical Human-Robot Interaction (pHRI) field.
The experimental set-up encompasses the following different sensor modules: 
- a wearable suit for the motion tracking,
- two standard force platforms to acquire the ground reaction wrenches, 
- the force/torque sensors of the robot arms. 

The model of the human is more complex with respect to the previous model with 3 links. See in **templates** the detailed description of the model for the human.

### Dependencies
- [iDynTree (with MATLAB bindings support)](https://github.com/robotology/idyntree)
- [OpenSim (APIs for MATLAB)](http://simtk-confluence.stanford.edu:8080/display/OpenSim/Scripting+with+Matlab)
- xml_io_tools (in **external**)

### What you need

1. Since this code works only for off-line computations, you need to have a folder **data** containing:
  - force plate data (a AMTI file .txt with force and moments + a file .txt with the acquisition timestamps);
  - Xsens suit data (a file .mvnx extracted directly from the MVN Studio SW from Xsens + the related file .trc (obtained from the extracted file .c3d, e.g. by using [Mokka](http://biomechanical-toolkit.github.io/mokka/) analyzer);
  -  iCub data (files .txt from robot data dumping).  These data are mandatory if you want to retrieve human dynamics in pHRI experiments;
  -  iCub urdf model.
  - a fileSetup.xml for the OpenSim computation.
  

2.  Folder **fixture** containing the fixture for the fixed position between the robot and the human.

3. Folder **templates** containig human templates for both the URDF and the osim models.  The osim model is mandatory for the OpenSim computation.


### How to use

You are able now to test your estimation only by running the `main.m`. It will call the following functions for you:
- `extractSuitData.m`: extracts data from the Xsens and creates a struct suit.mat
- `computeSuitSensorPosition.m`: computes the position of the sensor with respect to the reference frame of each link. It adds this information to the suit.mat;
- `extractForceplateData.m`: extracts raw data from the force plate acquisition file by creating a struct forceplate.mat;
- `extractRobotData.m`: extracts data from the robot data dumping file and creates a struct robot.mat;
- `dataSync.m`: function for 'manually' synchronizing the data coming from sensors
- `subjectParamsComputation.m`: once defined the mass and the weight of the subject, it computes the sizes for the model bounding boxes by extracting dimensions directly from suit.mat
- `processForceplateWrenches.m`: processes raw external wrenches estimates from forceplate.mat
- `createXsensLikeURDFmodel.m`: by starting from a generic template, it creates a subject model.urdf by considering its non-standard extended version (see [here](https://github.com/robotology/idyntree/blob/master/doc/model_loading.md))
- `createXsensLikeOSIMmodel.m`: creates an osim model by starting from a standard template
- `IK.m`: computes the Inverse Kinematics (IK) by using the APIs of OpenSim.  It extracts the vector (*q*) for the joint angles AND -through a Savitzki-Golay computation- the vectors of joint velocities (*dq*) and accelerations (*ddq*)

Here `iDynTree` comes in!  You can build in this way the humanModel and the sensorModel and initializing your berdy object (more documentation of the BerdyHelper class [here](https://github.com/robotology/idyntree/blob/master/src/estimation/include/iDynTree/Estimation/BerdyHelper.h)).

- `processRobotWrenches.m`: processes raw external wrenches estimates coming from the robot.
- `dataPackaging.m`: creates a data struct organised ( where each sensor measurement is identified by: type, id, measured value and variance)
- `berdyMeasurementsWrapping.m`: orders the measurements in a format compatible with the BerdyHelper class.  It returns a vector of ordered measurements and its associated covariance matrix
- `MAPcomputation.m`: solves the inverse dynamics problem with a Maximum-A-Posteriori estimation by using the Newton-Euler algorithm and redundant sensor measurements. See [here](https://github.com/claudia-lat/MAPest/blob/master/Experiments/23links_human/misc/schemeAlgorithm.png) for a visual overview of what MAP is doing for you.

