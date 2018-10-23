## MAPest with 23link_human

The code into this section is useful to compute the offline human dynamic estimation.

The experimental set-up encompasses the following different sensor modules: 

- a wearable suit for the motion tracking (i.e., Xsens suit)
- two sensorized shoes equipped with force/torque sensors to acquire the ground reaction wrenches

The code can be adapted to fit other sensor modules like

- forceplates (instead of/in addiction to the shoes)
- robot sensors

The model of the human is more complex with respect to the [previous 3-link model](https://github.com/claudia-lat/MAPest/tree/master/Experiments/2dofBowingTask/models). Refer to the [templates section](https://github.com/claudia-lat/MAPest/tree/master/Experiments/23links_human/templates) to know more about the 23-link model of the human.

### Dependencies
- [iDynTree (with MATLAB bindings support)](https://github.com/robotology/idyntree)
- [OpenSim (APIs for MATLAB)](http://simtk-confluence.stanford.edu:8080/display/OpenSim/Scripting+with+Matlab)

### What you need

1. A data folder (e.g., `dataExperiment`) containing your own dataset, per each subject (e.g., `S0X`) and each task (e.g., `TaskY`).  Per each task, two folders  structured as follows:

   **Folder: `data`**
   - `S0X_0Y.trc`:trc file comming from Xsens acquisition
   - `parsedFromMvnx`: a subfolder containing the following files parsed from the original Xsens .mvnx file [via C++ parser](https://github.com/robotology-playground/xsens-mvn/tree/master/mvnxparser):
      - a file `.xml` (generic info, points, identity/tpose/tpose-isb)
      - a `.log` file (segment and sensor list)
      - a `.csv` file (index, msTime, xSensTime, each link (acceleration, orientation, angular velocity, angular acceleration), each sensor (orientation, free acceleration))

   **Folder: `processed`**
      - a folder in which the code will automatically store all the processed data.

3. **Folder `templates`** containig human templates for both the URDF and the osim models.  The osim model and the `setupOpenSimIKTool_Template.xml` files are mandatory for the OpenSim computation.


### How to use

You are able now to use the code only by running `configureAndRunMAPest.m` that in turn calls directly `main.m`.

This script performs the following actions:

- creates a Matlab struct `suit.mat` (via `extractSuitDataFromParsing.m`, `computeSuitSensorPosition`). This is UNA TANTUM procedure.

- creates URDF and osim models (via `subjectParamsComputation.m`, `createXsensLikeURDFmodel.m`, `createXsensLikeOSIMmodel.m`)

- computes the OpenSim-based Inverse Kinematics (via `IK.m`)

- handles the synchronization of the data (via `rawDataHandling` script).  This section is highly user dependent.  Here you should have a collection of subfunctions/subscripts that allow you to synchronize your data (e.g., state, forceplates, ftShoes, robot)

- transforms data measurements from sensor reference frames into human reference frames (e.g., `transformShoesWrenches.m`)

- loads and initialiazes `berdy`.  Here *iDynTree* dependency comes in!  You can build in this way the humanModel and the sensorModel and initializing your `berdy` object (more documentation of the BerdyHelper class [here](https://github.com/robotology/idyntree/blob/master/src/estimation/include/iDynTree/Estimation/BerdyHelper.h))

- wraps the measurements in a format compatible with the BerdyHelper class (via `dataPackaging.m`,`berdyMeasurementsWrapping.m`)

- computes the base angular velocity required from the floating-base computation of the MAP (via `computeBaseAngularVelocity.m`)

- solves the estimation problem with the MAP algorithm (via `MAPcomputation_floating.m`)

- extracts variables from the estimated vector (e.g., external forces `extractEstimatedFext_from_mu_dgiveny.m` and torques `extractEstimatedTau_from_mu_dgiveny`)

- simulates the measurement vector useful for further validation of the MAP computation (via `sim_y_floating.m`)
