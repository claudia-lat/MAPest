function [tau] = iDynTreeID_estimClass_floating(filenameURDF, currentBase, jointsQty, baseVel, baseAcc, fext)
%IDYNTREEID_ESTIMCLASS_FLOATING computes the iDynTree estimation of joint 
% torques and external wrenches as implemented in iDynTree C++ 
% ExtWrenchesAndJointTorquesEstimator class.
%
% Input:
% - filenameURDF    : URDF name
% - currentBase     : current floating base
% - jointsQty       : q,dq + ddq
% - baseVel         : 3D velocity of the base expressed w.r.t. the base frame
% - baseAcc         : accelerations of the base  expressed w.r.t. base frame.
%                     1) 3D proper acceleration = base_R_G * (G_a_pelvis - G_g)
%                     2) 3D angular acceleration
% - fext            : 6D wrench expressed w.r.t. the link they are applied.
%
% Output:
% - tau             : torque


%% Load the estimator
estimator = iDynTree.ExtWrenchesAndJointTorquesEstimator();

% Load model and sensors from the URDF file
estimator.loadModelAndSensorsFromFile(filenameURDF);

% Check if the model was correctly created by printing the model
% estimator.model().toString() % print model

% estimatorLoader  = iDynTree.ModelLoader();
% estimatorLoader.loadReducedModelFromFile(bucket.filenameURDF, ...
%         cell2iDynTreeStringVector(selectedJoints))
% estimatorModel = estimatorLoader.model();
% estimatorModel.toString();
% estimatorSensors = estimatorLoader.sensors();
% estimator.setModelAndSensors(estimatorModel,estimatorSensors);

%
% % Create KinDynComputations class variable
% kinDyn_test = iDynTree.KinDynComputations();
% kinDyn_test.loadRobotModel(estimator.model);

% nrOfFTSensors = estimator.sensors().getNrOfSensors(iDynTree.SIX_AXIS_FORCE_TORQUE);
% nrOfAccSensors = estimator.sensors().getNrOfSensors(iDynTree.ACCELEROMETER_SENSOR);

% Set kinematics information
% gravity = iDynTree.Vector3();
% gravity.fromMatlab([0; 0; -9.81]);

%% Set joint information
dofs = estimator.model().getNrOfDOFs();
q_idyn   = iDynTree.JointPosDoubleArray(dofs);
dq_idyn  = iDynTree.JointDOFsDoubleArray(dofs);
ddq_idyn = iDynTree.JointDOFsDoubleArray(dofs);

baseProperAcc_idyn = iDynTree.Vector3();
baseAngVel_idyn    = iDynTree.Vector3();
baseAngAcc_idyn    = iDynTree.Vector3();

base_index   = estimator.model().getFrameIndex(currentBase);
l_foot_index = estimator.model().getFrameIndex('LeftFoot');
r_foot_index = estimator.model().getFrameIndex('RightFoot');

%% Set estimation inputs

% 1) INPUT 1: Wrenches
% ~~~~~~~~~~~~~~~~~~~~~~ Unknown wrench at the base
% We need to set the location of the unknown wrench. We express the unknown
% wrench at the origin of the base frame
unknownWrench_base = iDynTree.UnknownWrenchContact();
unknownWrench_base.unknownType = iDynTree.FULL_WRENCH;
% the position is the origin, so the conctact point wrt to base is zero
unknownWrench_base.contactPoint.zero();

% ~~~~~~~~~~~~~~~~~~~~~~ External known wrenches at the feet
% If I want to pass an external wrench here, I've to assume the 
% `the unknonw is of NO_UNKNOWNS enum type`!

% Left foot wrench init
unknownWrench_lFoot = iDynTree.UnknownWrenchContact();
unknownWrench_lFoot.unknownType = iDynTree.NO_UNKNOWNS;
% the position is the origin, so the conctact point wrt to LeftFoot is zero
% unknownWrench_rFoot.contactPoint.zero();

% Right foot wrench init
unknownWrench_rFoot = iDynTree.UnknownWrenchContact();
unknownWrench_rFoot.unknownType = iDynTree.NO_UNKNOWNS;
% unknownWrench_rFoot.contactPoint.zero();

% Storing wrenches in fullBodyUnknowns storage
fullBodyUnknowns = iDynTree.LinkUnknownWrenchContacts(estimator.model());
fullBodyUnknowns.clear();

% 2) INPUT 2: The estimated FT sensor measurements
estFTmeasurements = iDynTree.SensorsMeasurements(estimator.sensors());

% This variable should be filled with the method:
% estFTmeasurements.setMeasurement(iDynTree.SIX_AXIS_FORCE_TORQUE,ftIndex,data);
% But this is possible if in the model sensors of
% iDynTree.SIX_AXIS_FORCE_TORQUE type are defined.  
% NOTE: In the human URDF model we don't have them!!

%% Set estimation outputs

estContactForces = iDynTree.LinkContactWrenches(estimator.model());
estJointTorques  = iDynTree.JointDOFsDoubleArray(dofs);
tau = zeros(size(jointsQty.q));

%% Estimation
% samples = size(jointsQty.q,2);
samples = 10; % tmp test number

for i = 1 : samples
    q_idyn.fromMatlab(jointsQty.q(:,i));
    dq_idyn.fromMatlab(jointsQty.dq(:,i));
    ddq_idyn.fromMatlab(jointsQty.ddq(:,i));
    
    baseProperAcc_idyn.fromMatlab(baseAcc.baseProperAcc_wrtBase(:,i));
    baseAngVel_idyn.fromMatlab(baseVel(:,i));
    baseAngAcc_idyn.fromMatlab(baseAcc.baseAngAcc_wrtBase(:,i));
    
    estimator.updateKinematicsFromFloatingBase(q_idyn,dq_idyn,ddq_idyn,base_index, ...
        baseProperAcc_idyn, baseAngVel_idyn,baseAngAcc_idyn);
    
    shoesWrench_lFoot = unknownWrench_lFoot.knownWrench;
    shoesWrench_lFoot.fromMatlab(fext.Left_HF(:,i));
    shoesWrench_rFoot = unknownWrench_rFoot.knownWrench;
    shoesWrench_rFoot.fromMatlab(fext.Right_HF(:,i));
    
    fullBodyUnknowns.addNewContactInFrame(estimator.model(),base_index,unknownWrench_base);
    fullBodyUnknowns.addNewContactInFrame(estimator.model(),l_foot_index,unknownWrench_lFoot);
    fullBodyUnknowns.addNewContactInFrame(estimator.model(),r_foot_index,unknownWrench_rFoot);
    fullBodyUnknowns.toString(estimator.model()); %print the unknowns
    
    estimator.estimateExtWrenchesAndJointTorques(fullBodyUnknowns,...
        estFTmeasurements,...
        estContactForces,...
        estJointTorques);
    
% % %     % [Print] Wrenches values can easily be obtained as matlab vectors
% % %     estContactForcesExtWrenchesEst.contactWrench(estimator.model().getLinkIndex('LeftFoot'),0).contactWrench().getLinearVec3().toMatlab();
% % %     estContactForcesExtWrenchesEst.contactWrench(estimator.model().getLinkIndex('RightFoot'),0).contactWrench().getLinearVec3().toMatlab();
% % %     
% % %     % LinkContactWrenches is a structure that can contain multiple contact wrench for each link,
% % %     % but usually is convenient to just deal with a collection of net wrenches for each link
% % %     linkNetExtWrenches = iDynTree.LinkWrenches(estimator.model());
% % %     estContactForcesExtWrenchesEst.computeNetWrenches(linkNetExtWrenches);
% % %     
% % %     % also net external wrenches can easily be obtained as matlab vectors
% % %     wrench = linkNetExtWrenches(estimator.model().getLinkIndex('LeftFoot'));
% % %     % 6d wrench (force/torques)
% % %     wrench.toMatlab();
% % %     % just the force
% % %     wrench.getLinearVec3().toMatlab();

    tau(:,i) = estJointTorques.toMatlab();
end
end
