function [tau] = iDynTreeID_estimClass_floating(filenameURDF, selectedJoints, currentBase, jointsQty, baseVel, baseAcc, fext)
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
estimator.loadModelAndSensorsFromFileWithSpecifiedDOFs(filenameURDF,selectedJoints,'urdf');

% Check if the model was correctly created by printing the model
estimator.model().toString() % print model

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
unknownWrench_lFoot.contactPoint.zero();

% Right foot wrench init
unknownWrench_rFoot = iDynTree.UnknownWrenchContact();
unknownWrench_rFoot.unknownType = iDynTree.NO_UNKNOWNS;
unknownWrench_rFoot.contactPoint.zero();

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
    
    fullBodyUnknowns.addNewUnknownFullWrenchInFrameOrigin(estimator.model(),base_index);
    fullBodyUnknowns.addNewContactInFrame(estimator.model(),l_foot_index,unknownWrench_lFoot);
    fullBodyUnknowns.addNewContactInFrame(estimator.model(),r_foot_index,unknownWrench_rFoot);
    fullBodyUnknowns.toString(estimator.model()) %print the unknowns

    estimator.estimateExtWrenchesAndJointTorques(fullBodyUnknowns,...
        estFTmeasurements,...
        estContactForces,...
        estJointTorques);

    tau(:,i) = estJointTorques.toMatlab();
    
    %% Print the estimated external forces
    
    fprintf('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n');
    fprintf('External wrenches estimated');
    fprintf('%s',estContactForces.toString(estimator.model()));
    
    % LinkContactWrenches is a structure that can contain multiple contact wrench for each link,
    % but usually is convenient to just deal with a collection of net wrenches for each link
    linkNetExtWrenches = iDynTree.LinkWrenches(estimator.model());
    estContactForces.computeNetWrenches(linkNetExtWrenches);
    
    %PELVIS
    wrench_Pelvis = linkNetExtWrenches(estimator.model().getLinkIndex('Pelvis'));
    % 6d wrench (force/torques)
    Pelvis_wrench = wrench_Pelvis.toMatlab()
    % just the force
    %     wrench_LF.getLinearVec3().toMatlab()
    
    %LEFT_FOOT
    wrench_LF = linkNetExtWrenches(estimator.model().getLinkIndex('LeftFoot'));
    % 6d wrench (force/torques)
    LF_wrench = wrench_LF.toMatlab()
    % just the force
    %     wrench_LF.getLinearVec3().toMatlab()
    
    %RIGHT_FOOT
    wrench_RF = linkNetExtWrenches(estimator.model().getLinkIndex('RightFoot'));
    % 6d wrench (force/torques)
    RF_wrench = wrench_RF.toMatlab()
    % just the force
    %     wrench_RF.getLinearVec3().toMatlab()
end
end
