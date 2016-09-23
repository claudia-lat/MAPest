mdlLoader = iDynTree.ModelLoader();

% Load model with just legs 
consideredJoints = iDynTree.StringVector();
consideredJoints.push_back('l_hip_pitch');
consideredJoints.push_back('l_hip_roll');
consideredJoints.push_back('l_hip_yaw');
consideredJoints.push_back('l_knee');
consideredJoints.push_back('l_ankle_pitch');
consideredJoints.push_back('l_ankle_roll');
consideredJoints.push_back('r_hip_pitch');
consideredJoints.push_back('r_hip_roll');
consideredJoints.push_back('r_hip_yaw');
consideredJoints.push_back('r_knee');
consideredJoints.push_back('r_ankle_pitch');
consideredJoints.push_back('r_ankle_roll');


mdlLoader.loadReducedModelFromFile('../data/iCubGenova02.urdf',consideredJoints);

kinDynMdl = iDynTree.KinDynComputations();
kinDynMdl.loadRobotModel(mdlLoader.model());

% Consideter joint positions : homePosTwoFeetBalancing
% see https://github.com/robotology/codyco-modules/issues/202
dq = iDynTree.JointDOFsDoubleArray(mdlLoader.model());
dq.zero();
gravity = iDynTree.Vector3();
gravity.zero();

q = iDynTree.JointPosDoubleArray(mdlLoader.model());
qMat = q.toMatlab();
% left leg 
qMat(1) = deg2rad(12.0);
qMat(2) = deg2rad(5.0);
qMat(3) = deg2rad(0.0);
qMat(4) = deg2rad(-10.0);
qMat(5) = deg2rad(-3.0);
qMat(6) = deg2rad(-5.0);
% right leg 
qMat(7) = deg2rad(12.0);
qMat(8) = deg2rad(5.0);
qMat(9) = deg2rad(0.0);
qMat(10) = deg2rad(-10.0);
qMat(11) = deg2rad(-4.0);
qMat(12) = deg2rad(-5.0);

q.fromMatlab(qMat);

% Set state 
kinDynMdl.setRobotState(q,dq,gravity);

% Compute l_sole_H_r_sole 
fprintf('l_sole_H_r_sole:\n');
kinDynMdl.getRelativeTransform('l_sole','r_sole').asHomogeneousTransform().toMatlab()


