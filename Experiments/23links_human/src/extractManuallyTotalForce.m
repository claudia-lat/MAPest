function [ shoes ] = extractManuallyTotalForce(pathToSubject, shoes)
%extractManuallyTotalForce  provides manually the filed shoes.totalForce

%% Elaborate Offset
shoesOff = struct;

% LEFT---------------------------------------------------------------------    
bucket.leftShoe_frontForce_off = (fullfile(pathToSubject, ...
    '/ftShoeDriver_Left_offset/frontForce/data.log'));
bucket.leftShoe_rearForce_off  = (fullfile(pathToSubject,...
    '/ftShoeDriver_Left_offset/rearForce/data.log'));
% RIGHT--------------------------------------------------------------------
bucket.rightShoe_frontForce_off = (fullfile(pathToSubject, ...
    '/ftShoeDriver_Right_offset/frontForce/data.log'));
bucket.rightShoe_rearForce_off  = (fullfile(pathToSubject,...
    '/ftShoeDriver_Right_offset/rearForce/data.log'));

% $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
% COMPUTATION OF THE OFFSET FROM SHOES CALIB
% $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
%% Parse shoesOff measurements
% LEFT---------------------------------------------------------------------
shoesOff.Left.frameRate  = 100; %100Hz
shoesOff.Left.frontForce = parseYARPftShoes_fromDriver(bucket.leftShoe_frontForce_off);
shoesOff.Left.rearForce  = parseYARPftShoes_fromDriver(bucket.leftShoe_rearForce_off);
% RIGHT--------------------------------------------------------------------
shoesOff.Right.frameRate  = 100; %100Hz
shoesOff.Right.frontForce = parseYARPftShoes_fromDriver(bucket.rightShoe_frontForce_off);
shoesOff.Right.rearForce  = parseYARPftShoes_fromDriver(bucket.rightShoe_rearForce_off);

%% Synchronize YARP data (Front,Rear) within each shoe for shoesOff
shoesOff.Left.single_synch  = synchroIndividualShoe(shoesOff.Left);
shoesOff.Right.single_synch = synchroIndividualShoe(shoesOff.Right); 

%% Load calib matrices from shoes
matrices_left_calib   = load('calibMatrices/matrices_left.mat');
matrices_right_calib  = load('calibMatrices/matrices_right.mat');

% LEFT---------------------------------------------------------------------
calibMatrices.left.front  = matrices_left_calib.calibMatricesDriver.ftShoe_left_front;
calibMatrices.left.rear   = matrices_left_calib.calibMatricesDriver.ftShoe_left_rear;
% RIGHT--------------------------------------------------------------------
calibMatrices.right.front = matrices_right_calib.calibMatricesDriver.ftShoe_right_front;
calibMatrices.right.rear  = matrices_right_calib.calibMatricesDriver.ftShoe_right_rear;

%% Compute wrenchesOff with new calibration matrices
% we want to obtain: frontForceOUT = calibMatrixFront * frontForceIN and
% rearForceOUT = calibMatrixRear * rearForceIN for both Left and Right

% LEFT---------------------------------------------------------------------
shoesOff.Left.frontWrench = zeros(6,size(shoesOff.Left.single_synch.frontForce.time,2));
shoesOff.Left.rearWrench  = zeros(6,size(shoesOff.Left.single_synch.rearForce.time,2));
for i = 1 : size(shoesOff.Left.single_synch.frontForce.time,2)
  tmp_wrench_left = [shoesOff.Left.single_synch.frontForce.forces(:,i);...
                     shoesOff.Left.single_synch.frontForce.moments(:,i)];
  shoesOff.Left.frontWrench(:,i) = calibMatrices.left.front * tmp_wrench_left;
  shoesOff.Left.frontWrench(:,i) = -1 * shoesOff.Left.frontWrench(:,i);
  tmp_wrench_left = [shoesOff.Left.single_synch.rearForce.forces(:,i);...
                     shoesOff.Left.single_synch.rearForce.moments(:,i)];
  shoesOff.Left.rearWrench(:,i) = calibMatrices.left.rear * tmp_wrench_left;
  shoesOff.Left.rearWrench(:,i) = -1 * shoesOff.Left.rearWrench(:,i);
end
% RIGHT--------------------------------------------------------------------
shoesOff.Right.frontWrench = zeros(6,size(shoesOff.Right.single_synch.frontForce.time,2));
shoesOff.Right.rearWrench  = zeros(6,size(shoesOff.Right.single_synch.rearForce.time,2));
for i = 1 : size(shoesOff.Right.single_synch.frontForce.time,2)
  tmp_wrench_right = [shoesOff.Right.single_synch.frontForce.forces(:,i);...
                     shoesOff.Right.single_synch.frontForce.moments(:,i)];
  shoesOff.Right.frontWrench(:,i) = calibMatrices.right.front * tmp_wrench_right;
  shoesOff.Right.frontWrench(:,i) = -1 * shoesOff.Right.frontWrench(:,i);
  tmp_wrench_right = [shoesOff.Right.single_synch.rearForce.forces(:,i);...
                     shoesOff.Right.single_synch.rearForce.moments(:,i)];
  shoesOff.Right.rearWrench(:,i) = calibMatrices.right.rear * tmp_wrench_right;
  shoesOff.Right.rearWrench(:,i) = -1 * shoesOff.Right.rearWrench(:,i);
end

%% Transform both wrenches into a new SoR -->TotalForce
gravityZero = iDynTree.Vector3();
gravityZero.zero();

% ------------- 1step
% Transform wrenches from front into rear --> only translation, no rot
rear_R_front = iDynTree.Rotation();
rear_R_front.fromMatlab(eye(3));
rear_T_frontPos = iDynTree.Position();
distance_front_rear = 0.145; 
frontSeenFromRear = [-distance_front_rear*sin(pi/6); -distance_front_rear*cos(pi/6) ; 0];
rear_T_frontPos.fromMatlab(frontSeenFromRear);

rear_T_front = iDynTree.Transform(rear_R_front,rear_T_frontPos);

% In the following for loops the two forces are summed
% LEFT---------------------------------------------------------------------
left_intermediate_wrench = zeros(6,size(shoesOff.Left.single_synch.frontForce.time,2));
for i = 1 : size(shoesOff.Left.single_synch.frontForce.time,2)
    tmp_frontWrench = rear_T_front.asAdjointTransformWrench().toMatlab()* ...
                                        shoesOff.Left.frontWrench(:,i);
    left_intermediate_wrench(:,i) = tmp_frontWrench + shoesOff.Left.rearWrench(:,i);
end
% RIGHT--------------------------------------------------------------------
right_intermediate_wrench = zeros(6,size(shoesOff.Right.single_synch.frontForce.time,2));
for i = 1 : size(shoesOff.Right.single_synch.frontForce.time,2)
    tmp_frontWrench = rear_T_front.asAdjointTransformWrench().toMatlab()* ...
                                        shoesOff.Right.frontWrench(:,i);
    right_intermediate_wrench(:,i) = tmp_frontWrench + shoesOff.Right.rearWrench(:,i);
end

% ------------- 2step
% Transform wrenches from step1 front into rear --> only rotation,
% no trasl
total_R_rear = iDynTree.Rotation();
Rz_m120 = [cos(-120/180*pi), -sin(-120/180*pi), 0;
         sin(-120/180*pi),  cos(-120/180*pi), 0;
          0              ,  0           , 1];
Rx_180 = [ 1 ,       0,        0 ;
           0 , cos(pi), -sin(pi) ;
           0 , sin(pi),  cos(pi)];
total_R_rear.fromMatlab(Rz_m120 * Rx_180);   
total_T_rearPos = iDynTree.Position();
total_T_rearPos.fromMatlab([0.0; 0.0; 0.0]);

total_T_rear = iDynTree.Transform(total_R_rear,total_T_rearPos);
% LEFT---------------------------------------------------------------------
shoesOff.Left.totalWrench = zeros(6,size(left_intermediate_wrench,2));
for i = 1 : size(left_intermediate_wrench,2)
     shoesOff.Left.totalWrench(:,i) = total_T_rear.asAdjointTransformWrench().toMatlab()* ...
                                        left_intermediate_wrench(:,i);
end
% % RIGHT--------------------------------------------------------------------
shoesOff.Right.totalWrench = zeros(6,size(right_intermediate_wrench,2));
for i = 1 : size(right_intermediate_wrench,2)
    shoesOff.Right.totalWrench(:,i) = total_T_rear.asAdjointTransformWrench().toMatlab()* ...
                                        right_intermediate_wrench(:,i);
end

% Compute the average offset among all the calibration trial samples
shoesOff.Left.meanOff = mean(shoesOff.Left.totalWrench,2);
shoesOff.Right.meanOff = mean(shoesOff.Right.totalWrench,2);

% $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
% DATA REPROCESSING WITH SHOES OFFSET
% $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
%% Now reprocess the trial data
%% Compute wrenches with new calibration matrices
% we want to obtain: frontForceOUT = calibMatrixFront * frontForceIN and
% rearForceOUT = calibMatrixRear * rearForceIN for both Left and Right

% LEFT---------------------------------------------------------------------
shoes.Left.single_synch.frontWrench = zeros(6,size(shoes.Left.single_synch.frontForce.time,2));
shoes.Left.single_synch.rearWrench  = zeros(6,size(shoes.Left.single_synch.rearForce.time,2));
for i = 1 : size(shoes.Left.single_synch.frontForce.time,2)
  tmp_wrench_left = [shoes.Left.single_synch.frontForce.forces(:,i);...
                     shoes.Left.single_synch.frontForce.moments(:,i)];
  shoes.Left.single_synch.frontWrench(:,i) = calibMatrices.left.front * tmp_wrench_left;
  shoes.Left.single_synch.frontWrench(:,i) = -1 * shoes.Left.single_synch.frontWrench(:,i);
  tmp_wrench_left = [shoes.Left.single_synch.rearForce.forces(:,i);...
                     shoes.Left.single_synch.rearForce.moments(:,i)];
  shoes.Left.single_synch.rearWrench(:,i) = calibMatrices.left.rear * tmp_wrench_left;
  shoes.Left.single_synch.rearWrench(:,i) = -1 * shoes.Left.single_synch.rearWrench(:,i);
end
% RIGHT--------------------------------------------------------------------
shoes.Right.single_synch.frontWrench = zeros(6,size(shoes.Right.single_synch.frontForce.time,2));
shoes.Right.single_synch.rearWrench  = zeros(6,size(shoes.Right.single_synch.rearForce.time,2));
for i = 1 : size(shoes.Right.single_synch.frontForce.time,2)
  tmp_wrench_right = [shoes.Right.single_synch.frontForce.forces(:,i);...
                     shoes.Right.single_synch.frontForce.moments(:,i)];
  shoes.Right.single_synch.frontWrench(:,i) = calibMatrices.right.front * tmp_wrench_right;
  shoes.Right.single_synch.frontWrench(:,i) = -1 * shoes.Right.single_synch.frontWrench(:,i);
  tmp_wrench_right = [shoes.Right.single_synch.rearForce.forces(:,i);...
                     shoes.Right.single_synch.rearForce.moments(:,i)];
  shoes.Right.single_synch.rearWrench(:,i) = calibMatrices.right.rear * tmp_wrench_right;
  shoes.Right.single_synch.rearWrench(:,i) = -1 * shoes.Right.single_synch.rearWrench(:,i);
end

%% Transform both wrenches into a new SoR -->TotalForce
% ------------- 1step
% - Transform wrenches from Front into Rear --> only translation, no rot.
% - In the following for loops the two forces are summed.
% LEFT---------------------------------------------------------------------
left_intermediate_wrench = zeros(6,size(shoes.Left.single_synch.frontForce.time,2));
for i = 1 : size(shoes.Left.single_synch.frontForce.time,2)
    tmp_frontWrench = rear_T_front.asAdjointTransformWrench().toMatlab()* ...
                                        shoes.Left.single_synch.frontWrench(:,i);
    left_intermediate_wrench(:,i) = tmp_frontWrench + shoes.Left.single_synch.rearWrench(:,i);
end
% % RIGHT--------------------------------------------------------------------
right_intermediate_wrench = zeros(6,size(shoes.Right.single_synch.frontForce.time,2));
for i = 1 : size(shoes.Right.single_synch.frontForce.time,2)
    tmp_frontWrench = rear_T_front.asAdjointTransformWrench().toMatlab()* ...
                                        shoes.Right.single_synch.frontWrench(:,i);
    right_intermediate_wrench(:,i) = tmp_frontWrench + shoes.Right.single_synch.rearWrench(:,i);
end

% ------------- 2step
% - Transform wrenches from step1 Rear into Total --> only rotation, no trasl.
% - In the following for loops the offset of the fts sensors calibration is
%   removed.
% LEFT---------------------------------------------------------------------
shoes.Left.single_synch.totalForce.time = shoes.Left.single_synch.frontForce.time;
%shoes.Left.single_synch.totalForce.timeNormToZero = shoes.Left.single_synch.frontForce.timeNormToZero;
shoes.Left.single_synch.totalForce.forces = zeros(size(shoes.Left.single_synch.frontForce.forces));
shoes.Left.single_synch.totalForce.moments = zeros(size(shoes.Left.single_synch.frontForce.moments));
for i = 1 : size(left_intermediate_wrench,2)
     tmp_wrench = total_T_rear.asAdjointTransformWrench().toMatlab()* ...
                                        left_intermediate_wrench(:,i);
     tmp_wrench_noOff = tmp_wrench - shoesOff.Left.meanOff;
     shoes.Left.single_synch.totalForce.forces(:,i) = tmp_wrench_noOff(1:3);
     shoes.Left.single_synch.totalForce.moments(:,i) = tmp_wrench_noOff(4:6);
end
% RIGHT--------------------------------------------------------------------
shoes.Right.single_synch.totalForce.time = shoes.Right.single_synch.frontForce.time;
%shoes.Right.single_synch.totalForce.timeNormToZero = shoes.Right.single_synch.frontForce.timeNormToZero;
shoes.Right.single_synch.totalForce.forces = zeros(size(shoes.Right.single_synch.frontForce.forces));
shoes.Right.single_synch.totalForce.moments = zeros(size(shoes.Right.single_synch.frontForce.moments));
for i = 1 : size(right_intermediate_wrench,2)
     tmp_wrench = total_T_rear.asAdjointTransformWrench().toMatlab()* ...
                                        right_intermediate_wrench(:,i);
     tmp_wrench_noOff = tmp_wrench - shoesOff.Right.meanOff;
     shoes.Right.single_synch.totalForce.forces(:,i) = tmp_wrench_noOff(1:3);
     shoes.Right.single_synch.totalForce.moments(:,i) = tmp_wrench_noOff(4:6);
end

% $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
% DATA FROM STATIC PROCESSING WITH SHOES OFFSET
% $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
%% Remove offset between FP and shoes --> via static acquisition 
% Between fp and shoes there is an offset (both for Rear and Front) that we
% want to remove later by using the static acquisition (acquisition of the 
% subject, in static pose, on the two fp with the shoes).  Here we want to
% obtain the mean of the wrench of that static acquisition, both for Rear
% and Front and then to obtain the related (and proper transformed) Total
% vector of mean.
% IMPORTANT NOTE:  the vector of the mean wrench is expressed in sensors 
% (i.e., Total) frames.

% LEFT -------------------------------------------------------------
frontLeft_forcesStaticMean  = mean(shoes.Left.static_frontForce.forces,2);
frontLeft_momentsStaticMean = mean(shoes.Left.static_frontForce.moments,2);
frontLeft_staticWrenchMean  = [frontLeft_forcesStaticMean; frontLeft_momentsStaticMean];

rearLeft_forcesStaticMean  = mean(shoes.Left.static_rearForce.forces,2);
rearLeft_momentsStaticMean = mean(shoes.Left.static_rearForce.moments,2);
rearLeft_staticWrenchMean  = [rearLeft_forcesStaticMean; rearLeft_momentsStaticMean];
% RIGHT -------------------------------------------------------------
frontRight_forcesStaticMean  = mean(shoes.Right.static_frontForce.forces,2);
frontRight_momentsStaticMean = mean(shoes.Right.static_frontForce.moments,2);
frontRight_staticWrenchMean  = [frontRight_forcesStaticMean; frontRight_momentsStaticMean];

rearRight_forcesStaticMean  = mean(shoes.Right.static_rearForce.forces,2);
rearRight_momentsStaticMean = mean(shoes.Right.static_rearForce.moments,2);
rearRight_staticWrenchMean  = [rearRight_forcesStaticMean; rearRight_momentsStaticMean];

%% Compute static mean wrenches with calibration matrices
% we want to obtain: frontForceOUT = calibMatrixFront * frontForceIN and
% rearForceOUT = calibMatrixRear * rearForceIN for both Left and Right

% LEFT---------------------------------------------------------------------
  tmpLeft_front_static = calibMatrices.left.front * frontLeft_staticWrenchMean;
  leftFront_staticWrench = -1 * tmpLeft_front_static; 
  tmpLeft_rear_static = calibMatrices.left.rear * rearLeft_staticWrenchMean;
  leftRear_staticWrench = -1 * tmpLeft_rear_static;
% RIGHT--------------------------------------------------------------------
  tmpRight_front_static = calibMatrices.right.front * frontRight_staticWrenchMean;
  rightFront_staticWrench = -1 * tmpRight_front_static; 
  tmpRight_rear_static = calibMatrices.right.rear * rearRight_staticWrenchMean;
  rightRear_staticWrench = -1 * tmpRight_rear_static;

%% Transform both wrenches into a new SoR -->TotalForce
% Like in the previous computation, we need to change the SoR from 
% Rear/Front into Total

% ------------- 1step
% - Transform wrenches from Front into Rear --> only translation, no rot.
% - In the following for loops the two forces are summed.
% LEFT -------------------------------------------------------------
totalLeft_tmp = rear_T_front.asAdjointTransformWrench().toMatlab()* leftFront_staticWrench;
totalLeft_staticMean_partial = totalLeft_tmp + leftRear_staticWrench;
% RIGHT ------------------------------------------------------------
totalRight_tmp = rear_T_front.asAdjointTransformWrench().toMatlab()* rightFront_staticWrench;
totalRight_staticMean_partial = totalRight_tmp + rightRear_staticWrench;

% ------------- 2step
% - Transform wrenches from step1 Rear into Total --> only rotation, no trasl.
% - In the following for loops the offset of the fts sensors calibration is
%   removed.
% LEFT -------------------------------------------------------------
tmpLeft_wrench = total_T_rear.asAdjointTransformWrench().toMatlab()* ...
                                        totalLeft_staticMean_partial;
totalLeft_staticMean_noOff = tmpLeft_wrench - shoesOff.Left.meanOff;
% RIGHT ------------------------------------------------------------
tmpRight_wrench = total_T_rear.asAdjointTransformWrench().toMatlab()* ...
                                        totalRight_staticMean_partial;
totalRight_staticMean_noOff = tmpRight_wrench - shoesOff.Right.meanOff;

% Output static data
shoes.Left.static_totalForce.wrench_mean  = totalLeft_staticMean_noOff;
shoes.Right.static_totalForce.wrench_mean = totalRight_staticMean_noOff;

end
