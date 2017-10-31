% for i = 1 : size(calib, 1)
%     quat = iDynTree.Vector4;
%     quat.fromMatlab(calib(i,:));
%     rotCalib = iDynTree.Rotation;
%     rotCalib.fromQuaternion(quat);
%     
%     quat.fromMatlab(exp(i,:));
%     rotExp = iDynTree.Rotation;
%     rotExp.fromQuaternion(quat);
%     
%     (rotExp.asRPY.toMatlab - rotCalib.asRPY.toMatlab)* 180/pi
%     
%     diff = rotExp.inverse * rotCalib;
% %     diff.asRPY.toMatlab * 180/pi
% end

% COMPUTE W_R_A
for i =1:23
quatInTpose = (Exp9.subject.frames.frame(2).orientation((i-1)*4 + 1 : 4 * i)) ;
quat = iDynTree.Vector4;
quat.fromMatlab(quatInTpose);
rot = iDynTree.Rotation;
rot.fromQuaternion(quat);
W_R_A(:,:,i) = rot.toMatlab;
W_q_A(:,i) = (Exp9.subject.frames.frame(2).orientation((i-1)*4 + 1 : 4 * i));
end
    
% COMPUTE T_R_A using Npose field
for i =1:23
quat = iDynTree.Vector4;
quat.fromMatlab(suit.calibration.npose.orientation(:,i));
rot = iDynTree.Rotation;
rot.fromQuaternion(quat);
T_R_A(:,:,i) = rot.toMatlab;

temp = suit.calibration.npose.orientation(:,i);
temp = temp .* [1; -1; -1; -1];
temp = temp/norm(temp);
A_q_T(:,i) = temp;
end
 
% COMPUTE A_R_T
for i =1:23
A_R_T(:,:,i) = T_R_A(:,:,i)';
end

% COMPUTE W_R_T
for i =1:23  
W_R_T(:,:,i) = W_R_A(:,:,i) * A_R_T(:,:,i);   

%quat product of q1 * q2
%real:
%q1(1) * q2(1) - q1(2:4)' * q2(2:4);
%imaginary:
%q1(1) * q2(2:4) + q2(1) * q1(2:4) + S(q1(2:4) * q2(2:4)

temp = zeros(4,1);
%W_q_A * A_q_T
temp(1) = W_q_A(1,i) * A_q_T(1,i) - W_q_A(2:4,i)' * A_q_T(2:4,i);
temp(2:4) =  W_q_A(1,i) * A_q_T(2:4,i) + ...
    W_q_A(2:4,i) * A_q_T(1,i) + ...
    cross(W_q_A(2:4,i), A_q_T(2:4,i));

W_q_T(:,i) = temp;

%to get RPY
tempQuat = iDynTree.Vector4;
tempQuat.fromMatlab(W_q_T(:,i));
tempRot = iDynTree.Rotation;
tempRot.fromQuaternion(tempQuat);
W_Rdyn_T(i,:) = tempRot.asRPY.toMatlab() / pi * 180;

end

%%
linkLabel = cell(23,1);
sensorLabel = cell(17,1);
for i =1:23 
    linkLabel{i} = suit.links{i, 1}.label;
end
for i =1:17
sensorLabel{i} = suit.sensors{i, 1}.label;
end

for i =1:23
    figure;
    plot(suit.links{i, 1}.meas.angularVelocity');
    leg = legend('$w_x$','$w_y$','$w_z$','Location','northeast');
    set(leg,'Interpreter','latex');
    set(leg,'FontSize',18);                
    title(sprintf('link %s',linkLabel{i}),...
    'HorizontalAlignment','center',...
     'Interpreter','latex', 'FontSize',18);
     xlabel('frames','FontSize',20);
     ylabel('angular velocity','FontSize',20);
end

sensPosition = zeros(17,3);
for i=1:17
    sensPosition(i,:)=  suit.sensors{i, 1}.position';
end

%%
clc;

urdf = '/Users/makaveli/Documents/MATLAB/MAPest/Experiments/23links_human/models/XSensURDF_subj1.urdf';

%load model
modelLoader = iDynTree.ModelLoader;
modelLoader.loadModelFromFile(urdf);

%use default options
berdyOptions = iDynTree.BerdyOptions;

%load berdy
berdy = iDynTree.BerdyHelper;
berdy.init(modelLoader.model, modelLoader.sensors, berdyOptions);

%get the current traversal
traversal = berdy.dynamicTraversal;

currentBase = berdy.model().getLinkName(traversal.getBaseLink().getIndex());
disp(strcat('Current base is: ', currentBase));

linkNames = cell(traversal.getNrOfVisitedLinks(), 1);

%for each link in the traversal get the name
for i = 0 : traversal.getNrOfVisitedLinks() - 1
    link = traversal.getLink(i);
    linkNames{i + 1} = berdy.model().getLinkName(link.getIndex());
end

linkNames

%% second test: use another link as base frame
berdyOptions = iDynTree.BerdyOptions;
berdyOptions.baseLink = 'LeftFoot';

berdy = iDynTree.BerdyHelper;
berdy.init(modelLoader.model, modelLoader.sensors, berdyOptions);

traversal = berdy.dynamicTraversal;
currentBase = berdy.model().getLinkName(traversal.getBaseLink().getIndex());
disp(strcat('Current base is: ', currentBase));
linkNames = cell(traversal.getNrOfVisitedLinks(), 1);

for i = 0 : traversal.getNrOfVisitedLinks() - 1
    link = traversal.getLink(i);
    linkNames{i + 1} = berdy.model().getLinkName(link.getIndex());
end


linkNames


berdyOptions = iDynTree.BerdyOptions;
berdyOptions.baseLink = 'fake';

berdy = iDynTree.BerdyHelper;
berdy.init(modelLoader.model, modelLoader.sensors, berdyOptions)

%%
orderedJointName = cell(model.getNrOfJoints(),1);
orderedJointIndex = zeros(model.getNrOfJoints(),1);
for i = 0 : 65
    orderedJointName{i+1} = model.getJointName(i);
    orderedJointIndex(i+1,1)= model.getJointIndex(model.getJointName(i));
end

%% plot test

figure;

subplot(211)
plot (state.q(:,15))

subplot(212)
plot (state.dq(:,15))
% 
% subplot(313)
% plot (ddq(:,12))

jointNameTest = cell(66,1);
for i = 8 : size(motionData.colheaders,2)
 jointNameTest{i} = motionData.colheaders{i};
end 

%% test 

sensOrd = berdy.getSensorsOrdering;
orderStruct = cell(2,size(sensOrd,2));
for i = 1: size(sensOrd,2)
    orderStruct{1,i} = sensOrd{i}.id;
    orderStruct{2,i} = sensOrd{i}.type;
end

%%

jointNamesFromModel = cell(66,1);
for i = 1 : 66 
    jointNamesFromModel{i} = humanModel.getJointName(i-1);
end

%%
for indx = 1 : nOfSensor.fext
    if  strcmp(data.fext.id{indx,1}.label,linkName)
        linkStruct = TotalLinksStruct{indx,1};
        found = true;
        break;
    else 
        found = false;
    end   
end
if found == false
    error(sprintf('Link label <%s> not found.',linkName));
end
%%
% function [q, found] = angleFromName(struct, angleName)
%     %ANGLEFROMNAME
%     % Inputs:
%     % struct     : struct motionData from OSIM computation;
%     % angleName  : string denoting the angle q you are looking for;
%     % Outputs:
%     % q      : vector of joint angles;
%     % found  : true if q has been found, false otherwise.
% 
%     for indx = 1 : size(struct.data,2)
%         if  strcmp(struct.colheaders{1,indx},angleName)
%             q = struct.data(:,indx);
%             found = true;
%             break;
%         else
%             found = false;
%         end
%     end
%     if found == false
%         error(sprintf('Wrong joint angles matching! Angleq label <%s> not found.',angleName));
%     end
% end

%% Plot torques for all 66 links
torque = zeros(66,size(mu_dgiveny,2));
for i = 1 : 66
    torque(i,:) = mu_dgiveny(26*i-7, :);
end

for j = 1
    figure;
    plot (torque(j,:));
    title (sprintf(' %s', dJointOrder{j}));
end

for i = 1 : 66
    fext(i,:) = mu_dgiveny(26*i-6, :);
end

%%  

[index, len] = rangeOfSensorMeasurement( berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'LeftHand');


%% BALANCE

for sample = 1:size(mu_dgiveny,2)

gravityZero = iDynTree.Vector3();
gravityZero.zero();
humanZeroJntVel = iDynTree.JointDOFsDoubleArray(human_kinDynComp.model());
humanZeroJntVel.zero();
humanJntPos = iDynTree.JointPosDoubleArray(human_kinDynComp.model());
humanJntPos.fromMatlab(human_state.q(:,sample));
human_kinDynComp.setRobotState(humanJntPos,humanZeroJntVel,gravityZero);

LeftFoot_H_RightHand = human_kinDynComp.getRelativeTransform('LeftFoot','RightHand');
LeftFoot_R_RightHand = LeftFoot_H_RightHand.getRotation().toMatlab();

LeftFoot_H_LeftHand = human_kinDynComp.getRelativeTransform('LeftFoot','LeftHand');
LeftFoot_R_LeftHand = LeftFoot_H_LeftHand.getRotation().toMatlab();

LeftFoot_H_ = human_kinDynComp.getRelativeTransform('LeftFoot','RightFoot');
LeftFoot_R_RightFoot = LeftFoot_H_.getRotation().toMatlab();


balance_y(:,sample) = forceplate.processedData.humanLeftFootWrench(1:3,sample) + ...
    LeftFoot_R_RightFoot * forceplate.processedData.humanRightFootWrench(1:3,sample)+ ...
    LeftFoot_R_LeftHand * robot.processedData.humanLeftHandWrench(1:3,sample) + ...
    LeftFoot_R_RightHand * robot.processedData.humanRightHandWrench(1:3,sample);

balance_d(:,sample) = forceplate.processedData.humanLeftFootWrench(1:3,sample) + ...
    LeftFoot_R_RightFoot * mu_dgiveny(566:568,sample)+ ...
    LeftFoot_R_LeftHand * mu_dgiveny(1294:1296,sample) + ...
    LeftFoot_R_RightHand * mu_dgiveny(1580:1582,sample);

end

%% 

gravityZero = iDynTree.Vector3();
gravityZero.zero();
humanZeroJntVel = iDynTree.JointDOFsDoubleArray(human_kinDynComp.model());
humanZeroJntVel.zero();
humanJntPos = iDynTree.JointPosDoubleArray(human_kinDynComp.model());
humanJntPos.fromMatlab(human_state.q(:,450));
human_kinDynComp.setRobotState(humanJntPos,humanZeroJntVel,gravityZero);

for i= 1: size(suit.sensors,1)
   
    LeftFoot_H_L = human_kinDynComp.getRelativeTransform('LeftFoot',suit.sensors{i, 1}.attachedLink);
    LeftFoot_R_L = LeftFoot_H_L.getRotation().toMatlab();
    
    L_R_S_iDyn = iDynTree.Rotation.RPY(suit.sensors{i, 1}.RPY(1), suit.sensors{i, 1}.RPY(2), suit.sensors{i, 1}.RPY(3));
    L_R_S = L_R_S_iDyn.toMatlab();
    
  LeftFoot_R_S = LeftFoot_R_L * L_R_S;
  LeftFoot_acc_S(:,i) = LeftFoot_R_S * suit.sensors{i, 1}.meas.sensorAcceleration(:,450);
 
end





























