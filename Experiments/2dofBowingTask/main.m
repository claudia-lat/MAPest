
clear;clc;close all;
rng(1); % forcing the casual generator to be const


%% Add src to the path
addpath(genpath('../../src'));

%% Load URDF model

model.FileName = 'models/threeLinkHuman_subject1_sensorExt.urdf';
modelLoader = iDynTree.ModelLoader();
if ~modelLoader.loadModelFromFile(model.FileName);
    fprint('Something wrong with the model loading.')
end
model   = modelLoader.model(); 
sensors = modelLoader.sensors();

%% Specify the sensors structure

berdyOptions = iDynTree.BerdyOptions();
berdyOptions.includeAllNetExternalWrenchesAsSensors          = true;
berdyOptions.includeAllNetExternalWrenchesAsDynamicVariables = true;
berdyOptions.includeAllJointAccelerationsAsSensors           = true;
berdyOptions.includeAllJointTorquesAsSensors                 = false;
berdyOptions.includeFixedBaseExternalWrench                  = true;

% sensors.removeAllSensorsOfType(iDynTree.GYROSCOPE);
sensors.removeSensor(iDynTree.GYROSCOPE, 'imu_gyro');

berdy = iDynTree.BerdyHelper();
berdy.init(model, sensors, berdyOptions);
                                             
%% Load data from sensors

data = load('data/ExperimentDataset.mat');
data.numOfSamples = size(data.sensors.acc,2);

% Each sensor is identified by:
% - type: the type associated to the sensors in iDynTree;
% - id  : the name associated in the URDF;
% - meas: value of measurements;
% - var : variance of sensor. 


%accelerometer
SENS.acc_struct.type    = iDynTree.ACCELEROMETER_SENSOR;
SENS.acc_struct.id      = 'imu_acc';
SENS.acc_struct.meas    = data.sensors.acc;
SENS.acc_struct.var     = 0.001111 * ones(3,1); %from datasheet

%gyroscope
SENS.gyro_struct.type   = iDynTree.GYROSCOPE_SENSOR;
SENS.gyro_struct.id     = 'imu_gyro';
SENS.gyro_struct.meas   = zeros(size(data.sensors.acc)); %no gyro for this experiment
SENS.gyro_struct.var    = 5e-5 * ones(3,1); %from datasheet

%joint acceleration as sensors
SENS.ddq1.type          = iDynTree.DOF_ACCELERATION_SENSOR;
SENS.ddq1.id            = 'ankle';
SENS.ddq1.meas          = data.joints.ddq(1,:);
SENS.ddq1.var           = 6.66e-6; %from datasheet

SENS.ddq2.type          = iDynTree.DOF_ACCELERATION_SENSOR;
SENS.ddq2.id            = 'hip';
SENS.ddq2.meas          = data.joints.ddq(2,:);
SENS.ddq2.var           = 6.66e-6; %from datasheet

%external forse as sensors
SENS.fext1.type         = iDynTree.NET_EXT_WRENCH_SENSOR;
SENS.fext1.id           = 'foot';
SENS.fext1.meas         = data.sensors.fts_footFrame;
SENS.fext1.var          = [59; 59; 36; 2.25; 2.25; 0.56]; %from datasheet

SENS.fext2.type         = iDynTree.NET_EXT_WRENCH_SENSOR;
SENS.fext2.id           = 'leg';
SENS.fext2.meas         = zeros(6, data.numOfSamples);
SENS.fext2.var          = [59; 59; 36; 2.25; 2.25; 0.56]; %from datasheet

SENS.fext3.type         = iDynTree.NET_EXT_WRENCH_SENSOR;
SENS.fext3.id           = 'torso';
SENS.fext3.meas         = zeros(6, data.numOfSamples);
SENS.fext3.var          = [59; 59; 36; 2.25; 2.25; 0.56]; %from datasheet

% (for the experiment with Xsens, this will be a file .mat with
% data extracted from the file .mvnx)
data.packedSens = [SENS.ddq1; SENS.fext2; SENS.acc_struct; SENS.ddq2; ...
                   SENS.gyro_struct; SENS.fext3; SENS.fext1];
    
[y, Sigmay] = berdyMeasurementsWrapping(berdy, data.packedSens);

%% MAP

% Set priors
priors        = struct;
priors.mud    = zeros(berdy.getNrOfDynamicVariables(), 1);
priors.Sigmad = 1e+4 * eye(berdy.getNrOfDynamicVariables());
priors.SigmaD = 1e-4 * eye(berdy.getNrOfDynamicEquations());
priors.Sigmay = Sigmay; 

% Set state
state    = struct;
state.q  = data.joints.q;
state.dq = data.joints.dq;

% Compute MAP
[mu_dgiveny, Sigma_dgiveny] = MAPcomputation(berdy,state,y,priors);

%% Plots results

time = 0:0.01:0.01*(data.numOfSamples-1);

fig = figure();
axes1 = axes('Parent',fig,...
             'FontSize',16,...
             'box','on');

%joint angles         
subplot(311);
plot1 = plot(time,state.q(1,:).*(180/pi),'lineWidth',1.5); hold on;
set(plot1,'color',[1 0 0]);
plot2= plot(time,state.q(2,:).*(180/pi),'lineWidth',1.5);
set(plot2,'color',[0 0.498039215803146 0]);

leg = legend('$q_1$','$q_2$');
set(leg,'Interpreter','latex',...
        'FontSize',13,...
        'Location','northeast');

xlabel('Time [s]',...
        'FontWeight','bold',...
        'FontSize',11,...
        'Interpreter','latex');
ylabel('$q$ [deg]',...
        'FontWeight','bold',...
        'FontSize',13,...
        'Interpreter','latex');
axis tight;
grid on;  

title('Joint quantities',...
      'FontSize',15,...
      'HorizontalAlignment','center',...
      'Interpreter','latex');

%joint velocities
subplot(312);
plot1 = plot(time,state.dq(1,:).*(180/pi),'lineWidth',1.5); hold on;
set(plot1,'color',[1 0 0]);
plot2= plot(time,state.dq(2,:).*(180/pi),'lineWidth',1.5); 
set(plot2,'color',[0 0.498039215803146 0]);

leg = legend('$\dot q_{1}$','$\dot q_{2}$');
set(leg,'Interpreter','latex',...
        'FontSize',13,...
        'Location','northeast');
    
xlabel('Time [s]',...
        'FontWeight','bold',...
        'FontSize',11,...
        'Interpreter','latex');
ylabel('$\dot q$ [deg/s]',...
        'FontWeight','bold',...
        'FontSize',13,...
        'Interpreter','latex');
axis tight;
grid on;

%tau
subplot(313);
plot1 = plot(time,mu_dgiveny(19,:)); hold on;
set(plot1,'color',[1 0 0],...
          'lineWidth',1.5,...
          'LineStyle','-');
plot2 = plot(time,mu_dgiveny(45,:)); 
set(plot2,'color',[0 0.498039215803146 0],...
          'lineWidth',1.5,...
          'LineStyle','-');     

leg = legend('$\tau_{1,MAP}$','$\tau_{2,MAP}$');
set(leg,'Interpreter','latex',...
        'FontSize',13,...
        'Location','northeast');
    
xlabel('Time [s]',...
        'FontWeight','bold',...
        'FontSize',11,...
        'Interpreter','latex');
ylabel('$\tau$ [Nm]',...
        'FontWeight','bold',...
        'FontSize',13,...
        'Interpreter','latex');
axis tight;
grid on;