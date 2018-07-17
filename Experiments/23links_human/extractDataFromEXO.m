
%% Preliminaries
close all;

% Load EXO data
EXO.dataFilename = fullfile(bucket.datasetRoot, 'EXOforceData.csv');

% Extract EXO data
EXO.extractedData = table2array(readtable(EXO.dataFilename,'Delimiter',';'));

% Load human kinematics
load(fullfile(bucket.pathToProcessedData,'synchroKin.mat'));
load(fullfile(bucket.pathToProcessedData,'selectedJoints.mat'));

% Option for plots
EXO.opts.plots = true;

% Blocks
block.labels = {'block1'; ...
    'block2'; ...
    'block3'; ...
    'block4'; ...
    'block5'};
block.nrOfBlocks = size(block.labels,1);

%% Transform angles from CURRENT to FIXED frames
% IK data are expressed in current frames. To help this analisys is useful
% to have the shoulder angles expressed in fixed frame.

for sjIdx = 1 : size(selectedJoints,1)
    if (strcmp(selectedJoints{sjIdx,1},'jRightShoulder_rotx'))
        EXO.jRshoRotx_idx = sjIdx;
    end
    if (strcmp(selectedJoints{sjIdx,1},'jRightShoulder_roty'))
        EXO.jRshoRoty_idx = sjIdx;
    end
    if (strcmp(selectedJoints{sjIdx,1},'jRightShoulder_rotz'))
        EXO.jRshoRotz_idx = sjIdx;
    end
end

for blockIdx = 1 : block.nrOfBlocks
    EXO.jRightShoulder_rotx_grad = synchroKin(blockIdx).q(EXO.jRshoRotx_idx,:) * 180/pi; %deg
    EXO.jRightShoulder_roty_grad = synchroKin(blockIdx).q(EXO.jRshoRoty_idx,:) * 180/pi; %deg
    EXO.jRightShoulder_rotz_grad = synchroKin(blockIdx).q(EXO.jRshoRotz_idx,:) * 180/pi; %deg
    
    EXO.rpy_deg = zeros (3,size(EXO.jRightShoulder_rotx_grad,2));
    for i = 1 : size(EXO.jRightShoulder_rotx_grad,2)
        EXO.q_shoulder = [synchroKin(blockIdx).q(EXO.jRshoRotx_idx,i); ...
            synchroKin(blockIdx).q(EXO.jRshoRoty_idx,i); ...
            synchroKin(blockIdx).q(EXO.jRshoRotz_idx,i)];
        [EXO.Rx, EXO.Ry, EXO.Rz] = angle2rots(EXO.q_shoulder);
        
        % The angles are expressed in current frame
        EXO.R_CF = EXO.Rx * EXO.Ry * EXO.Rz;
        
        [EXO.rpy] = mat2RPY(EXO.R_CF);
        EXO.rpy_deg(:,i) = EXO.rpy * 180/pi;
    end
    
    % Plots for the comparison
    if EXO.opts.plots
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
        box(axes1,'on');
        hold(axes1,'on');
        grid on;
        
        subplot (211)
        plot(EXO.jRightShoulder_rotx_grad,'r','Linewidth',1.5)
        hold on
        plot(EXO.jRightShoulder_roty_grad,'g','Linewidth',1.5)
        hold on
        plot(EXO.jRightShoulder_rotz_grad,'b','Linewidth',1.5)
        ylabel('joint angle [deg]','FontSize',15);
        % xlabel('samples','FontSize',15);
        title(sprintf('CURRENT FRAME, Block %s', num2str(blockIdx)))
        leg = legend('rotx','roty','rotz','southeast');
        set(leg,'FontSize',17)
        
        subplot (212)
        plot(EXO.rpy_deg(1,:),'r','Linewidth',1.5)
        hold on
        plot(EXO.rpy_deg(2,:),'g','Linewidth',1.5)
        hold on
        plot(EXO.rpy_deg(3,:),'b','Linewidth',1.5)
        ylabel('joint angle [deg]','FontSize',15);
        xlabel('samples','FontSize',15);
        title(sprintf('FIXED FRAME, Block %s', num2str(blockIdx)))
        leg = legend('rotx','roty','rotz','southeast');
        set(leg,'FontSize',17)
    end
end





function [Rx, Ry, Rz] = angle2rots(x)
%ANGLE2ROTS computes the three rotation matrices given a vector x of angle 3x1
% It is required that the vector x is ordered as follow:
%    | angle of the rotation around x |
% x =| angle of the rotation around y |
%    | angle of the rotation around z |

Rx = [ 1     0          0    ;
    0 cos(x(1)) -sin(x(1));
    0 sin(x(1))  cos(x(1))];

Ry = [ cos(x(2)) 0 sin(x(2));
    0     1     0    ;
    -sin(x(2)) 0 cos(x(2))];

Rz = [ cos(x(3)) -sin(x(3)) 0;
    sin(x(3))  cos(x(3)) 0;
    0          0     1];

end
