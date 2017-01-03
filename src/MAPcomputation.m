        function [mu_dgiveny, Sigma_dgiveny] = MAPcomputation(berdy, state, y, priors, varargin)
% MAPCOMPUTATION solves the inverse dynamics problem with a 
% maximum-a-posteriori estimation by using the Newton-Euler algorithm and 
% redundant sensor measurements as originally described in the paper 
% [Whole-Body Human Inverse Dynamics with Distributed Micro-Accelerometers,
% Gyros and Force Sensing,Latella, C.; Kuppuswamy, N.; Romano, F.; 
% Traversaro, S.; Nori, F.,Sensors 2016, 16, 727].
%
% Considering a generic multibody model composed by n moving rigid bodies 
% (i.e. links) connected by joints and being in the Gaussian framework, 
% the output 'mu_dgiveny' coincides with the vector 'd' structured as 
% follows:
%                 d  = [d_1, d_2, ..., d_n], i = 1,...,n
%
% where:
%                d_i = [a_i, fB_i, f_i, tau_i, fx_i, ddq_i]
%
% and a_i is the link-i spatial acceleration, fB_i is the net spatial
% force on the link-i, f_i is spatial wrench transmitted to link-i from
% its parent, tau_i is torque on joint-i, fx_i is the external force on
% link-i and ddq_i is acceleration of joint-i.  
% The relationship between d and the sensor measurements y is given by
%
%                      Y(q, dq) d + b_Y = y                   (1)
%
% where the matrix Y(q, dq), is represented as a sparse matrix. Moreover, 
% the variables in d should satisfy the Newton-Euler equations 
%
%                    D(q,dq) d + b_D(q, dq) = 0               (2)
%
% again represented as a sparse matrix. 
% MEASUREMENTS EQUATIONS (1) and CONSTRAINTS EQUATIONS (2) stacked
% together represent the system that MAP solves.
%
% -------------------------------------------------------------------------
% Important note: 
% the function provides an option for removing from the
% analysis a sensor.  By default, if it is not specified anything, MAP is
% computed by using all the sensors (i.e. the full vector of y
% measurements). If it is specified in the option the sensor to remove, MAP
% loads the full y and then remove automatically values related to that
% sensor and the related block variance from the Sigmay.
% -------------------------------------------------------------------------

options = struct(   ...
    'SENSORS_TO_REMOVE', []...
    );

% read the acceptable names
optionNames = fieldnames(options);

% count arguments
nArgs = length(varargin);
if round(nArgs/2)~=nArgs/2
    error('createXsensLikeURDFmodel needs propertyName/propertyValue pairs')
end

for pair = reshape(varargin,2,[]) % pair is {propName;propValue}
    inpName = upper(pair{1}); % make case insensitive

    if any(strcmp(inpName,optionNames))
        % overwrite options. If you want you can test for the right class here
        % Also, if you find out that there is an option you keep getting wrong,
        % you can use "if strcmp(inpName,'problemOption'),testMore,end"-statements
        options.(inpName) = pair{2};
    else
        error('%s is not a recognized parameter name',inpName)
    end
end

%%
rangeOfRemovedSensors = [];
for i = 1 : size(options.SENSORS_TO_REMOVE)
    ithSensor = options.SENSORS_TO_REMOVE(i);
    [index, len] = rangeOfSensorMeasurement( berdy, ithSensor.type, ithSensor.id);
    rangeOfRemovedSensors = [rangeOfRemovedSensors, index : index + len - 1];
end

y(rangeOfRemovedSensors,:) = [];
priors.Sigmay(rangeOfRemovedSensors, :) = [];  % TO BE CHECKED!
priors.Sigmay(:, rangeOfRemovedSensors) = [];  % TO BE CHECKED!
%% 
% Set gravity 
gravity = [0 0 -9.81];
grav  = iDynTree.Vector3();
grav.fromMatlab(gravity);

% Set matrices
berdyMatrices       = struct;
berdyMatrices.D     = iDynTree.MatrixDynSize();
berdyMatrices.b_D   = iDynTree.VectorDynSize();
berdyMatrices.Y     = iDynTree.MatrixDynSize();
berdyMatrices.b_Y   = iDynTree.VectorDynSize();

berdy.resizeAndZeroBerdyMatrices(berdyMatrices.D,...
                                 berdyMatrices.b_D,...
                                 berdyMatrices.Y,...
                                 berdyMatrices.b_Y);
% Set priors
mud        = priors.mud;
Sigmad_inv = sparse(inv(priors.Sigmad));
SigmaD_inv = sparse(inv(priors.SigmaD));
Sigmay_inv = sparse(inv(priors.Sigmay));

% Allocate outputs 
samples = 1:size(y, 2);
% samples = 1:50;
numOfsamples = length(samples);
nrOfDynVariables = berdy.getNrOfDynamicVariables();
mu_dgiveny    = zeros(nrOfDynVariables, numOfsamples);
% Sigma_dgiveny = sparse(nrOfDynVariables, nrOfDynVariables, samples);
Sigma_dgiveny =  cell(numOfsamples,1);

% MAP Computation
q  = iDynTree.JointPosDoubleArray(berdy.model());
dq = iDynTree.JointDOFsDoubleArray(berdy.model());

% modelError = zeros(berdy.getNrOfDynamicEquations, samples);
% measError  = zeros(size(y,1), samples);

for i = samples
    
    q.fromMatlab(state.q(:,i));
    dq.fromMatlab(state.dq(:,i));
    
    berdy.updateKinematicsFromTraversalFixedBase(q,dq,grav);

    berdy.getBerdyMatrices(berdyMatrices.D,...
                           berdyMatrices.b_D,...
                           berdyMatrices.Y,...
                           berdyMatrices.b_Y);        
                                        
    D   = sparse(berdyMatrices.D.toMatlab());
    b_D = berdyMatrices.b_D.toMatlab();
    Y_nonsparse = berdyMatrices.Y.toMatlab();
    Y_nonsparse(rangeOfRemovedSensors, :) = [];
    Y   = sparse(Y_nonsparse);
    b_Y = berdyMatrices.b_Y.toMatlab();
    
    b_Y(rangeOfRemovedSensors) = [];

    SigmaBarD_inv = D' * SigmaD_inv * D + Sigmad_inv;
    muBarD        = SigmaBarD_inv \ (Sigmad_inv * mud - D' * SigmaD_inv * b_D);

%     Sigma_dgiveny{i} = inv(SigmaBarD_inv + Y' * Sigmay_inv * Y);
    mu_dgiveny(:,i)      = (SigmaBarD_inv + Y' * Sigmay_inv * Y) \ (Y' * Sigmay_inv * (y(:,i) - b_Y) ...
                          + SigmaBarD_inv * muBarD);
    
% %     Sigma_dgiveny{i}     = inv(SigmaBarD_inv + Y' * Sigmay_inv * Y);
% %     mu_dgiveny(:,i)      = Sigma_dgiveny{i} * (Y' * Sigmay_inv * (y(:,i) - b_Y) ...
% %                           + SigmaBarD_inv * muBarD);
%
%
%     %test for checking errors
%     modelError(:,i) = (D * mu_dgiveny(:,i)) + b_D;
%     measError(:,i)  = (Y * mu_dgiveny(:,i)) + b_Y - y(:,i);
end
end
