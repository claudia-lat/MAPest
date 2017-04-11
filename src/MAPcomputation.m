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
% For d it computes only the mean and not the variance.
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
samples = size(y, 2); 
nrOfDynVariables = berdy.getNrOfDynamicVariables();
mu_dgiveny    = zeros(nrOfDynVariables, samples);
% Sigma_dgiveny = sparse(nrOfDynVariables, nrOfDynVariables, samples);
Sigma_dgiveny =  cell(samples,1);

% MAP Computation
q  = iDynTree.JointPosDoubleArray(berdy.model());
dq = iDynTree.JointDOFsDoubleArray(berdy.model());

for i = 1 : samples
    
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

    SigmaBarD_inv   = D' * SigmaD_inv * D + Sigmad_inv;
    
    % the permutation matrix for SigmaBarD_inv is computed only for the first
    % sample, beacuse this matrix does not change in the experiment     
    if (i==1)  
        [~,~,PBarD]= chol(SigmaBarD_inv);
    end
    
    rhsBarD         = Sigmad_inv * mud - D' * (SigmaD_inv * b_D);
    muBarD          = CholSolve(SigmaBarD_inv , rhsBarD, PBarD);
    
    Sigma_dgiveny_inv = SigmaBarD_inv + Y' * Sigmay_inv * Y;
    
    % the permutation matrix for Sigma_dgiveny_inv is computed only for the first
    % sample, beacuse this matrix does not change in the experiment     
    if (i==1)
        [~,~,P]= chol(Sigma_dgiveny_inv);
    end
    
    rhs             = Y' * (Sigmay_inv * (y(:,i) - b_Y)) + SigmaBarD_inv * muBarD;
    
    if nargout > 1   % Sigma_dgiveny requested as output
        Sigma_dgiveny{i}   = inv(Sigma_dgiveny_inv);
        mu_dgiveny(:,i)    = Sigma_dgiveny{i} * rhs;
    else             % Sigma_dgiveny does not requested as output
        mu_dgiveny(:,i)    = CholSolve(Sigma_dgiveny_inv, rhs, P);
    end
end
end

function [x] = CholSolve(A, b, P)
% control if A is symmetric
if (issymmetric(round(A,5)) == 1)
    C              = P'*A*P;  % P is given as input 
    [R]            = chol(C); % R is such that R'*R = P'*C*P
    
    w_forward      = P\b;
    z_forward      = R'\w_forward;
    y_forward      = R\z_forward;
    x              = P'\y_forward;
else
    error('matrix A is not symmetric')        
end    
end