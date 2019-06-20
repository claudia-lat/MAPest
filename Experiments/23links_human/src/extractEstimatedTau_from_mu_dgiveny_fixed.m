function [tau, ddq] = extractEstimatedTau_from_mu_dgiveny_fixed(berdy, dJointOrder, mu_dgiveny)
%EXTRACTEDESTIMATEDtau_FROM_MUDGIVENY_fixed extracts the estimated torque
% and joint acceleration into the vector estimated by MAP.

nrOfJoints = size(dJointOrder,1);
range_tau = zeros(nrOfJoints,1);
range_ddq = zeros(nrOfJoints,1);
nrOfSamples  = size(mu_dgiveny  ,2);

tau = zeros(size(dJointOrder,1), nrOfSamples);
ddq = zeros(size(dJointOrder,1), nrOfSamples);
for i = 1 : nrOfJoints
    range_tau = (rangeOfDynamicVariable(berdy, iDynTree.DOF_TORQUE, dJointOrder{i}));
    tau(i,:) = mu_dgiveny(range_tau,:);
    
    range_ddq = (rangeOfDynamicVariable(berdy, iDynTree.DOF_ACCELERATION, dJointOrder{i}));
    ddq(i,:) = mu_dgiveny(range_ddq,:);
end
end
