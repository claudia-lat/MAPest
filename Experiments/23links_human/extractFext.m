% % % Script to extract the estimated external wrenches by MAP.

bucket.nrOfLinks = size(dVectorOrder,1);
computedFext = struct;
range = zeros(bucket.nrOfLinks,1);

for blockIdx = 1 : block.nrOfBlocks
    computedFext(blockIdx).label  = dVectorOrder;
    
    nrOfSamples  = size(data(blockIdx).y, 2);
    computedFext(blockIdx).values = zeros(6*size(dVectorOrder,1), nrOfSamples);
    for i = 1 : bucket.nrOfLinks
        range(i,1) = (rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, dVectorOrder{i}));
        tmpRange = range(i,1) : range(i,1)+5;
        computedFext(blockIdx).values(6*(i-1)+1:6*i,:) = ...
            estimation(blockIdx).mu_dgiveny(tmpRange,:);
    end
end

% % % Script to extract via berdy the estimated external wrenches by MAP.
% % mu_dgiveny_berdy = iDynTree.VectorDynSize();
% % fext_berdy = iDynTree.LinkWrenches(); %the output
% % bucket.nrOfLinks = size(dVectorOrder,1);
% %
% % % Resize vector
% % mu_dgiveny_berdy.resize(berdy.getNrOfDynamicVariables());
% % fext_berdy.resize(bucket.nrOfLinks);
% %
% % computedFextFromBerdy = struct;
% % for blockIdx = 1 : block.nrOfBlocks
% %     nrOfSamples  = size(data(blockIdx).y, 2);
% %     fext_vector = zeros(6*bucket.nrOfLinks, nrOfSamples);
% %     fextFromBerdy = zeros(6*bucket.nrOfLinks, nrOfSamples);
% %     for i = 1 : nrOfSamples
% %         mu_dgiveny_berdy.fromMatlab(estimation(blockIdx).mu_dgiveny(:,i));
% %
% %
% % %         fext_berdy.fromMatlab(fext_vector(:,i)); % no method
% % %           .fromMatlab for iDynTree.LinkWrenches ... TODO
% %
% %
% %         berdy.extractLinkNetExternalWrenchesFromDynamicVariables(mu_dgiveny_berdy, fext_berdy);
% %         fextFromBerdy(:,i) = fext_berdy.toMatlab();
% %     end
% %     computedFextFromBerdy(blockIdx).values = fextFromBerdy;
% %     computedFextFromBerdy(blockIdx).label  = selectedJoints;
% % end
