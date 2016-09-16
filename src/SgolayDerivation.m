function [da,dda] = SgolayDerivation(polynOrder,window,a,samplingTime)
% SGOLAYDERIVATION computes first order and second order derivation of a
% signal using SgolayFilt choosing properly the polynomial order and the
% size of the moving window. 
% It computes a diffCoeff matrix of (polynomialOrder-1) columns where:
% - ( ,1)                 --> coefficient for S-Golay as smoother;
% - ( ,2)                 --> coefficient for S-Golay as 1st differentiator;
% - ( ,3)                 --> coefficient for S-Golay as 2nd differentiator;
%     .   
%     .
%     .
% - ( ,polynomialOrder-1) --> coefficient for S-Golay as 
%                                (polynomialOrder) differentiator;


[~, diffCoeff] = SgolayWrapper(polynOrder, window);
 halfWindow  = ((window+1)/2) - 1;
 l = length(a);
 da = zeros(l, 1);
 dda = zeros(l, 1);
 
     for n = (window+1)/2 : l-(window+1)/2,
         % 1st differential
         da(n) = dot(diffCoeff(:,2), a(n - halfWindow:n + halfWindow));
         % 2nd differential
         dda(n) = dot(diffCoeff(:,3), a(n - halfWindow:n + halfWindow));
     end

da = da ./ samplingTime;
dda = dda ./ (samplingTime)^2;

end
