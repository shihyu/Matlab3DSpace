function [ H_I_V_est ] = calculateChangeofGlobalFrames(H_0_V_t,H_1_I_t,startEst,numberOfFrames)
%CALCULATECHANGEOFGLOBALFRAMES Summary of this function goes here
%   Detailed explanation goes here
%Get the estimated H Matrix: Least Means square solution
%from within the Still part of the experiment.
points_v = [];
points_i = [];
for i = startEst:startEst+numberOfFrames
    cvt = H_0_V_t(i);
    qvt = H_1_I_t(i);
    points_v = [points_v cvt.getT()];
    points_i = [points_i qvt.getT()];
end
%Estimate the transform from 1 to 0.
H_I_V_est = points_v*pinv(points_i);
%Zero translation and error removed
H_I_V_est(1:3,4) = 0;
H_I_V_est(4,1:3) = 0;
H_I_V_est(4,4) = 1;

end

