function [tm_est] = getChangeOfGlobalReferenceFrames(tm_t_0,...
    tm_t_1,startIndex,numberOfSamples)
% GETCHANGEOFGLOBALREFERENCEFRAMES Get the estimated change of
% reference frame matrix.
tm_t_0 = tm_t_0(:,startIndex:startIndex+numberOfSamples-1);
tm_t_1 = tm_t_1(:,startIndex:startIndex+numberOfSamples-1);
%             size(tm_t_0)
p_0 = cell(1,numberOfSamples);
p_1 = cell(1,numberOfSamples);
parfor i = 1:numberOfSamples
    p_0{i} = tm_t_0{i}.getT;
    p_1{i} = tm_t_1{i}.getT;
    %p_1(:,(i-1)*4+1:i*4) = tm_t_1(i).getT;
end
H_1_0_est = cell2mat(p_0)*pinv(cell2mat(p_1));
H_1_0_est(1:3,4) = 0;
H_1_0_est(4,1:3) = 0;
H_1_0_est(4,4) = 1;
%display('Estimated H')
%H_1_0_est
tm_est = ThreeD(matrix2quaternion(H_1_0_est));

end