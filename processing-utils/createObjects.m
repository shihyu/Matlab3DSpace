function [ metric_v,metric_i,ctm_t,qtm_t ] = createObjects(rbt,lbt,fot,quaternions)
%CALCULATESYNCHORNISATIONMETRIC Calculates the metrics to use
%for synchronisation between ThreeMarker objects.
t_v = rbt(:,4);
t_i = quaternions(:,5);
vikonSize = size(rbt,1)
quaternionSize = size(quaternions,1)
N=max(vikonSize,quaternionSize)
N_first = min(vikonSize,quaternionSize)

ctm_t = [];
qtm_t = [];

metric_v = zeros(1,N_first);
metric_i = zeros(1,N_first);

hdiff_v = zeros(4,4);
hdiff_i = zeros(4,4);

for i = 1:N_first
    %Get the vicon points.
    cvt = ViconThreeMarkers(rbt(i,1:3),...
        lbt(i,1:3),fot(i,1:3),t_v(i));
    qvt = QuaternionsThreeMarkers([ quaternions(i,1:4) t_i(i)]);
    %Save the cvt
    ctm_t = [ctm_t cvt];
    %Save the quaternions.
    qtm_t = [qtm_t qvt];
    
    %Calculate the difference between H matrix's from last point
    metric_v(i) = cvt.calculateRotDiff(cvt.getH(),hdiff_v);
    metric_i(i) = cvt.calculateRotDiff(qvt.getH(),hdiff_i);
    %Save current H matrix for next iteration.
    hdiff_v = cvt.getH();
    hdiff_i = qvt.getH();
end
%Fill the rest.
% if N_first == vikonSize
%     display('IMU longer thatn Vikon');
%     for i = N_first+1:N
%         qvt = QuaternionsThreeMarkers(quaternions(i,1:4),t_i(i));
%         qtm_t = [qtm_t qvt];
%         metric_i(i) = cvt.calculateRotDiff(qvt.getH(),hdiffOld(1:4,5:8));
%         hdiffOld = [zeros(4,4) qvt.getH()];
%     end
% else
%     display('Vikon longer than IMU');
%     for i = N_first+1:N
%         cvt = ViconThreeMarkers(rbt(i,1:3),...
%                 lbt(i,1:3),fot(i,1:3),t_v(i));
%         ctm_t = [ctm_t cvt];
%         metric_v(i) = cvt.calculateRotDiff(cvt.getH(),hdiffOld(1:4,1:4));
%         hdiffOld = [cvt.getH() zeros(4,4)];
%     end
% end
end

