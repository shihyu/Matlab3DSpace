function [ data_1,data_2 ] = synchronise(metric_1,metric_2,data_1,data_2,Fs,theStart,numberOfSamples)
%SYNCRHONISE Using a metric it synchronises the data inputs.

display('Syncrohonising:');

N=min(size(data_1,2),size(data_2,2));
%Synchronise over a subset of the matrix's
metric_1 = metric_1(theStart:size(metric_1,2));
metric_2 = metric_2(theStart:size(metric_2,2));
N = N-(theStart+1);
t = [0:1/Fs:N/Fs-1/Fs];
figure
subplot(2,1,1);
hold on;
plot(t,metric_1(1:N),'--k');
plot(t,metric_2(1:N),'--m');
title('Magnitude of Rotation Matrix change over time.');
grid on;

subplot(2,1,2);
hold on;
%Get correlation.
R12 = xcorr(metric_1',metric_2');
R21 = xcorr(metric_2',metric_1');
R=R12(1:N);
R12=R;
R=R21(1:N);
R21=R;

[max_12,max_lags_12]=max(R12)
[max_21,max_lags_21]=max(R21)

%Adjust input data.
if (max(max_12,max_21) == max_12)
    display('Syncrohonising: Vicon ahead of IMU');
    data_2 = data_2(N-max_lags_12:size(data_2,2));
    metric_2 = metric_2(N-max_lags_12:size(metric_2,2));
    theTitle = [' Synchronising: Vicon ahead of IMU by '...
        num2str(max_lags_12) ' '  num2str(max_12) ]
else
    data_1 = data_1(N-max_lags_21:size(data_1,2));
    metric_1 = metric_1(N-max_lags_21:size(metric_1,2));
    theTitle = ['Syncrohonising: IMU ahead of Vicon by ' ...
        num2str(max_lags_21) ' '  num2str(max_21)];
end

% Threshold = max(metric_1(theStart:theStart+numberOfSamples));
% 
% newStart = find(metric_1>Threshold*2,1);
% newStart = newStart -100;
% if newStart <0
%     newStart = 0;
% end
% 
% data_1 = data_1(newStart:size(data_1,2));
% data_2 = data_2(newStart:size(data_2,2));
% 
% N=min(size(data_1,2),size(data_2,2));
% plot(t(newStart:N),metric_1(newStart:N),'--k')
% plot(t(newStart:N),metric_2(newStart:N),'--m')
% title(theTitle)
end

