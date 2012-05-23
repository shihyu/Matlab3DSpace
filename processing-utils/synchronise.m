function [ data_1,data_2,corrValue] = synchronise(metric_1,...
    metric_2,data_1,data_2,...
    Fs,theStart,numberOfSamples)
%SYNCRHONISE Performs simple cross-correlation syncrhonisation between two
%signals using the metric_1 and metric_2 as the correlation signal.

%display('Syncrohonising:');
N1 = size(data_1,2);
N2 = size(data_2,2);
N=max(N1,N2);

%Synchronise over a subset of the matrix's
metric_1 = metric_1(theStart:N1);
metric_2 = metric_2(theStart:N2);
metric_1 = metric_1./norm(metric_1);
metric_2 = metric_2./norm(metric_2);

N = N-(theStart-1);
N1 = N1-(theStart-1);
N2 = N2-(theStart-1);
t = [0:1/Fs:N/Fs-1/Fs];

subplot(2,1,1);
hold on;
plot(t(1:N1),metric_1(1:N1),'--k');
plot(t(1:N2),metric_2(1:N2),'--m');
title('Magnitude of Rotation Matrix change over time.');
grid on;

subplot(2,1,2);
hold on;
%Get correlation.
[R12] = xcorr(metric_1',metric_2');
[corrValue,max_lags_12]=max(R12);
% display(['SIZE correlation: R12:' num2str(size(R12,1))...
%     ' MAX12:' num2str(max_12)  ...
%     ' MAXLAGS12:' num2str(max_lags_12) ' N:' num2str(N)])
%Adjust input data.
if (max_lags_12 < N)
    Nstart = N-max_lags_12+1;
    theTitle = ['Synchronising: Black ahead of Magenta: Shifting:' ...
        num2str(Nstart)];
    %display(theTitle);
    data_2 = data_2(Nstart:size(data_2,2));
    metric_2 = metric_2(Nstart:size(metric_2,2));
    
    N2 = size(metric_2,2);
else
    Nstart = max_lags_12-N+1;
    theTitle = ['Synchronising: Magenta ahead of Black: Shifting:' ...
        num2str(Nstart)];
    %display(theTitle);
    data_1 = data_1(Nstart:size(data_1,2));
    metric_1 = metric_1(Nstart:size(metric_1,2));
    N1 = size(metric_1,2);
end
grid on;
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
plot(t(1:N1),metric_1,'--k')
plot(t(1:N2),metric_2,'--m')
N=min(length(metric_1),length(metric_2));
[R12] = xcorr(metric_1(1:N)',metric_2(1:N)','coeff');
[corrValue]=max(R12);
title(theTitle)
end

