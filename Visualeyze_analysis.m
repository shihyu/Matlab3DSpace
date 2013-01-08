%LOADS a run and does the processing.
tic
clear all;
close all;
format compact;
format long;
initialiseprocessing();
filename = 'D:\Sofie-HDF-Format\win32build\Visualeyze_tests.h5'

StaticRun = '/StaticMeasurement';
VisualEyezRun = '/VeraNormal20';



% Read in the 3 markers for all sensors
[Steer, SteerGapArray] = Markers3D.readDataVicon(filename,VisualEyezRun,...
              'SteerRB','SteerLB','SteerFT','doInterpolate',true); 
          % true means the gaps in the data will be interpolated
[Roll,RollGapArray] = Markers3D.readDataVicon(filename,VisualEyezRun,...
            'RollRB','RollLB','RollFT','doInterpolate',true);     
[Pelvis, PelvisGapArray] = Markers3D.readDataVicon(filename,VisualEyezRun,...
            'PelvisRB','PelvisLB','PelvisFT','doInterpolate',true);          
[UpperBody, UpperBodyGapArray] = Markers3D.readDataVicon(filename,VisualEyezRun,...
            'UpperBodyRB','UpperBodyLB','UpperBodyFT','doInterpolate',true);
[RightThigh, RightThighGapArray] = Markers3D.readDataVicon(filename,VisualEyezRun,...
           'RightThighRB','RightThighLB','RightThighFT','doInterpolate',true);
[LeftThigh, LeftThighGapArray] = Markers3D.readDataVicon(filename,VisualEyezRun,...
            'LeftThighRB','LeftThighLB','LeftThighFT','doInterpolate',true);

%         size(Roll);
%         size(Pelvis);
        
% Get and plot the Roll, Pitch, Yaw for all sensors (in the local coordinate frame)
% The missed markerdata is not loaded in
    PlotStyle = '--m*'
 
[roll_Steer,pitch_Steer,yaw_Steer,t_Steer]=ThreeD.getAndPlotRPYt(Steer,'Steer',...
   false,'timeseries',PlotStyle);
hold on

% false means it will be plotted in a new figure
 [roll_Roll,pitch_Roll,yaw_Roll,t_Roll]=ThreeD.getAndPlotRPYt(Roll,'Roll',...
    false,'timeseries',PlotStyle);
 [roll_Pelvis,pitch_Pelvis,yaw_Pelvis,t_Pelvis]=ThreeD.getAndPlotRPYt(Pelvis,'Pelvis',...
    false,'timeseries',PlotStyle);
 [roll_UpperBody,pitch_UpperBody,yaw_UpperBody,t_UpperBody]=ThreeD.getAndPlotRPYt(UpperBody,'UpperBody',...
    false,'timeseries',PlotStyle);
[roll_RightThigh,pitch_RightThigh,yaw_RightThigh,t_RightThigh]=ThreeD.getAndPlotRPYt(RightThigh,'RightThigh',...
   false,'timeseries',PlotStyle);
 [roll_LeftThigh,pitch_LeftThigh,yaw_LeftThigh,t_LeftThigh]=ThreeD.getAndPlotRPYt(LeftThigh,'LeftThigh',...
    false,'timeseries',PlotStyle);

% Resample the data to fill in the gaps from the missing marker data and
% plot.

Fs = 100;
t_Steer = [t_Steer(1):1/Fs:t_Steer(length(t_Steer))];
t_Roll = [t_Roll(1):1/Fs:t_Roll(length(t_Roll))];
t_Pelvis = [t_Pelvis(1):1/Fs:t_Pelvis(length(t_Pelvis))];
t_UpperBody = [t_UpperBody(1):1/Fs:t_UpperBody(length(t_UpperBody))];
t_RightThigh = [t_RightThigh(1):1/Fs:t_RightThigh(length(t_RightThigh))];
t_LeftThigh = [t_LeftThigh(1):1/Fs:t_LeftThigh(length(t_LeftThigh))];


Steer_resampled = ThreeD.resample(Steer,t_Steer);
Roll_resampled = ThreeD.resample(Roll,t_Roll);
Pelvis_resampled = ThreeD.resample(Pelvis,t_Pelvis);
UpperBody_resampled = ThreeD.resample(UpperBody,t_UpperBody);
RightThigh_resampled = ThreeD.resample(RightThigh,t_RightThigh);
LeftThigh_resampled = ThreeD.resample(LeftThigh,t_LeftThigh);


% Get and plot the Roll, Pitch, Yaw for all sensors (in the local coordinate frame)
% After the resampling

ResampledPlotStyle = '--b*'

[roll_SteerR,pitch_SteerR,yaw_SteerR,t_SteerR]=ThreeD.getAndPlotRPYt(Steer_resampled,'Steer Resampled',...
    false,'timeseries',ResampledPlotStyle);
[roll_RollR,pitch_RollR,yaw_RollR,t_RollR]=ThreeD.getAndPlotRPYt(Roll_resampled,'Roll Resampled',...
    false,'timeseries',ResampledPlotStyle);
[roll_PelvisR,pitch_PelvisR,yaw_PelvisR,t_PelvisR]=ThreeD.getAndPlotRPYt(Pelvis_resampled,'Pelvis Resampled',...
    false,'timeseries',ResampledPlotStyle);
[roll_UpperBodyR,pitch_UpperBodyR,yaw_UpperBodyR,t_UpperBodyR]=ThreeD.getAndPlotRPYt(UpperBody_resampled,'UpperBody Resampled',...
    false,'timeseries',ResampledPlotStyle);
[roll_RightThighR,pitch_RightThighR,yaw_RightThighR,t_RightThighR]=ThreeD.getAndPlotRPYt(RightThigh_resampled,'RightThigh Resampled',...
    false,'timeseries',ResampledPlotStyle);
[roll_LeftThighR,pitch_LeftThighR,yaw_LeftThighR,t_LeftThighR]=ThreeD.getAndPlotRPYt(LeftThigh_resampled,'LeftThigh Resampled',...
    false,'timeseries',ResampledPlotStyle);


% Get the joint angles

% 'Callibrate' roll sensor, to get the angles in the global reference frame
Roll_callibrated = ThreeD.zeroTheRun(Roll_resampled,20,20);
[roll_Roll,pitch_Roll,yaw_Roll,t_Roll]=ThreeD.getAndPlotRPYt(Roll_callibrated,'Roll Resampled Callibrated',...
    false,'timeseries',ResampledPlotStyle);

%         % set the start time, to make all data the same length
% (build here something to check if all markers are visible..)
startTime = 0.1;
endTime = 10;
[t_Roll,roll_Roll]= ThreeD.setStartTime(t_Roll,roll_Roll, startTime,endTime,Fs);

% filter the roll angle
%filter settings:
Fs = 100;
freqLowPass = 15;
orderLowPass = 4;

[RollAngleFilt] = ThreeD.filterData(roll_Roll,Fs,freqLowPass,orderLowPass);

%plot the filtered and unfiltered roll angle
 figure;
plotStyle = 'b';
 plot(t_Roll,RollAngleFilt,plotStyle);
ylabel('Roll angle (degrees)');
xlabel('time (sec)');
title('roll angle of the bicycle');
hold on
plot(t_Roll,roll_Roll,'r');
legend('filtered', 'not filtered');

% Get the steering angle by taking the difference between the roll and
% steer sensor
[t_Steer,SteerAngle,SteerAngleRoll,SteerAnglePitch,SteerAngleYaw,...
               Roll_RollAngle,Roll_PitchAngle...
               Roll_YawAngle,figures] = ...
               ThreeD.getjointangles(Roll_resampled,Steer_resampled,'SteerAngle',20);
 % filter the steering angle
 [t_Steer,SteerAngleYaw]= ThreeD.setStartTime(t_Steer,SteerAngleYaw, startTime,endTime,Fs);
 [SteerAngleFilt] = ThreeD.filterData(SteerAngleYaw,Fs,freqLowPass,orderLowPass);

% plot the steering angle, filtered and un-filtered
  figure;
plotStyle = '--b*';
 plot(t_Steer,SteerAngleFilt,plotStyle);
ylabel('Steering angle (degrees)');
xlabel('time (sec)');
title('Steering angle');
hold on
plot(t_Steer,SteerAngleYaw,'--r*');
legend('filtered', 'not filtered');
 
% Get the lateral right knee angle, by getting the difference between the 
% pelvis and right leg sensor.
% (is this right?? check with model)
[t_RightKnee,RightKneeAngle,RightKneeAngleRoll,RightKneeAnglePitch,RightKneeAngleYaw,...
               Pelvis_RollAngle,Pelvis_PitchAngle...
               Roll_YawAngle,figures] = ...
               ThreeD.getjointangles(Pelvis_resampled,RightThigh_resampled,'RightKneeAngle',20);
% filter the lateral right knee angle
   [t_RightKnee,RightKneeAngleYaw]= ThreeD.setStartTime(t_RightKnee,RightKneeAngleYaw, startTime,endTime,Fs);
  [RightKneeAngleFilt] = ThreeD.filterData(RightKneeAngleYaw,Fs,freqLowPass,orderLowPass);
 
% plot the lateral right knee angle, filtered and un-filtered
  figure;
plotStyle = '--b*';
 plot(t_RightKnee,RightKneeAngleFilt,plotStyle);
ylabel('Right Thigh angle (degrees)');
xlabel('time (sec)');
title('Right Thigh angle');
hold on
plot(t_RightKnee,RightKneeAngleYaw,'--r*');
legend('filtered', 'not filtered');
 
% Get the lateral left knee angle, by getting the difference between the 
% pelvis and left leg sensor.
% (is this right?? check with model)             
 angleName = 'LeftKneeAngle';
[t_LeftKnee,LeftKneeAngle_t,LeftKneeAngleRoll,LeftKneeAnglePitch,LeftKneeAngleYaw,...
                Pelvis_RollAngle,Pelvis_PitchAngle...
                Roll_YawAngle,figures] = ...
                ThreeD.getjointangles(Pelvis_resampled,LeftThigh_resampled,angleName,20);
   
 [t_LeftKnee,LeftKneeAngleYaw]= ThreeD.setStartTime(t_LeftKnee,LeftKneeAngleYaw, startTime,endTime,Fs);      
            
 [LeftKneeAngleFilt] = ThreeD.filterData(LeftKneeAngleYaw,Fs,freqLowPass,orderLowPass);
 
 % plot the lateral left knee angle, filtered and un-filtered
  figure;
plotStyle = '--b*';
 plot(t_LeftKnee,LeftKneeAngleFilt,plotStyle);
ylabel('Left Thigh angle (degrees)');
xlabel('time (sec)');
title('Left Thigh angle');
hold on
plot(t_LeftKnee,LeftKneeAngleYaw,'--r*');
legend('filtered', 'not filtered');
            
 % get the upper body lean angle, by getting the difference between the pelvis 
 % sensor and upper body sensor
angleName = 'UpperBodyLean';            
[t_UpperBody,UpperBodyAngle_t,UpperBodyRoll,UpperBodyPitch,UpperBodyYaw,...
                Pelvis_RollAngle,Pelvis_PitchAngle...
                Roll_YawAngle,figures] = ...
                ThreeD.getjointangles(Pelvis_resampled,UpperBody_resampled,angleName,20);
   [t_UpperBody,UpperBodyRoll]= ThreeD.setStartTime(t_UpperBody,UpperBodyRoll, startTime,endTime,Fs);          
% filter the upper body lean angle
   [UpperBodyLeanFilt] = ThreeD.filterData(UpperBodyRoll,Fs,freqLowPass,orderLowPass);

 % plot the upper body lean angle, filtered and un-filtered
  figure;
plotStyle = '--b*';
 plot(t_UpperBody,UpperBodyLeanFilt,plotStyle);
ylabel('Upper Body Lean angle (degrees)');
xlabel('time (sec)');
title('Upper Body Lean angle');
hold on
plot(t_UpperBody,UpperBodyRoll,'--r*');
legend('filtered', 'not filtered');



% 
% 
% % Write results to file for import in Adams
% 
% 
filename = [strrep(VisualEyezRun, '/', ''), '.dat'];
fid = fopen(filename,'w');

fprintf(fid, '%s %s %s %s %s %s\n', 'Time', 'rollAngle', 'SteeringAngle', 'leftKneeAngle', 'rightKneeAngle', 'upperBodyLean');


for i = 1: length(t_Roll)
    fprintf(fid, '%d %d %d %d %d %d\n',t_Roll , RollAngleFilt, SteerAngleFilt, RightKneeAngleFilt, RightKneeAngleFilt, UpperBodyLeanFilt) ;
end

fclose(fid);

% 



            
