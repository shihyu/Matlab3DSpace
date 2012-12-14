%LOADS a run and does the processing.
tic
clear all;
close all;
format compact;
format long;
initialiseprocessing();
filename = 'D:\Git-Repository-Vera\code\Visualeyze\Visualeyze_tests.h5'

VisualEyezRun = '/Upperbody';



% Read in the 3 markers for all sensors
[Steer] = Markers3D.readDataVicon(filename,VisualEyezRun,...
            'SteerRB','SteerLB','SteerFT','doFilter',false);        
[Roll] = Markers3D.readDataVicon(filename,VisualEyezRun,...
            'RollRB','RollLB','RollFT','doFilter',false);     
[Pelvis] = Markers3D.readDataVicon(filename,VisualEyezRun,...
            'PelvisRB','PelvisLB','PelvisFT','doFilter',false);          
[UpperBody] = Markers3D.readDataVicon(filename,VisualEyezRun,...
            'UpperBodyRB','UpperBodyLB','UpperBodyFT','doFilter',false);
[RightThigh] = Markers3D.readDataVicon(filename,VisualEyezRun,...
            'RightThighRB','RightThighLB','RightThighFT','doFilter',false);
[LeftThigh] = Markers3D.readDataVicon(filename,VisualEyezRun,...
            'LeftThighRB','LeftThighLB','LeftThighFT','doFilter',false);
        
% Get and plot the Roll, Pitch, Yaw for all sensors (in the local coordinate frame)
% The missed markerdata is not loaded in
    PlotStyle = '--m*'
 
 [roll_Steer,pitch_Steer,yaw_Steer,t_Steer]=ThreeD.getAndPlotRPYt(Steer,'Steer',...
    false,'timeseries',PlotStyle);
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

N = length(t_Steer);
t_end = t_Steer(N);
t_start = t_Steer(1);
Fs = 100;
t = [t_start:1/Fs:t_end];

Steer_resampled = ThreeD.resample(Steer,t);
Roll_resampled = ThreeD.resample(Roll,t);
Pelvis_resampled = ThreeD.resample(Pelvis,t);
UpperBody_resampled = ThreeD.resample(UpperBody,t);
RightThigh_resampled = ThreeD.resample(RightThigh,t);
LeftThigh_resampled = ThreeD.resample(LeftThigh,t);

ResampledPlotStyle = '--b*'

[roll_SteerR,pitch_SteerR,yaw_SteerR,t_SteerR]=ThreeD.getAndPlotRPYt(Steer_resampled,'Steer Resampled',...
    false,'timeseries',ResampledPlotStyle);
[roll_RollR,pitch_RollR,yaw_RollR,t_RollR]=ThreeD.getAndPlotRPYt(Roll_resampled,'Roll Resampled',...
    false,'timeseries',ResampledPlotStyle);
[roll_PelvisR,pitch_PelvisR,yaw_PelvisR,t_PelvisR]=ThreeD.getAndPlotRPYt(Pelvis_resampled,'Pelvis Resampled',...
    false,'timeseries',ResampledPlotStyle);
[roll_UpperBodyR,pitch_UpperBodyR,yaw_UpperBodyR,t_UpperBodyR]=ThreeD.getAndPlotRPYt(SUpperBody_resampled,'UpperBody Resampled',...
    false,'timeseries',ResampledPlotStyle);
[roll_RightThighR,pitch_RightThighR,yaw_RightThighR,t_RightThighR]=ThreeD.getAndPlotRPYt(RightThigh_resampled,'RightThigh Resampled',...
    false,'timeseries',ResampledPlotStyle);
[roll_LeftThighR,pitch_LeftThighR,yaw_LeftThighR,t_LeftThighR]=ThreeD.getAndPlotRPYt(LeftThigh_resampled,'LeftThigh Resampled',...
    false,'timeseries',ResampledPlotStyle);

% 'Callibrate' roll sensor, to get the angles in the global reference frame
UpperBody_calllibrated = ThreeD.callibrate(Roll_resampled,20,20);
[roll_Roll,pitch_Roll,yaw_Roll,t_Roll]=ThreeD.getAndPlotRPYt(Roll_calllibrated,'Roll Resampled Callibrated',...
    false,'timeseries',ResampledPlotStyle);

% Get the joint angles
[t_Steer,SteerAngle,SteerAngleRoll,SteerAnglePitch,SteerAngleYaw,...
                Roll_RollAngle,Roll_PitchAngle...
                Roll_YawAngle,figures] = ...
                ThreeD.getjointangles(Roll_resampled,Steer_resampled,SteerAngle,20)
            
            [t_RightKnee,RightKneeAngle,RightKneeAngleRoll,RightKneeAnglePitch,RightKneeAngleYaw,...
                Pelvis_RollAngle,Pelvis_PitchAngle...
                Roll_YawAngle,figures] = ...
                ThreeD.getjointangles(Pelvis_resampled,RightThigh_resampled,RightKneeAngle,20)
            
                        [t_LeftKnee,LeftKneeAngle,LeftKneeAngleRoll,LeftKneeAnglePitch,LeftKneeAngleYaw,...
                Pelvis_RollAngle,Pelvis_PitchAngle...
                Roll_YawAngle,figures] = ...
                ThreeD.getjointangles(Pelvis_resampled,LeftThigh_resampled,LeftKneeAngle,20)
