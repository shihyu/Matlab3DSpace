
close all;
clc;
filename='D:/Adams/Vera/Vers_2011_09_27/output_firstpaper/output_bicycle_3ms.h5'

runs = h5Reader.listRuns(filename)
runName = '/bicycle3ms';
node1 = 1;
% Fs1 (sample frequency should be the same as used in Adams.
Fs1 = 500/15;
poolSize = matlabpool('size');
if poolSize == 0
matlabpool local 4;
end
inDegrees=true;
%Read the quaternion data. This is static function is custom
%for CSV files that Intertia Technology outputs from their
%ProMoveGUI.
[steer_t] = ViconThreeMarkers.readDataAdams(filename,runName,'RBO','LBO','FON');

[tm_t] = ViconThreeMarkers.readDataAdams(filename,runName,'RBT','LBT','FTN');
%You can then callibrate the data. It takes the ‘average’
%quaternion of the 10 quaternions starting at 100 samples
%into the experiment and then uses this as the zero frame.
%All the samples in the set are then inversely rotated by
%quaternion.

%tm_t = ThreeMarkers.callibrate(tm_t,100,10);

%Get the Roll Pitch and Yaw of the experiment.
[roll_t,pitch_t,yaw_t] = ThreeMarkers.getRPYt(tm_t,inDegrees);
%Plot this!

% This is the roll, pitch and yaw of the markers on the rear frame (figure
% 1)
ThreeMarkers.plotRPY(roll_t,pitch_t,yaw_t,inDegrees,Fs1);
% This is the roll, pitch and yaw of the markers on the front fork (figure
% 2)
figure
[roll_t,pitch_t,yaw_t] = ThreeMarkers.getRPYt(steer_t,inDegrees);
ThreeMarkers.plotRPY(roll_t,pitch_t,yaw_t,inDegrees,Fs1);

%Calibrate rearframe sensor
callibrateStart = 
roll_t = ThreeMarkers.callibrate(roll_t,callibrateStart,10);







[steering_angle,roll_angle,speed,cadence,ant_t,figures] = getbicycleangles(figures,...
    'D:/Adams/Vera/Vers_2011_09_27/output_firstpaper/output_bicycle_3ms.h5','/bicycle3ms',Fs1,Fs1,...
    1,2,callibrateStart,plotRuns,synchroniseTheImus,imuOrAdams)