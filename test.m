
close all;
clc;
filename='D:/Adams/Vera/Vers_2011_09_27/output_firstpaper/test/15sec_45ms.h5'

runs = h5Reader.listRuns(filename)
runName = '/bicycletest45ms';
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

[roll_t] = ViconThreeMarkers.readDataAdams(filename,runName,'RBT','LBT','FTN');
%You can then callibrate the data. It takes the ‘average’
%quaternion of the 10 quaternions starting at 100 samples
%into the experiment and then uses this as the zero frame.
%All the samples in the set are then inversely rotated by
%quaternion.

%tm_t = ThreeMarkers.callibrate(tm_t,100,10);

%Get the Roll Pitch and Yaw of both sensors.
[roll_r,pitch_r,yaw_r,t] = ThreeMarkers.getRPYt(roll_t,inDegrees);
[roll_s,pitch_s,yaw_s,t] = ThreeMarkers.getRPYt(steer_t,inDegrees);
%Plot this!

% This is the roll, pitch and yaw of the markers on the rear frame (figure
% 1)
ThreeMarkers.plotRPY(...
    roll_r,pitch_r,yaw_r,true,Fs1,t,0);
title('steer sensor - uncalibrated');
% This is the roll, pitch and yaw of the markers on the front fork (figure
% 2)
figure
ThreeMarkers.plotRPY(...
    roll_s,pitch_s,yaw_s,true,Fs1,t,0);
title('steer sensor - uncalibrated');

%Calibrate rearframe sensor
callibrateStart = 66;
orig_roll_t = roll_t;
roll_t = ThreeMarkers.callibrate(roll_t,callibrateStart,1);

%Calibrate front fork sensor
callibrateStart = 66;
orig_steer_t = steer_t;
steer_t = ThreeMarkers.callibrate(steer_t,callibrateStart,1);

% Get roll, pitch, yaw after calibration
[roll_r,pitch_r,yaw_r,t]=ThreeMarkers.getRPYt(roll_t,true);
[roll_s,pitch_s,yaw_s,t]=ThreeMarkers.getRPYt(steer_t,true);

%plot the calibrated roll, pitch and yaw
figure
ThreeMarkers.plotRPY(...
    roll_r,pitch_r,yaw_r,true,Fs1,t,0);
figure
ThreeMarkers.plotRPY(...
    roll_s,pitch_s,yaw_s,true,Fs1,t,0);

% Synchronisation of the Adams data is not necessary.
% Now calculate the steering angle and the roll angle of the bicycle
% The steering angle is defined as the difference between the yaw of the
% sensor on the rear frame and the sensor on the front fork
% The roll angle of the bicyle is the same as the roll angle of the rear
% frame sensor.


%Un - Callibrated steer

diff_t = ThreeMarkers.cellminus(orig_roll_t,orig_steer_t);
[roll_d,pitch_d,yaw_d,t]=ThreeMarkers.getRPYt(diff_t,true);

steering_angle = yaw_d;
roll_angle = roll_r;

figure
plot(t,steering_angle); 
title('uncalibrated - steering angle');
%figure
%plot(t,roll_angle);

%Callibrated steer

diff_t = ThreeMarkers.cellminus(roll_t,steer_t);
theLength = length(diff_t)
steering_angle_t = cell(1,theLength);
parfor i = 1:theLength
    steering_angle_t{i} = steer_t{i}.*roll_t{i}';
end;
[roll_d,pitch_d,yaw_d,t]=ThreeMarkers.getRPYt(steering_angle_t,true);
steering_angle = yaw_d;
roll_angle = roll_r;

figure
plot(t,steering_angle);
title('calibrated - yaw');

figure
plot(t,roll_d); 
title('calibrated - roll');
figure
plot(t,pitch_d); 
title('calibrated - pitch');



figure
plot(t,roll_angle);
title('calculated roll angle');
   
