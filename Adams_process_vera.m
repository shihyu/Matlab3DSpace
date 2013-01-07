%Adams handling.

tic;
clc;
close all;
format long;
format compact;
filename='./testrun.h5';
%runName='falling';
runName='testrun';
Fs_ImuMeasured=33.3333;
[steering_t,steering_sync_t] = ...
            Markers3D.readDataAdams(filename,runName,...
            'RBO','LBO','FON','kabsch');
 [roll_t,roll_sync_t] = ...
            Markers3D.readDataAdams(filename,runName,...
            'RBT','LBT','FTN','kabsch');
reader = adamsReader(filename,runName);
adamsData = reader.readData(false);

%ThreeMarkers.plotRun([roll_t;steering_t]);
[roll_s,pitch_s,yaw_s,t]=ThreeD.getRPYt(steering_t,true);   
[roll_r,pitch_r,yaw_r,t]=ThreeD.getRPYt(roll_t,true);


%Steering Angle:
% q2 = Steering Column sensor.
% q1 = Roll sensor, on the frame.
% q0 = Origin
% q2 = H_0_2*q0 => We Have, steering_t
% q1 = H_0_1*q0 => We have, roll_t
% q2 = H_1_2*q1 => We want H_1_2 => Now lets solve:
% H_0_2*q0 =  H_1_2*H_0_1*q0
% H_0_2 =  H_1_2*H_0_1
% H_1_2*H_0_1*(H_0_1)^-1 = H_0_2*(H_0_1)^-1
% H_1_2*I = H_0_2*(H_0_1)^-1
% H_1_2 = H_0_2*(H_0_1)^-1
%In code: steering_t = H_0_2 and roll_t = H_0_1
% Thus steering_angle = H_1_2
% steer_angle_t = steering_t*roll_t' = 
% ThreeMarkers.cellminus(steering_t,roll_t)
steer_angle_t = ThreeD.cellminus(steering_t,roll_t);
[steer_roll_s,steer_pitch_s,steer_yaw_s]=...
    ThreeD.getAndPlotRPY(steer_angle_t,'STEERING ANGLE-UNCALLIBRATED');
callibrateStart=floor(0.1*Fs_ImuMeasured);
%Callibrate to see what happens! We want to make this frame the origin...
steer_angle_t = ThreeD.zeroTheRun(steer_angle_t,callibrateStart,30);
[steer_roll_c,steer_pitch_c,steer_yaw_c,t2] = ThreeD.getAndPlotRPY(steer_angle_t,'STEERING ANGLE-CALLIBRATED');
%And the steering angle sits in the yaw access.
%The roll angle is the roll angle of the rearframe
%Plot the actual roll and steering angle of the bicycle
ThreeD.plotRS(roll_r,steer_yaw_s,true,Fs_ImuMeasured,t,0);


% find maxima, starting from t = 3s. (perturbation time)
[pks_roll,locs_roll] = findpeaks(roll_r((Fs_ImuMeasured*3):length(roll_r)));
figure;

%plot the roll angle and the maxima in one figure
plot(t,roll_r);
hold on

plot(((locs_roll./Fs_ImuMeasured)+(3)),pks_roll);

% typeOfPlot=0;
% YLABEL='(degrees)';
% ThreeMarkers.plotAngle(t,roll_r,typeOfPlot,YLABEL)
%             hold on;
%             grid on;
% ThreeMarkers.plotAngle(((locs_roll./Fs_ImuMeasured)+(3)),pks_roll,0,'(degrees)')



% curve fitting of the maxima with an exponential function, to find lambda
% lambda negative = stable, lambda positive = unstable
curve = fit( t(locs_roll)', pks_roll', 'exp1');
figure;
plot((locs_roll./Fs_ImuMeasured),pks_roll);
hold on
plot(curve);
coef = coeffvalues(curve);
form = formula(curve);
lambda = coef(2);
toc
