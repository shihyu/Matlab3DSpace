function [jointAngleTimestamps,jointAngle_t,jointAngleRoll,jointAnglePitch,jointAngleYaw,...
    sensor1RollAngle,sensor1PitchAngle...
    sensor1YawAngle,figures] = ...
    getjointangles(sensor1_t,sensor2_t,angleName,callibrateStart)
%GETJOINTANGLES Calculates the angles between two sensors.
display(['%%%%%%%%%%%%%%Processing Angle:' angleName ' %%%%%%%%%%%%%%'])
sensor2Style = '--b.';
sensor1Style = '--r.';
jointAnglePlotStyle = '--m.';
%ThreeD.plotRun([sensor1_t;sensor2_t]);
figures =[];
[~,~,~,~,unCallibratedOrignalFigure]=...
    ThreeD.getAndPlotRPYt(sensor2_t,...
    ['GETJOINTANGLES: Uncalibrated original sensor position: Sensor 1(' sensor1Style...
    ') Sensor 2 (' sensor2Style ')' ],false,'timeseries',sensor2Style); 
figures = [figures unCallibratedOrignalFigure];
[sensor1RollAngle,sensor1PitchAngle,sensor1YawAngle,~]=...
    ThreeD.getAndPlotRPYt(sensor1_t,...
    '',unCallibratedOrignalFigure,'timeseries',sensor1Style);

%joint Angle:
% psiS = Steering Column Frame
% psiR = Roll sensor frame, on the bicycle frame.
% psi0 = Origin frame
% For a point q
% q_psi0 = H_S0*q_psiS => We Have, sensor2_t
% q_psi0 = H_R0*q_psiR => We have, sensor1_t
% q_psiR = HSR*q_psiS => We want H12 => Now lets solve:
% H_SR =  (H_R0)^-1*HS0
%In code: sensor2_t = H_RS and sensor1_t = H_R0
% Thus joint_angle = H_SR
% jointAngle_t = sensor1_t'*sensor2_t
jointAngle_t = ThreeD.cellInverseMultiply(sensor1_t,sensor2_t);

ThreeD.getAndPlotRPYt(jointAngle_t,...
    ['GETJOINTANGLES: Calculated '  angleName '(un-callibrated).' ],...
    false,'timeseries',jointAnglePlotStyle);
%Callibrate to see what happens! We want to make this frame the origin...
jointAngle_t = ThreeD.callibrate(jointAngle_t,callibrateStart,30);
[jointAngleRoll,jointAnglePitch,jointAngleYaw,jointAngleTimestamps,steeringAnglePlot] = ...
    ThreeD.getAndPlotRPYt(jointAngle_t,...
    ['GETJOINTANGLES: Calulated '  angleName '(callibrated).' ],...
    false,'timeseries',jointAnglePlotStyle);
figures = [figures steeringAnglePlot];
%Make sure they all have the same length.
minLength = min(length(jointAngleTimestamps),length(sensor1RollAngle));
sensor1RollAngle = sensor1RollAngle(1:minLength);
sensor1PitchAngle = sensor1PitchAngle(1:minLength);
sensor1YawAngle = sensor1YawAngle(1:minLength);