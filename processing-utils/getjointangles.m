function [t,jointAngle_t,jointAngleRoll,jointAnglePitch,jointAngleYaw,...
    sensor1RollAngle,sensor1PitchAngle...
    sensor1YawAngle] = ...
    getjointangles(sensor1_t,sensor2_t,angleName,callibrateStart)
%GETJOINTANGLES Calculates the angles between two sensors.
display(['%%%%%%%%%%%%%%Processing Angle:' angleName ' %%%%%%%%%%%%%%'])
steerSensorPlotStyle = '--b.';
rollPlotStyle = '--r.';
jointAnglePlotStyle = '--m.';
%ThreeD.plotRun([sensor1_t;sensor2_t]);
[roll2,pitch2,yaw2,t,unCallibratedFigure]=...
    ThreeD.getAndPlotRPYt(sensor2_t,...
    [angleName ' Sensor2'],false,'timeseries',steerSensorPlotStyle);   
[sensor1RollAngle,sensor1PitchAngle,sensor1YawAngle,t]=...
    ThreeD.getAndPlotRPYt(sensor1_t,...
    [angleName ' Sensor1'],unCallibratedFigure,'timeseries',rollPlotStyle);
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
    [angleName ' ANGLE-UNCALLIBRATED'],false,'timeseries',jointAnglePlotStyle);
%Callibrate to see what happens! We want to make this frame the origin...
jointAngle_t = ThreeD.callibrate(jointAngle_t,callibrateStart,30);
[jointAngleRoll,jointAnglePitch,jointAngleYaw,t] = ...
    ThreeD.getAndPlotRPYt(jointAngle_t,[angleName ' ANGLE-CALLIBRATED'],...
    false,'timeseries',jointAnglePlotStyle);