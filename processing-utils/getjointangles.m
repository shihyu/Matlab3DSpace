function [jointAngle_t,jointAngleRoll,jointAnglePitch,jointAngleYaw,...
    sensor1RollAngle,sensor1PitchAngle...
    sensor1YawAngle] = ...
    getjointangles(sensor1_t,sensor2_t,angleName,callibrateStart)
%GETJOINTANGLES Calculates the angles between two sensors.
display(['%%%%%%%%%%%%%%Processing Angle:' angleName ' %%%%%%%%%%%%%%'])
steerSensorPlotStyle = '--ob';
rollPlotStyle = '--or';
jointAnglePlotStyle = '--om';
%ThreeD.plotRun([sensor1_t;sensor2_t]);
[roll2,pitch2,yaw2,t,unCallibratedFigure]=...
    ThreeD.getAndPlotRPYt(sensor2_t,...
    [angleName ' Sensor2'],false,'timeseries',steerSensorPlotStyle);   
[sensor1RollAngle,sensor1PitchAngle,sensor1YawAngle,t]=...
    ThreeD.getAndPlotRPYt(sensor1_t,...
    [angleName ' Sensor1'],unCallibratedFigure,'timeseries',rollPlotStyle);
%joint Angle:
% q2 = joint Column sensor.
% q1 = Roll sensor, on the frame.
% q0 = Origin
% q2 = H_02*q0 => We Have, sensor2_t
% q1 = H_01*q0 => We have, sensor1_t
% q2 = H12*q1 => We want H12 => Now lets solve:
% H_02*q0 =  H12*H_01*q0
% H_02 =  H12*H_01
% H12*H_01*(H_01)^-1 = H_02*(H_01)^-1
% H12*I = H_02*(H_01)^-1
% H12 = H_02*(H_01)^-1
%In code: sensor2_t = H_02 and sensor1_t = H_01
% Thus joint_angle = H12
% jointAngle_t = sensor2_t*sensor1_t' = 
% ThreeD.cellminus(sensor2_t,sensor1_t)
jointAngle_t = ThreeD.cellminus(sensor2_t,sensor1_t);
jointAngle_t = ThreeD.inverseMultiply(sensor1_t,sensor2_t);

ThreeD.getAndPlotRPYt(jointAngle_t,...
    [angleName ' ANGLE-UNCALLIBRATED'],false,'timeseries',jointAnglePlotStyle);
%Callibrate to see what happens! We want to make this frame the origin...
jointAngle_t = ThreeD.callibrate(jointAngle_t,callibrateStart,30);
[jointAngleRoll,jointAnglePitch,jointAngleYaw,t] = ...
    ThreeD.getAndPlotRPYt(jointAngle_t,[angleName ' ANGLE-CALLIBRATED'],...
    false,'timeseries',jointAnglePlotStyle);