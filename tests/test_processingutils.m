function test_suite = test_processingutils
initTestSuite;
function test_synchronise
clear all
close all
Fs = 120;
data1 = [0 1 0 0 0];
data2 = [0 0 0 1 0 0];
metric1 = [0 1 0 0 0];
metric2 = [0 0 0 1 0 0];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,1,false)
assertEqual(data1,result1);
assertEqual(result2,[0 1 0 0]);
assertEqual(data1(1:4),result2);
clear all
close all
Fs = 120;
data1 = [0 0 0 1 0 0];
data2 = [0 1 0 0 0];
metric1 = [0 0 0 1 0 0];
metric2 = [0 1 0 0 0];

[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,1,false)
assertEqual(data2,result2);
assertEqual(result1,[0 1 0 0]);
assertEqual(data2(1:4),result1);

Fs = 120;
data1 = [0 1 0 0 2 0 0 1];
data2 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
metric1 = [0 1 0 0 2 0 0 1];
metric2 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,1,false)
assertEqual(data1,result1);
assertEqual(result2,[0 1 0 0 4 0 0 2 0 1 1 3]);

Fs = 120;
data1 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
data2 = [0 1 0 0 2 0 0 1];
metric1 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
metric2 = [0 1 0 0 2 0 0 1];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,1,false)
assertEqual(data2,result2);
assertEqual(result1,[0 1 0 0 4 0 0 2 0 1 1 3]);

Fs = 1;
data1 = [0 1 0 0 2 0 0 1];
data2 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
metric1 = [0 1 0 0 2 0 0 1];
metric2 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,2,false)
assertEqual(data1,result1);
assertEqual(result2,[0 1 0 0 4 0 0 2 0 1 1 3]);

Fs = 1;
data1 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
data2 = [0 1 0 0 2 0 0 1];
metric1 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
metric2 = [0 1 0 0 2 0 0 1];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,2,false)
assertEqual(data2,...
    result2);
assertEqual(result1,[0  1 0 0 4 0 0 2 0 1 1 3]);

Fs = 1;
data1 = [0 0 0 0 1 0 0 8 0 0 1 0 0 2 0 0 1 0 1 1 3];
data2 = [0 0 1 0 0 2 0 0 1];
metric1 = [0 0 0 0 1 0 0 8 0 0 1 0 0 2 0 0 1 0 1 1 3];
metric2 = [0 0 1 0 0 2 0 0 1];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,2,false)
assertEqual(data2,...
    result2);
assertEqual(result1,[0 0 1 0 0 8 0 0 1 0 0 2 0 0 1 0 1 1 3]);


[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,2,true)
assertEqual([1 0 0 2 0 0 1],...
    result2);
assertEqual(result1,[ 1 0 0 8 0 0 1 0 0 2 0 0 1 0 1 1 3]);

% function test_getbicycleangles
% filename='test-data/test-data.h5';
% runName = 'testrun';
% figures = [];
% [roll_angle,steering_angle] = getbicycleangles(...
%     figures,...
%     filename,runName,...
%     200,200,1,2,1,0,false,0);

function test_synchroniseWithRespectToRPY
filename='./test-data/test-data.h5'
runName = 'promove'
[steering_t] = Quat3D.readDataPromove(filename,runName,1,10,200);
[roll_t] = Quat3D.readDataPromove(filename,runName,...
    2,200,200);
steering_t = ThreeD.changeStartTime(steering_t,0);
roll_t = ThreeD.changeStartTime(roll_t,0);

[roll_r,pitch_r,yaw_r]=ThreeD.getRPYt(roll_t,true);
[roll_s,pitch_s,yaw_s]=ThreeD.getRPYt(steering_t,true);

[roll_t,steering_t] = synchroniseWithRespectToRPY(...
    roll_r,pitch_r,yaw_r,roll_t,...
    roll_s,pitch_s,yaw_s,steering_t,200);
[roll_r,pitch_r,yaw_r,t]=ThreeD.getRPYt(roll_t,true);
[roll_s,pitch_s,yaw_s,t_s]=ThreeD.getRPYt(steering_t,true);
figure
minSize = min(length(steering_t),length(roll_t));
ThreeD.plotRPY(...
    roll_r(1:minSize),pitch_r(1:minSize),yaw_r(1:minSize),t(1:minSize),...
    true,'timeseries');
ThreeD.plotRPY(...
    roll_s(1:minSize),pitch_s(1:minSize),yaw_s(1:minSize),...
    t_s(1:minSize),true,'timeseries');

