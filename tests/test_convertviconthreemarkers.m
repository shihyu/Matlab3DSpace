function test_suite = test_arrayToString
%test_arrayToString Unit test for arrayToString.

%   Steven L. Eddins
%   Copyright 2009 The MathWorks, Inc.



initTestSuite;

function test_ConvertViconThreeMarkers
rightback = [1 0 0];
leftback = [-1 0 0];
front = [0 1 0];
theTimestamp = 2.3;
cvt = ConvertViconThreeMarkers(rightback,...
        leftback,front,theTimestamp)
assertEqual(cvt.points_psi_0,[1 -1 0 0; 0 0 1 0; 0 0 0 1; 1 1 1 1]);

assertEqual(cvt.H_psi_v_psi_0,eye(4));
assertEqual(cvt.points_psi_0,cvt.points_psi_v);


rightback = [-1 0 0];
leftback = [1 0 0];
front = [0 1 0];
theTimestamp = 2.3;
cvt = ConvertViconThreeMarkers(rightback,...
        leftback,front,theTimestamp)
assertEqual(cvt.H_psi_v_psi_0,[-1 0 0 0; 0 1 0 0; 0 0 -1 0; 0 0 0 1]);
assertEqual([-1 1 0 0; 0 0 1 0; 0 0 0 -1; 1 1 1 1],cvt.points_psi_v);

rightback = [1 1 2];
leftback = [-1 1 2];
front = [0 2 2];
theTimestamp = 2.3;
cvt = ConvertViconThreeMarkers(rightback,...
        leftback,front,theTimestamp)
assertEqual(cvt.points_psi_0,[1 -1 0 0; 0 0 1 0; 0 0 0 1; 1 1 1 1]);

assertEqual(cvt.H_psi_v_psi_0,eye(4));
assertEqual(cvt.points_psi_0,cvt.points_psi_v);

rightback = [1 0 0];
leftback = [-1 0 0];
front = [0 0 1];
theta = -pi/2;
H = [ 1     0           0        0
      0  cos(theta) -sin(theta)  0
      0  sin(theta)  cos(theta)  0
      0     0           0        1];
cvt = ConvertViconThreeMarkers(rightback,...
        leftback,front,theTimestamp)
assertElementsAlmostEqual(cvt.H_psi_v_psi_0,H);
assertElementsAlmostEqual([1 -1 0 0; 0 0 0 -1; 0 0 1 0; 1 1 1 1],cvt.points_psi_v);

rightback = [4.4 5.5 -6.25];
leftback = [2.4 5.5 -6.25];
front = [3.4 5.5 -5.25];
theta = -pi/2;
H = [ 1     0           0        0
      0  cos(theta) -sin(theta)  0
      0  sin(theta)  cos(theta)  0
      0     0           0        1];
cvt = ConvertViconThreeMarkers(rightback,...
        leftback,front,theTimestamp)
assertElementsAlmostEqual(cvt.H_psi_v_psi_0,H);
assertElementsAlmostEqual([1 -1 0 0; 0 0 0 -1; 0 0 1 0; 1 1 1 1],cvt.points_psi_v);

function test_angleDiff
x = 0;
y = pi/2;
assertEqual(angleDifference(x,y),-pi/2);
assertEqual(angleDifference(y,x),pi/2);
x=pi; 
y=pi-1;
assertEqual(angleDifference(x,y),1);
assertEqual(angleDifference(y,x),-1);
x=-pi/2; 
y=pi/2-1;
assertEqual(angleDifference(x,y),-(pi-1));
assertEqual(angleDifference(y,x),(pi-1));
x = pi-0.1;
y = pi;
assertElementsAlmostEqual(angleDifference(x,y),-0.1);
assertElementsAlmostEqual(angleDifference(y,x),0.1);
x = pi-0.1;
y = pi+0.1;
assertElementsAlmostEqual(angleDifference(x,y),-0.2);
assertElementsAlmostEqual(angleDifference(y,x),0.2);
x = pi;
y = pi+0.1;
assertElementsAlmostEqual(angleDifference(x,y),-0.1);
assertElementsAlmostEqual(angleDifference(y,x),0.1);
x = 2*pi-0.1;
y = 2*pi;
assertElementsAlmostEqual(angleDifference(x,y),-0.1);
assertElementsAlmostEqual(angleDifference(y,x),0.1);
x = 2*pi-0.1;
y = 2*pi+0.1;
assertElementsAlmostEqual(angleDifference(x,y),-0.2);
assertElementsAlmostEqual(angleDifference(y,x),0.2);
x = 2*pi;
y = 2*pi+0.1;
assertElementsAlmostEqual(angleDifference(x,y),-0.1);
assertElementsAlmostEqual(angleDifference(y,x),0.1);
x = 2*pi;
y = 0;
assertElementsAlmostEqual(angleDifference(x,y),0);
assertElementsAlmostEqual(angleDifference(y,x),0);

function test_synchronise
Fs = 120;
data1 = [0 1 0 0 0];
data2 = [0 0 1 0 0];
metric1 = [0 1 0 0 0];
metric2 = [0 0 1 0 0];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,1)
assertEqual(data1,result1);
assertEqual(result2,[0 1 0 0]);
assertEqual(data1(1:4),result2);

