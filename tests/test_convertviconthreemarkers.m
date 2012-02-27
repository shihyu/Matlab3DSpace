function test_suite = test_arrayToString
%test_arrayToString Unit test for arrayToString.

%   Steven L. Eddins
%   Copyright 2009 The MathWorks, Inc.



initTestSuite;

function test_readLeafData
rightback = [1 0 0];
leftback = [-1 0 0];
front = [0 1 0];
theTimestamp = 2.3;
cvt = ConvertViconThreeMarkers(rightback,...
        leftback,front,theTimestamp);
assertEqual(cvt.points_psi_0,[1 -1 0 0; 0 0 1 0; 0 0 0 1; 1 1 1 1]);

assertEqual(cvt.H_psi_v_psi_0,eye(4));
assertEqual(cvt.points_psi_0,cvt.points_psi_v);
cvt.quaternion


rightback = [-1 0 0];
leftback = [1 0 0];
front = [0 1 0];
theTimestamp = 2.3;
cvt = ConvertViconThreeMarkers(rightback,...
        leftback,front,theTimestamp);
assertEqual(cvt.H_psi_v_psi_0,[-1 0 0 0; 0 1 0 0; 0 0 -1 0; 0 0 0 1]);
assertEqual([-1 1 0 0; 0 0 1 0; 0 0 0 -1; 1 1 1 1],cvt.points_psi_v);
cvt.quaternion


rightback = [1 1 2];
leftback = [-1 1 2];
front = [0 2 2];
theTimestamp = 2.3;
cvt = ConvertViconThreeMarkers(rightback,...
        leftback,front,theTimestamp);
cvt.quaternion
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
        leftback,front,theTimestamp);
cvt.quaternion
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
        leftback,front,theTimestamp);
cvt.quaternion
assertElementsAlmostEqual(cvt.H_psi_v_psi_0,H);
assertElementsAlmostEqual([1 -1 0 0; 0 0 0 -1; 0 0 1 0; 1 1 1 1],cvt.points_psi_v);
[errorQuat,errorEuler] = quaternionerror(cvt.quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 0 -theta])
[errorQuat,errorEuler] = quaternionerror([1 0 0 0],cvt.quaternion);
assertElementsAlmostEqual(errorEuler,[0 0 theta])

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

function test_quaternionerror
rightback = [1 0 0];
leftback = [-1 0 0];
front = [0 1 0];
theTimestamp = 2.3;
cvt = ConvertViconThreeMarkers(rightback,...
        leftback,front,theTimestamp);
assertEqual(cvt.points_psi_0,[1 -1 0 0; 0 0 1 0; 0 0 0 1; 1 1 1 1]);

assertEqual(cvt.H_psi_v_psi_0,eye(4));
assertEqual(cvt.points_psi_0,cvt.points_psi_v);
q2 = matrix2quaternion(eye(4));
[errorQuat,errorEuler] = quaternionerror(cvt.quaternion,q2);
assertEqual(errorQuat,[1 0 0 0]')
assertEqual(errorEuler,[0 0 0])

theta = -2*pi;
H = [ 1     0           0        0
      0  cos(theta) -sin(theta)  0
      0  sin(theta)  cos(theta)  0
      0     0           0        1];
 quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 0 0])


theta = -0.2*pi;
H = [ 1     0           0        0
      0  cos(theta) -sin(theta)  0
      0  sin(theta)  cos(theta)  0
      0     0           0        1];
 quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 0 theta])

theta = -1.2*pi;
H = [ 1     0           0        0
      0  cos(theta) -sin(theta)  0
      0  sin(theta)  cos(theta)  0
      0     0           0        1];
 quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 0 0.8*pi])

