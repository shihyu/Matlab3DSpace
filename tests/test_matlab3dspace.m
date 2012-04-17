function test_suite = test_matlab3dspace
% MUST BE IN THE DIRECTORY WHERE THE TEST RUNS.
initTestSuite;
function test_calculateEst
rightback = [1 0 0];
leftback = [-1 0 0];
front = [0 1 0];
theTimestamp = 2.3;
cvt = ViconThreeMarkers(rightback,...
        leftback,front,theTimestamp)
assertEqual(cvt.get0,[1 -1 0 0; 0 0 1 0; 0 0 0 1; 1 1 1 1]);
cvt_t = [cvt cvt cvt cvt];
H = calculateChangeofGlobalFrames(cvt_t,cvt_t,1,2);
assertElementsAlmostEqual(H,eye(4,4));

function test_QuaternionsThreeMarkers
quat = [1,0,0,0,0];
qvt = QuaternionsThreeMarkers([quat(1,1:5)]);
assertEqual(qvt.getH,eye(4,4));
assertEqual(qvt.get0,qvt.getT);
assertEqual(qvt.getT,[1 -1 0 0; 0 0 1 0; 0 0 0 1; 1 1 1 1]);


function test_ViconThreeMarkers
rightback = [1 0 0];
leftback = [-1 0 0];
front = [0 1 0];
theTimestamp = 2.3;
cvt = ViconThreeMarkers(rightback,...
        leftback,front,theTimestamp)
assertEqual(cvt.get0,[1 -1 0 0; 0 0 1 0; 0 0 0 1; 1 1 1 1]);

assertEqual(cvt.getH,eye(4));
assertEqual(cvt.get0,cvt.getT);
cvt.getQ


rightback = [-1 0 0];
leftback = [1 0 0];
front = [0 1 0];
theTimestamp = 2.3;
cvt = ViconThreeMarkers(rightback,...
        leftback,front,theTimestamp);
assertElementsAlmostEqual(cvt.getH,[-1 0 0 0; 0 1 0 0; 0 0 -1 0; 0 0 0 1]);
assertElementsAlmostEqual([-1 1 0 0; 0 0 1 0; 0 0 0 -1; 1 1 1 1],cvt.getT);
cvt.getQ

rightback = [1 1 2];
leftback = [-1 1 2];
front = [0 2 2];
theTimestamp = 2.3;
cvt = ViconThreeMarkers(rightback,...
        leftback,front,theTimestamp);
cvt.getQ
assertEqual(cvt.get0,[1 -1 0 0; 0 0 1 0; 0 0 0 1; 1 1 1 1]);

assertEqual(cvt.getH,eye(4));
assertEqual(cvt.get0,cvt.getT);

rightback = [1 0 0];
leftback = [-1 0 0];
front = [0 0 1];
theta = -pi/2;
H = [ 1     0           0        0
      0  cos(theta) sin(theta)  0
      0  -sin(theta)  cos(theta)  0
      0     0           0        1];
cvt = ViconThreeMarkers(rightback,...
        leftback,front,theTimestamp);
cvt.getQ

assertElementsAlmostEqual(cvt.getH,H);
assertElementsAlmostEqual([1 -1 0 0; 0 0 0 -1; 0 0 1 0; 1 1 1 1],cvt.getT);

rightback = [4.4 5.5 -6.25];
leftback = [2.4 5.5 -6.25];
front = [3.4 5.5 -5.25];
theta = -pi/2;
H = [ 1     0           0        0
      0  cos(theta) sin(theta)  0
      0  -sin(theta)  cos(theta)  0
      0     0           0        1];
cvt = ViconThreeMarkers(rightback,...
        leftback,front,theTimestamp);
cvt.getQ
assertElementsAlmostEqual(cvt.getH,H);
assertElementsAlmostEqual([1 -1 0 0; 0 0 0 -1; 0 0 1 0; 1 1 1 1],cvt.getT);
[errorQuat,errorEuler] = quaternionerror(cvt.getQ,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 0 -theta])
[errorQuat,errorEuler] = quaternionerror([1 0 0 0],cvt.getQ);
assertElementsAlmostEqual(errorEuler,[0 0 theta])


function test_angleDiff
x = 0;
y = pi/2;
assertEqual(ThreeMarkers.angleDifference(x,y),-pi/2);
assertEqual(ThreeMarkers.angleDifference(y,x),pi/2);
x=pi; 
y=pi-1;
assertEqual(ThreeMarkers.angleDifference(x,y),1);
assertEqual(ThreeMarkers.angleDifference(y,x),-1);
x=-pi/2; 
y=pi/2-1;
assertEqual(ThreeMarkers.angleDifference(x,y),-(pi-1));
assertEqual(ThreeMarkers.angleDifference(y,x),(pi-1));

x = pi-0.1;
y = pi;
assertElementsAlmostEqual(ThreeMarkers.angleDifference(x,y),-0.1);
assertElementsAlmostEqual(ThreeMarkers.angleDifference(y,x),0.1);
x = pi-0.1;
y = pi+0.1;
assertElementsAlmostEqual(ThreeMarkers.angleDifference(x,y),-0.2);
assertElementsAlmostEqual(ThreeMarkers.angleDifference(y,x),0.2);
x = pi;
y = pi+0.1;
assertElementsAlmostEqual(ThreeMarkers.angleDifference(x,y),-0.1);
assertElementsAlmostEqual(ThreeMarkers.angleDifference(y,x),0.1);
x = 2*pi-0.1;
y = 2*pi;
assertElementsAlmostEqual(ThreeMarkers.angleDifference(x,y),-0.1);
assertElementsAlmostEqual(ThreeMarkers.angleDifference(y,x),0.1);
x = 2*pi-0.1;
y = 2*pi+0.1;
assertElementsAlmostEqual(ThreeMarkers.angleDifference(x,y),-0.2);
assertElementsAlmostEqual(ThreeMarkers.angleDifference(y,x),0.2);
x = 2*pi;
y = 2*pi+0.1;
assertElementsAlmostEqual(ThreeMarkers.angleDifference(x,y),-0.1);
assertElementsAlmostEqual(ThreeMarkers.angleDifference(y,x),0.1);
x = 2*pi;
y = 0;
assertElementsAlmostEqual(ThreeMarkers.angleDifference(x,y),0);
assertElementsAlmostEqual(ThreeMarkers.angleDifference(y,x),0);

function test_synchronise
clear all
close all
Fs = 120;
data1 = [0 1 0 0 0];
data2 = [0 0 1 0 0];
metric1 = [0 1 0 0 0];
metric2 = [0 0 1 0 0];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,1,4)
assertEqual(data1,result1);
assertEqual(result2,[0 1 0 0]);
assertEqual(data1(1:4),result2);
close all;


function test_quaternionerror
rightback = [1 0 0];
leftback = [-1 0 0];
front = [0 1 0];
theTimestamp = 2.3;
cvt = ViconThreeMarkers(rightback,...
        leftback,front,theTimestamp);
assertEqual(cvt.get0,[1 -1 0 0; 0 0 1 0; 0 0 0 1; 1 1 1 1]);

assertEqual(cvt.getH,eye(4));
assertEqual(cvt.get0,cvt.getT);
q2 = matrix2quaternion(eye(4));
[errorQuat,errorEuler] = quaternionerror(cvt.getQ,q2);
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


function test_threeMarkerMinusComparison
quat = [1,0,0,0,0];
qvt = QuaternionsThreeMarkers(quat(1,1:5));
quat = [0.8,0.2,0,0,0];
qvt1 = QuaternionsThreeMarkers(quat(1,1:5));
quat = [1.0,0,0,0,0];
qvt2 = QuaternionsThreeMarkers(quat(1,1:5));
assertEqual(qvt,qvt)
assertEqual(qvt,qvt2)

diff = qvt-qvt1;
qvt1Cal = qvt.*qvt1';
display('Results')
display(qvt1Cal)
display(diff)
assertElementsAlmostEqual(qvt1Cal.getQ,diff.getQ);
qvt1Conj = qvt1'
qvt1Conj.getQ
prod = qvt.*qvt1.*qvt1';
DP=display(qvt)
QVT=display(prod)
assertEqual(prod.getQ,qvt.getQ)


function test_viconthreemarkers_readData
filename='test-data/test-data.h5';
runName = '/vicon';
vtm_t = ViconThreeMarkers.readData(filename,runName,'RBO','LBO','FON');
vtm_t(1).plotT()
assertEqual(size(vtm_t),[1 5136]);
ThreeMarkers.plotRun(vtm_t);