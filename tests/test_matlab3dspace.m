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
cvt_t = {cvt cvt cvt cvt};
tm = ThreeMarkers.getChangeOfGlobalReferenceFrames(cvt_t,cvt_t,1,2);
assertElementsAlmostEqual(tm.getH,eye(4,4));

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
data2 = [0 0 0 1 0 0];
metric1 = [0 1 0 0 0];
metric2 = [0 0 0 1 0 0];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,1,4)
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

[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,1,4)
assertEqual(data2,result2);
assertEqual(result1,[0 1 0 0]);
assertEqual(data2(1:4),result1);

Fs = 120;
data1 = [0 1 0 0 2 0 0 1];
data2 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
metric1 = [0 1 0 0 2 0 0 1];
metric2 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,1,4)
assertEqual(data1,result1);
assertEqual(result2,[0 1 0 0 4 0 0 2 0 1 1 3]);

Fs = 120;
data1 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
data2 = [0 1 0 0 2 0 0 1];
metric1 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
metric2 = [0 1 0 0 2 0 0 1];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,1,4)
assertEqual(data2,result2);
assertEqual(result1,[0 1 0 0 4 0 0 2 0 1 1 3]);

Fs = 1;
data1 = [0 1 0 0 2 0 0 1];
data2 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
metric1 = [0 1 0 0 2 0 0 1];
metric2 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,2,4)
assertEqual(data1,result1);
assertEqual(result2,[0 1 0 0 4 0 0 2 0 1 1 3]);

Fs = 1;
data1 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
data2 = [0 1 0 0 2 0 0 1];
metric1 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
metric2 = [0 1 0 0 2 0 0 1];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,2,4)
assertEqual(data2,...
    result2);
assertEqual(result1,[0  1 0 0 4 0 0 2 0 1 1 3]);

Fs = 1;
data1 = [0 0 1 0 0 8 0 0 1 0 0 2 0 0 1 0 1 1 3];
data2 = [0 1 0 0 2 0 0 1];
metric1 = [0 0 1 0 0 8 0 0 1 0 0 2 0 0 1 0 1 1 3];
metric2 = [0 1 0 0 2 0 0 1];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,2,4)
assertEqual(data2,...
    result2);
assertEqual(result1,[0 1 0 0 8 0 0 1 0 0 2 0 0 1 0 1 1 3]);
%close all;

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


function test_slerp
display('Testing slerp:')
theta = -0.2*pi;
H = [ 1     0           0        0
      0  cos(theta) -sin(theta)  0
      0  sin(theta)  cos(theta)  0
      0     0           0        1];
quaternion = matrix2quaternion(H)'
quaternionNorm = quaternionnormalise(quaternion);
assertElementsAlmostEqual(quaternion,quaternionNorm)

startQ=ThreeMarkers([1 0 0 0 0]);
endQ=ThreeMarkers([quaternion 1.0]);


[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0])
assertElementsAlmostEqual(errorEuler,[0 0 theta])


midQ = [slerp(startQ.getQ, endQ.getQ, 0.5, eps) 0.5]

midQ = ThreeMarkers(midQ);
ThreeMarkers.plotRun({startQ;...
    midQ;...
    endQ});

t = 0.0:0.001:1.0;
tm_t = cell(1,size(t,2));
parfor i = 1:size(t,2)
    tm_t{i} = ThreeMarkers([slerp(startQ.getQ,...
            endQ.getQ, t(i), eps) t(i)]);
end
qs = slerp(startQ.getQ,...
            endQ.getQ, t, eps)
ThreeMarkers.plotRun(tm_t);

function test_threeMarkerMinusComparisonMultiply
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

quat = [1,0,0,0,0];
qvt = QuaternionsThreeMarkers(quat(1,1:5));
quat = [0.8,0.2,0,0,0];
qvt1 = QuaternionsThreeMarkers(quat(1,1:5));
quat = [1.0,0,0,0,0];
qvt2 = QuaternionsThreeMarkers(quat(1,1:5));

arrayQs = {qvt qvt1 qvt2};
arrayBs = {qvt; qvt1; qvt2};
display('Array Diff: ');
display(size(arrayQs));
diffQs = ThreeMarkers.cellminus(arrayQs,arrayBs);
assertTrue(iscell(diffQs))
assertEqual(size(diffQs),[1,3]);
for i = 1:3
    obj = diffQs{i};
    display(obj)
    assertEqual(obj.getQ,[1,0,0,0]);
end

function test_threeMarkerQuaternion2euler()
quat = [1,0,0,0,0];
euler = ThreeMarkers.quaternion2euler(quat,false)
assertEqual(euler,[0 0 0]);
eulerDeg = ThreeMarkers.quaternion2euler(quat,true)
assertEqual(euler,[0 0 0]);
quat = [ cos(30/180*pi) sin(30/180*pi) 0 0];
format long;
euler = ThreeMarkers.quaternion2euler(quat,false)
assertElementsAlmostEqual(euler,[0 0  1.047197551196598]);
eulerDeg = ThreeMarkers.quaternion2euler(quat,true)
assertEqual(eulerDeg,[0 0 60]);

tm = ThreeMarkers([ cos(30/180*pi) sin(30/180*pi) 0 0]);
euler = tm.getRPY(true);
assertEqual(eulerDeg,[0 0 60]);

function teest_viconthreemarkers_readData
filename='test-data/test-data.h5';
runName = '/vicon';
[vtm_t] = ViconThreeMarkers.readData(filename,runName,'RBO','LBO','FON');
vtm_t(1).plotT()
assertEqual(size(vtm_t),[1 5136]);
%ThreeMarkers.plotRun(vtm_t);

function quatReadNonExistantNode
filename='test-data/test-data.h5';
runName = '/promove2';
[vtm_t] = QuaternionsThreeMarkers.readData(filename,runName,1,10,200);

function test_promovethreemarkers_readData
filename='test-data/test-data.h5';
assertExceptionThrown(@quatReadNonExistantNode,...
    'QuaternionsThreeMarkers:readData')

runName = '/promove';
[vtm_t] = QuaternionsThreeMarkers.readData(filename,runName,1,10,200);
vtm_t{1}.plotT()
assertEqual(size(vtm_t),[1 724]);

[metrics] = ThreeMarkers.calculateSyncMetrics(vtm_t);
%More tests need to make sure it makes sense.
assertEqual(size(metrics),size(vtm_t));

%ThreeMarkers.plotRun(vtm_t(1:3));
%figure
size(vtm_t)
display('Testing change of Global Frame');
tm_est = ThreeMarkers.getChangeOfGlobalReferenceFrames(vtm_t,...
   vtm_t,1,3)
display(['Returned value: ' class(tm_est)]);
assertTrue(isa(tm_est,'ThreeMarkers'));
assertElementsAlmostEqual(tm_est.getH,eye(4));
assertElementsAlmostEqual(tm_est.getQ,[1 0 0 0]);
vmt_t2 = tm_est*vtm_t(1:3);
ThreeMarkers.plotRun([vtm_t(1:3);vtm_t(1:3)]);
vtm_t2 = tm_est*vtm_t;
try
 vtm_t*[tm_est tm_est];
catch exception
    assertEqual(exception.identifier,'matlab3Dspace:mtimes');
end
ThreeMarkers.plotRun([vtm_t(1:3);vtm_t2(1:3)]);

figure;
[roll,pitch,yaw,diff_t]=ThreeMarkers.getDiff(...
    vtm_t(1:3),vtm_t(2:4),true);
assertTrue(max(roll)>0);
assertTrue(max(pitch)>0);
assertTrue(max(yaw)>0);
[roll,pitch,yaw,diff_t]=ThreeMarkers.plotDiff(...
    vtm_t(1:3),vtm_t(1:3),true,120);
class(diff_t)
euler=diff_t{1}.getRPY(true)
assertEqual([0 0 0],euler)
assertElementsAlmostEqual(max(roll),0);
assertElementsAlmostEqual(max(pitch),0);
assertElementsAlmostEqual(max(yaw),0);
close all

