function test_suite = test_threemarkers
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

function test_threeMarkerMinusComparisonMultiplyTranspose
quat = [1,0,0,0,0];
qvt = QuaternionsThreeMarkers(quat(1,1:5));
quat = [0.8,0.2,0,0,0];
qvt1 = QuaternionsThreeMarkers(quat(1,1:5));
quat = [1.0,0,0,0,0];
qvt2 = QuaternionsThreeMarkers(quat(1,1:5));
assertEqual(qvt,qvt)
assertEqual(qvt,qvt2)

diff = qvt-qvt1;
qvtConj = qvt1';
qvtConjFun = quaternionnormalise(quaternionconjugate(qvt1.getQ)');
assertElementsAlmostEqual(qvtConj.getQ,qvtConjFun);
qvt1Cal = qvt.*ThreeMarkers(qvtConjFun);
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

quat = [1,0,0,0];
qvt =  ThreeMarkers(quat);
quat = [0.8,0.2,0,0,0];
qvt1 = ThreeMarkers(quat);
quat = [1.0,0,0,0,0];
qvt2 = ThreeMarkers(quat);

arrayQs = {qvt qvt1 qvt2};
arrayBs = {qvt qvt1 qvt2};
display('Array Diff: ');
display(size(arrayQs));
diffQs = ThreeMarkers.cellminus(arrayQs,arrayBs);
assertTrue(iscell(diffQs))
assertEqual(size(diffQs),[1,3]);
for i = 1:3
    obj = diffQs{i};
    display(obj)
    assertElementsAlmostEqual(obj.getQ,[1,0,0,0]);
end

function test_promovethreemarkers_readData
filename='test-data/test-data.h5';
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
diff_t = ThreeMarkers.cellminus(vtm_t(1:3),vtm_t(1:3));
[roll,pitch,yaw]=ThreeMarkers.getRPYt(...
    diff_t,true,120);
ThreeMarkers.plotRPY(roll,pitch,yaw,true,200);

class(diff_t)
euler=diff_t{1}.getRPY(true)
assertEqual([0 0 0],euler)
assertElementsAlmostEqual(max(roll),0);
assertElementsAlmostEqual(max(pitch),0);
assertElementsAlmostEqual(max(yaw),0);
close all

diff_t = ThreeMarkers.cellminus(vtm_t(1:2),vtm_t(1:3));
assertEqual(size(diff_t),[1,2]);
diff_t = ThreeMarkers.cellminus(vtm_t(1:3),vtm_t(1:2));
assertEqual(size(diff_t),[1,2]);

function test_threemarkersgetRPHt
filename='test-data/test-data.h5';
runName = '/promove';
[vtm_t] = QuaternionsThreeMarkers.readData(filename,runName,1,10,200);
[roll,pitch,yaw] = ThreeMarkers.getRPYt(vtm_t,true);
assertTrue(roll(1)>0);
assertTrue(pitch(1)~=0);
assertTrue(yaw(1)~=0);
assertEqual(size(roll),size(pitch),size(yaw));

function test_callibrate
quat = [1,0,0,0];
qvt =  ThreeMarkers(quat);
quat = [0.8,0.2,0,0,0];
qvt1 = ThreeMarkers(quat);
quat = [0.8,0.6,0,0,0];
qvt2 = ThreeMarkers(quat);
tm_t = {qvt,qvt1,qvt2};
tm_t = ThreeMarkers.callibrate(tm_t,1,3);
assertElementsAlmostEqual(tm_t{1}.getQ,[ 0.957601107188393  -0.288097413233032 0 0])

