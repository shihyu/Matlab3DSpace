function test_suite = test_3d
% MUST BE IN THE DIRECTORY WHERE THE TEST RUNS.
initTestSuite;

function test_normWithOffset
if ([0 0 0]-[0 0 0] <= repmat(eps, size([0 0 0])))
    display('Good Test')
end
assertEqual(zeros(1,3),ThreeD.normWithOffset([0 0 0],[0 0 0]));
assertEqual(ones(1,3),ThreeD.normWithOffset([1 1 1],[1 1 1]));
assertEqual(ones(1,3)/sqrt(3)+ones(1,3),...
    ThreeD.normWithOffset([2 2 2],[1 1 1]));

thePoint = [1 1 1];
assertEqual(thePoint/norm(thePoint),ThreeD.normWithOffset(thePoint,[0 0 0]));
theReference = thePoint;
thePoint = [0 0 0];
REF=~all(theReference < repmat(eps, size(theReference)))
assertEqual((thePoint-theReference)/norm(thePoint-theReference)+...
                    theReference,...
                    ThreeD.normWithOffset(thePoint,theReference));
                
function test_resample
  rightback = [0 1  0];
leftback = [0 -1  0];
front = [1 0  0];
theTimestamp = 2.4;
cvt = Markers3D(rightback,...
        leftback,front,theTimestamp);
    rightback = [0 1  0];
leftback = [0 -1  0];
front = [1 0  0];
theTimestamp = 2.5;
theQuat = cvt.getQ;
cvt2 = Markers3D(rightback,...
        leftback,front,theTimestamp);
tm_t = {cvt,cvt2,cvt2};
[tm_t,t] = ThreeD.resample(tm_t,[2.43,2.45]);
t
assertEqual(size(tm_t),[1 2]);
for i = 1:2
    i
    tm_t{i}.getQ
    if i == 1
        assertEqual(tm_t{i}.getTimestamp,2.43);
    else
        assertEqual(tm_t{i}.getTimestamp,2.45);
    end
    normewd = quaternionnormalise(tm_t{i}.getQ)
    assertElementsAlmostEqual(tm_t{i}.getQ,theQuat);
end


function test_calculateEst
rightback = [0 1  0];
leftback = [0 -1  0];
front = [1 0  0];
theTimestamp = 2.4;
cvt = Markers3D(rightback,...
        leftback,front,theTimestamp);
assertEqual(cvt.getTimestamp(),2.4);
assertEqual(cvt.get0,[0 0 1 0; -1 1 0  0; 0 0 0 1; 1 1 1 1]);
cvt1 = cvt;
cvt1 = cvt1.setTimestamp(2.5);
cvt2 = cvt;
cvt2 = cvt2.setTimestamp(2.6);
cvt3 = cvt;
cvt3 = cvt3.setTimestamp(2.7);
cvt_t = {cvt cvt1 cvt2 cvt3};
tm = ThreeD.getChangeOfGlobalReferenceFrames(cvt_t,cvt_t,1,2);
assertElementsAlmostEqual(tm.getH,eye(4,4));
tm.getTimestamp
assertEqual(cvt_t{1}.getTimestamp(),2.4);
assertEqual(cvt_t{2}.getTimestamp(),2.5);
assertEqual(cvt_t{3}.getTimestamp(),2.6);
assertEqual(cvt_t{4}.getTimestamp(),2.7);
cvt_t =  tm*cvt_t;
assertEqual(cvt_t{1}.getTimestamp(),2.4);
assertEqual(cvt_t{2}.getTimestamp(),2.5);
assertEqual(cvt_t{3}.getTimestamp(),2.6);
assertEqual(cvt_t{4}.getTimestamp(),2.7);

quat = [1,0,0,0];
qvt =  ThreeD(quat);
quat = [0.4,0.4,0.4,0.4];
qvt1 =  ThreeD(quat);
tm = {qvt qvt1};

zero_t = {qvt qvt};
tm_est = ThreeD.getChangeOfGlobalReferenceFrames(zero_t,tm,1,1);
assertEqual(tm_est.getQ, [1 0 0 0]);
tm_est = ThreeD.getChangeOfGlobalReferenceFrames(zero_t,tm,2,1);
tm_expected = qvt1';
assertElementsAlmostEqual(tm_est.getQ, tm_expected.getQ);
tm = tm_expected*tm;
assertElementsAlmostEqual(tm{2}.getQ, [1 0 0 0]);
assertElementsAlmostEqual(tm{1}.getQ, ...
    quaternionproduct([0.4,-0.4,-0.4,-0.4],[1 0 0 0])');


function test_calculateEst_Advanced
quat = [1,0,0,0];
qvt =  ThreeD(quat);
zero_t = {qvt qvt};
testQ =  [0.684259741117616  -0.003722627105096   0.011703270486976  -0.729134954718945];
qvt =  ThreeD(testQ);
tm = {qvt,qvt};
tm_est = ThreeD.getChangeOfGlobalReferenceFrames(zero_t,tm,1,2);
tm_expected_inverse = qvt';
close all;
hold on;
grid on;
tm_est.plotT
tm_expected_inverse.plotT
assertElementsAlmostEqual(tm_est.getQ, -tm_expected_inverse.getQ);
tm_expected = tm_est.*qvt;
assertElementsAlmostEqual(tm_expected.getQ,[-1 0 0 0]);


function test_quats

q1 = [1 -0.5 -0.5 -0.5];
qvt1 = ThreeD(q1);
q2 = [-1 0.5 0.5 0.5];
qvt2 = ThreeD(q2);
figure
hold on;
qvt1.plotT;
qvt2.plotT;
qvt1.getQ
qvt2.getQ
qvt3 = qvt2'
qvt3.getQ

function test_angleDiff
x = 0;
y = pi/2;
assertEqual(ThreeD.angleDifference(x,y),-pi/2);
assertEqual(ThreeD.angleDifference(y,x),pi/2);
x=pi; 
y=pi-1;
assertEqual(ThreeD.angleDifference(x,y),1);
assertEqual(ThreeD.angleDifference(y,x),-1);
x=-pi/2; 
y=pi/2-1;
assertEqual(ThreeD.angleDifference(x,y),-(pi-1));
assertEqual(ThreeD.angleDifference(y,x),(pi-1));

x = pi-0.1;
y = pi;
assertElementsAlmostEqual(ThreeD.angleDifference(x,y),-0.1);
assertElementsAlmostEqual(ThreeD.angleDifference(y,x),0.1);
x = pi-0.1;
y = pi+0.1;
assertElementsAlmostEqual(ThreeD.angleDifference(x,y),-0.2);
assertElementsAlmostEqual(ThreeD.angleDifference(y,x),0.2);
x = pi;
y = pi+0.1;
assertElementsAlmostEqual(ThreeD.angleDifference(x,y),-0.1);
assertElementsAlmostEqual(ThreeD.angleDifference(y,x),0.1);
x = 2*pi-0.1;
y = 2*pi;
assertElementsAlmostEqual(ThreeD.angleDifference(x,y),-0.1);
assertElementsAlmostEqual(ThreeD.angleDifference(y,x),0.1);
x = 2*pi-0.1;
y = 2*pi+0.1;
assertElementsAlmostEqual(ThreeD.angleDifference(x,y),-0.2);
assertElementsAlmostEqual(ThreeD.angleDifference(y,x),0.2);
x = 2*pi;
y = 2*pi+0.1;
assertElementsAlmostEqual(ThreeD.angleDifference(x,y),-0.1);
assertElementsAlmostEqual(ThreeD.angleDifference(y,x),0.1);
x = 2*pi;
y = 0;
assertElementsAlmostEqual(ThreeD.angleDifference(x,y),0);
assertElementsAlmostEqual(ThreeD.angleDifference(y,x),0);

function test_threeMarkerMinusComparisonMultiplyTranspose
quat = [1,0,0,0,1.1];
qvt = Quat3D(quat(1,1:5));
quat = [0.8,0.2,0,0,2];
qvt1 = Quat3D(quat(1,1:5));
quat = [1.0,0,0,0,3];
qvt2 = Quat3D(quat(1,1:5));
assertEqual(qvt,qvt)
assertEqual(qvt.getTimestamp(),1.1);
assertEqual(qvt1.getTimestamp(),2);

diff = qvt-qvt1;
assertEqual(diff.getTimestamp(),1.1);

diff = diff.setTimestamp(1.5);
assertEqual(diff.getTimestamp(),1.5);

qvtConj = qvt1';
assertEqual(qvtConj.getTimestamp,qvt1.getTimestamp)
assertEqual(qvtConj.getTimestamp,2)
qvtConjFun = quaternionconjugate(qvt1.getQ);
assertElementsAlmostEqual(qvtConj.getQ,qvtConjFun);

qvtConjFun = ThreeD(qvtConjFun);
qvtConjFun = qvtConjFun.setTimestamp(9.0)
qvt1Cal = qvt.*qvtConjFun;
assertEqual(qvt1Cal.getTimestamp,9.0);

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
qvt =  ThreeD(quat);
quat = [0.8,0.2,0,0];
qvt1 = ThreeD(quat);
quat = [1.0,0,0,0];
qvt2 = ThreeD(quat);

arrayQs = {qvt qvt1 qvt2};
arrayBs = {qvt qvt1 qvt2};
display('Array Diff: ');
display(size(arrayQs));
diffQs = ThreeD.cellminus(arrayQs,arrayBs);
assertTrue(iscell(diffQs))
assertEqual(size(diffQs),[1,3]);
for i = 1:3
    obj = diffQs{i};
    display(obj)
    assertElementsAlmostEqual(obj.getQ,[1,0,0,0]);
    assertEqual(arrayQs{i}.getTimestamp,diffQs{i}.getTimestamp);
end

function test_promove3d_readData
filename='test-data/test-data.h5';
runName = '/promove';
[vtm_t] = Quat3D.readDataPromove(filename,runName,1,10,200);
vtm_t{1}.plotT()
assertEqual(size(vtm_t),[1 724]);
assertElementsAlmostEqual(vtm_t{1}.getTimestamp, 1.743399479347263e+05);

%ThreeD.plotRun(vtm_t(1:3));
%figure
size(vtm_t)
display('Testing change of Global Frame');
tm_est = ThreeD.getChangeOfGlobalReferenceFrames(vtm_t,...
   vtm_t,1,3)

assertElementsAlmostEqual(vtm_t{1}.getTimestamp, 1.743399479347263e+05);

display(['Returned value: ' class(tm_est)]);
assertTrue(isa(tm_est,'ThreeD'));
assertElementsAlmostEqual(tm_est.getH,eye(4));
assertElementsAlmostEqual(tm_est.getQ,[1 0 0 0]);
vmt_t2 = tm_est*vtm_t(1:3);
ThreeD.plotRun([vtm_t(1:3);vtm_t(1:3)]);
vtm_t2 = tm_est*vtm_t;
try
 vtm_t*[tm_est tm_est];
catch exception
    assertEqual(exception.identifier,'matlab3Dspace:mtimes');
end
ThreeD.plotRun([vtm_t(1:3);vtm_t2(1:3)]);

figure;
[roll,pitch,yaw,diff_t]=ThreeD.getDiff(...
    vtm_t(1:3),vtm_t(2:4),true);
assertElementsAlmostEqual(diff_t{1}.getTimestamp, 1.743399479347263e+05);
assertTrue(max(roll)>0);
assertTrue(max(pitch)>0);
assertTrue(max(yaw)>0);
assertElementsAlmostEqual(vtm_t{1}.getTimestamp, 1.743399479347263e+05);
diff_t = ThreeD.cellminus(vtm_t(1:3),vtm_t(1:3));
[roll,pitch,yaw,t]=ThreeD.getRPYt(...
    diff_t,true);
ThreeD.plotRPY(roll,pitch,yaw,t,true,'normal','--b');
ThreeD.plotRPY(roll,pitch,yaw,t,true,'stem','--b');
assertEqual(t(1),diff_t{1}.getTimestamp);
assertElementsAlmostEqual(diff_t{1}.getTimestamp, 1.743399479347263e+05);

class(diff_t)
euler=diff_t{1}.getRPY(true)
assertElementsAlmostEqual([0 0 0],euler)
assertElementsAlmostEqual(max(roll),0);
assertElementsAlmostEqual(max(pitch),0);
assertElementsAlmostEqual(max(yaw),0);
close all

diff_t = ThreeD.cellminus(vtm_t(1:2),vtm_t(1:3));
assertEqual(size(diff_t),[1,2]);
diff_t = ThreeD.cellminus(vtm_t(1:3),vtm_t(1:2));
assertEqual(size(diff_t),[1,2]);

function test_getRPHt
filename='test-data/test-data.h5';
runName = '/promove';
[vtm_t] = Quat3D.readDataPromove(filename,runName,1,...
    10,200);
[roll,pitch,yaw,t] = ThreeD.getRPYt(vtm_t,true);
assertTrue(yaw(1)>0);
assertTrue(pitch(1)~=0);
assertTrue(roll(1)~=0);
assertEqual(size(roll),size(pitch),size(yaw));
figure
ThreeD.plotRPY(roll,pitch,yaw,t,true,'normal','--b');


function test_callibrate
quat = [1,0,0,0];
qvt =  ThreeD(quat);
qvt = qvt.setTimestamp(7.8);
quat = [0.8,0.2,0,0];
qvt1 = ThreeD(quat);
qvt1 = qvt1.setTimestamp(8.2);
quat = [0.8,0.6,0,0];
qvt2 = ThreeD(quat);
qvt2 = qvt2.setTimestamp(9.5);
tm_t = {qvt,qvt1,qvt2};
tm_t_c = ThreeD.callibrate(tm_t,1,1);
assertElementsAlmostEqual(tm_t_c{1}.getQ,[1 0 0 0])
assertElementsAlmostEqual(tm_t_c{2}.getQ,[0.8 0.2 0 0]/norm([0.8 0.2 0 0]))
assertElementsAlmostEqual(tm_t_c{3}.getQ,[0.8 0.6 0 0]/norm([0.8 0.6 0 0]))
assertEqual(tm_t_c{1}.getTimestamp,0.0)
assertEqual(tm_t_c{2}.getTimestamp,8.2-7.8)
assertEqual(tm_t_c{3}.getTimestamp,9.5-7.8)
tm_t_c = ThreeD.callibrate(tm_t,2,1);
assertEqual(length(tm_t_c),2);
qvt1_2=[0.8 0.2 0 0]/norm([0.8 0.2 0 0])
assertElementsAlmostEqual(tm_t_c{1}.getQ,[1 0 0 0]);

assertEqual(tm_t_c{1}.getTimestamp,8.2-8.2)
assertEqual(tm_t_c{2}.getTimestamp,9.5-8.2)

tm_t_c = ThreeD.callibrate(tm_t,2,2);
assertEqual(length(tm_t_c),2);
assertEqual(tm_t_c{1}.getTimestamp,8.2-8.2)
assertEqual(tm_t_c{2}.getTimestamp,9.5-8.2)

function test_getandplotrpyt
quat = [0.8,0.2,0,0];
qvt1 = ThreeD(quat);
qvt1 = qvt1.setTimestamp(0.1);

quat = [0.8,0.2,0.2,0];
qvt2 = ThreeD(quat);
qvt2 = qvt2.setTimestamp(0.2);
qvt3 = qvt2.setTimestamp(11.3);
qvt4 = qvt2.setTimestamp(11.4);
q1_t= {qvt1,qvt2};
q2_t= {qvt3,qvt4};
[roll,pitch,yaw,t,theFigure] = ThreeD.getAndPlotRPYt(q1_t,...
            'TEST',false,'timeseries','--b')
[roll,pitch,yaw,t,theFigure] = ThreeD.getAndPlotRPYt(q2_t,...
            'TEST AGAIN',theFigure,'timeseries','--b')
        
function test_sortAccordingToTimestamp(t,tm_t)
q = [3 2 4 1];
t = [0.3 0.2 0.4 0.1];
[t,q] = ThreeD.sortAccordingToTimestamp(t,q);
assertEqual([1 2 3 4],q);
assertEqual([0.1 0.2 0.3 0.4],t);
t = [0.3 0.2 0.4 0.1];
q = [3 2 4 1];
[t,q] = ThreeD.sortAccordingToTimestamp(t',q');
assertEqual([1 2 3 4]',q);
assertEqual([0.1 0.2 0.3 0.4]',t);
q = [3 2 4 1; 1 2 3 4]'
t = [0.3 0.2 0.4 0.1];
[t,q] = ThreeD.sortAccordingToTimestamp(t',q);
assertEqual([1 2 3 4; 4 2 1 3]',q);
assertEqual([0.1 0.2 0.3 0.4]',t);




