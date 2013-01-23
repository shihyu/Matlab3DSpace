function test_suite = test_3d
% MUST BE IN THE DIRECTORY WHERE THE TEST RUNS.
initTestSuite;
function test_synchronise
clear all
close all
Fs = 120;
data1 = [0 1 0 0 0];
data2 = [0 0 0 1 0 0];
metric1 = [0 1 0 0 0];
metric2 = [0 0 0 1 0 0];
[result1,result2] = ThreeD.synchronise(...
                'metric_1',metric1,...
                'metric_2',metric2,...
                'data_1',data1,...
                'data_2',data2,...
                'Fs',Fs,...
                'theStart',1);
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

[result1,result2] = ThreeD.synchronise(...
                'metric_1',metric1,...
                'metric_2',metric2,...
                'data_1',data1,...
                'data_2',data2,...
                'Fs',Fs,...
                'theStart',1);
assertEqual(data2,result2);
assertEqual(result1,[0 1 0 0]);
assertEqual(data2(1:4),result1);

Fs = 120;
data1 = [0 1 0 0 2 0 0 1];
data2 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
metric1 = [0 1 0 0 2 0 0 1];
metric2 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
[result1,result2] = ThreeD.synchronise(...
                'metric_1',metric1,...
                'metric_2',metric2,...
                'data_1',data1,...
                'data_2',data2,...
                'Fs',Fs,...
                'theStart',1);
assertEqual(data1,result1);
assertEqual(result2,[0 1 0 0 4 0 0 2 0 1 1 3]);

Fs = 120;
data1 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
data2 = [0 1 0 0 2 0 0 1];
metric1 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
metric2 = [0 1 0 0 2 0 0 1];
[result1,result2] = ThreeD.synchronise(...
                'metric_1',metric1,...
                'metric_2',metric2,...
                'data_1',data1,...
                'data_2',data2,...
                'Fs',Fs,...
                'theStart',1);
assertEqual(data2,result2);
assertEqual(result1,[0 1 0 0 4 0 0 2 0 1 1 3]);

Fs = 1;
data1 = [0 1 0 0 2 0 0 1];
data2 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
metric1 = [0 1 0 0 2 0 0 1];
metric2 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
[result1,result2] = ThreeD.synchronise(...
                'metric_1',metric1,...
                'metric_2',metric2,...
                'data_1',data1,...
                'data_2',data2,...
                'Fs',Fs,...
                'theStart',1);
assertEqual(data1,result1);
assertEqual(result2,[0 1 0 0 4 0 0 2 0 1 1 3]);

Fs = 1;
data1 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
data2 = [0 1 0 0 2 0 0 1];
metric1 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
metric2 = [0 1 0 0 2 0 0 1];
[result1,result2] = ThreeD.synchronise(...
                'metric_1',metric1,...
                'metric_2',metric2,...
                'data_1',data1,...
                'data_2',data2,...
                'Fs',Fs,...
                'theStart',1);
assertEqual(data2,...
    result2);
assertEqual(result1,[0  1 0 0 4 0 0 2 0 1 1 3]);

Fs = 1;
data1 = [0 0 0 0 1 0 0 8 0 0 1 0 0 2 0 0 1 0 1 1 3];
data2 = [0 0 1 0 0 2 0 0 1];
metric1 = [0 0 0 0 1 0 0 8 0 0 1 0 0 2 0 0 1 0 1 1 3];
metric2 = [0 0 1 0 0 2 0 0 1];
[result1,result2] = ThreeD.synchronise(...
                'metric_1',metric1,...
                'metric_2',metric2,...
                'data_1',data1,...
                'data_2',data2,...
                'Fs',Fs,...
                'theStart',1);
assertEqual(data2,...
    result2);
assertEqual(result1,[0 0 1 0 0 8 0 0 1 0 0 2 0 0 1 0 1 1 3]);


[result1,result2] = ThreeD.synchronise(...
                'metric_1',metric1,...
                'metric_2',metric2,...
                'data_1',data1,...
                'data_2',data2,...
                'Fs',Fs,...
                'theStart',1,...
                'metricIsAlsoStartOfExperiment',true...
                );
assertEqual([1 0 0 2 0 0 1],...
    result2);
assertEqual(result1,[ 1 0 0 8 0 0 1 0 0 2 0 0 1 0 1 1 3]);

function test_getjointangles
filename='test-data/test-data.h5';
runName='adamslongrun';
[rawData] = RawMarkers.readFromFile(filename,...
    runName,'RBO','LBO','FON','adams',true);
steering_t = ...
    Markers3D.create3DMarkersFromRawData(rawData,'adams',true);
[rawData] = RawMarkers.readFromFile(filename,...
    runName,'RBT','LBT','FTN','adams',true);
roll_t = ...
    Markers3D.create3DMarkersFromRawData(rawData,'adams',true);

reader = adamsReader(filename,runName);
adamsData = reader.readData(false);

steerSensorPlotStyle = '--ob';
rollPlotStyle = '--or';
steerAnglePlotStyle = '--om';
%ThreeD.plotRun([roll_t;steering_t]);
callibrateStart=1;
[steeringAngle_t] = ...
    ThreeD.getjointangles(roll_t,steering_t,'STEERING',callibrateStart);
%And the steering angle sits in the yaw access.
jointAnglePlotStyle = '--m.';
[~,~,steer_yaw] = ...
    ThreeD.getAndPlotRPYt(steeringAngle_t,...
                    'Joint Yaw Angle=>Steer on bike',...
                    false,'plotStyle',jointAnglePlotStyle);
[~,~,ErrorVector] =...
    rmserrorplot({steer_yaw},{adamsData.Steeringangle'},...
        ['RMS ERROR: ' runName ...
    ': SteeringAngle'],true)
assertTrue(all(abs(ErrorVector{1})<0.15));
[roll_roll] = ...
    ThreeD.getAndPlotRPYt(roll_t,...
                    'Roll Sensor Roll Angle',...
                    false,'plotStyle',jointAnglePlotStyle);
[~,~,ErrorVector]=rmserrorplot({roll_roll},{adamsData.rollangle'},['RMS ERROR: ' runName ...
    ': ROLL ANGLE'],true);
assertTrue(all(abs(ErrorVector{1})<1e-2));

function test_synchroniseWithRespectToRPY
filename='./test-data/test-data.h5'
runName = 'newpromovewithsync'
%Resample to make it more difficult
[steering_t] = Quat3D.readDataSHDF(filename,runName,'nodeId',1);
[roll_t] = Quat3D.readDataSHDF(filename,runName,...
    'nodeId',2);
steering_t = ThreeD.changeStartTime(steering_t,0);
roll_t = ThreeD.changeStartTime(roll_t,0);

[roll_r,pitch_r,yaw_r]=ThreeD.getRPYt(roll_t,true);
[roll_s,pitch_s,yaw_s]=ThreeD.getRPYt(steering_t,true);

[roll_t,steering_t] = ThreeD.synchroniseWithRespectToRPY(...
    'roll_one',roll_r,...
    'pitch_one',pitch_r,...
    'yaw_one',yaw_r,...
    'one_t',roll_t,...
    'roll_two',roll_s,...
    'pitch_two',pitch_s,...
    'yaw_two',yaw_s,...
    'two_t',steering_t,...
    'Fs_plot',200);
[roll_r,pitch_r,yaw_r,t]=ThreeD.getRPYt(roll_t,true);
[roll_s,pitch_s,yaw_s,t_s]=ThreeD.getRPYt(steering_t,true);
figure
minSize = min(length(steering_t),length(roll_t));
ThreeD.plotRPY(...
    roll_r(1:minSize),pitch_r(1:minSize),yaw_r(1:minSize),t(1:minSize),...
    'plotStyle','--g');
ThreeD.plotRPY(...
    roll_s(1:minSize),pitch_s(1:minSize),yaw_s(1:minSize),...
    t_s(1:minSize),'plotStyle','--r');

function test_normWithOffset
if ([0 0 0]-[0 0 0] <= repmat(eps, size([0 0 0])))
    display('Good Test')
end
assertEqual(zeros(1,3),ThreeD.normWithOffset([0 0 0],[0 0 0]));
assertEqual(ones(1,3),ThreeD.normWithOffset([1 1 1],[1 1 1]));
assertEqual(ones(1,3)/sqrt(3)+ones(1,3),...
    ThreeD.normWithOffset([2 2 2],[1 1 1]));
assertEqual(ones(1,3)/sqrt(3),...
    ThreeD.normWithOffset([2.1 2.1 2.1],[0 0 0]));
assertElementsAlmostEqual([7.1 77.1 7.1]/sqrt(sum([7.1 77.1 7.1].^2)),...
    ThreeD.normWithOffset([7.1 77.1 7.1],[0 0 0]));
assertElementsAlmostEqual([5.1 -5.1 5.1]/sqrt(sum([5.1 -5.1 5.1].^2)),...
    ThreeD.normWithOffset([5.1 -5.1 5.1],[0 0 0]));
assertElementsAlmostEqual([5.1 0.0 5.1]/sqrt(sum([5.1 0.0 5.1].^2)),...
    ThreeD.normWithOffset([5.1 -0.0 5.1],[0 0 0]));
testIn = [-5.1 0.0 5.1];
assertElementsAlmostEqual(testIn/sqrt(sum(testIn.^2)),...
    ThreeD.normWithOffset(testIn,[0 0 0]));
testIn = [0.0 0.0 0.0];
assertElementsAlmostEqual([0 0 0],...
    ThreeD.normWithOffset(testIn,[0 0 0]));
testIn = [0.0 0.0 0.0];
assertElementsAlmostEqual([5.1 -5.1 5.1]-[1 -1 1]/sqrt(3),...
    ThreeD.normWithOffset(testIn,[5.1 -5.1 5.1]));
testIn = [0.0 0.0 0.0];
assertElementsAlmostEqual([5.1 5.1 5.1]-ones(1,3)/sqrt(3),...
    ThreeD.normWithOffset(testIn,[5.1 5.1 5.1]));
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
cvt = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp);
rightback = [0 1  0];
leftback = [0 -1  0];
front = [1 0  0];
theTimestamp = 2.5;
theQuat = cvt.getQ;
cvt2 = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp);
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
cvt = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp);
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
[vtm_t] = Quat3D.readDataSHDF(filename,runName,'Fs_wanted',10,...
    'Fs_recorded',200,'nodeId',1);
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
ThreeD.plotRPY(roll,pitch,yaw,t,true,'plotType','normal','plotStyle','--b');
ThreeD.plotRPY(roll,pitch,yaw,t,true,'plotType','stem','plotStyle','--b');
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
[vtm_t] = Quat3D.readDataSHDF(filename,runName,'nodeId',1,...
    'Fs_wanted',10,'Fs_recorded',200);
[roll,pitch,yaw,t] = ThreeD.getRPYt(vtm_t,true);
assertTrue(yaw(1)>0);
assertTrue(pitch(1)~=0);
assertTrue(roll(1)~=0);
assertEqual(size(roll),size(pitch),size(yaw));
figure
ThreeD.plotRPY(roll,pitch,yaw,t,true,'plotType',...
    'normal','plotStyle','--b');


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
tm_t_c = ThreeD.zeroTheRun(tm_t,1,1);
assertElementsAlmostEqual(tm_t_c{1}.getQ,[1 0 0 0])
assertElementsAlmostEqual(tm_t_c{2}.getQ,[0.8 0.2 0 0]/norm([0.8 0.2 0 0]))
assertElementsAlmostEqual(tm_t_c{3}.getQ,[0.8 0.6 0 0]/norm([0.8 0.6 0 0]))
assertEqual(tm_t_c{1}.getTimestamp,0.0)
assertEqual(tm_t_c{2}.getTimestamp,8.2-7.8)
assertEqual(tm_t_c{3}.getTimestamp,9.5-7.8)
tm_t_c = ThreeD.zeroTheRun(tm_t,2,1);
assertEqual(length(tm_t_c),2);
qvt1_2=[0.8 0.2 0 0]/norm([0.8 0.2 0 0])
assertElementsAlmostEqual(tm_t_c{1}.getQ,[1 0 0 0]);

assertEqual(tm_t_c{1}.getTimestamp,8.2-8.2)
assertEqual(tm_t_c{2}.getTimestamp,9.5-8.2)

tm_t_c = ThreeD.zeroTheRun(tm_t,2,2);
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
    'TEST',false,'plotStyle','--b')
[roll,pitch,yaw,t,theFigure] = ThreeD.getAndPlotRPYt(q2_t,...
    'TEST AGAIN',theFigure,'plotStyle','--b')

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

function test_setStartTime
startTime = 1.3;
endTime = 2.3;
Fs=10;
t = 0:0.1:4;
tm_t = t.*3;
t_exp = 1.3:0.1:2.3;
tm_t_exp = t_exp.*3;
[t_new,tm_t_new]= ThreeD.setStartTime(t,tm_t, startTime, endTime, Fs);
assertElementsAlmostEqual([t_exp,tm_t_exp],[t_new,tm_t_new]);





