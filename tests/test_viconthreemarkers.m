function test_suite = test_viconthreemarkers
% MUST BE IN THE DIRECTORY WHERE THE TEST RUNS.
initTestSuite;

function test_ViconThreeMarkers
rightback = [0 -1 0];
leftback = [0 1 0];
front = [1 0 0];
theTimestamp = 2.3;
cvt = ViconThreeMarkers(rightback,...
    leftback,front,theTimestamp);
assertEqual(cvt.get0,[0 0 1 0; -1 1 0 0; 0 0 0 1; 1 1 1 1]);

assertElementsAlmostEqual(cvt.getH,eye(4));
assertElementsAlmostEqual(cvt.get0,cvt.getT);
assertElementsAlmostEqual(cvt.getQ,[1 0 0 0]);
assertEqual(cvt.getTimestamp(),2.3);

rightback = [0 1 0];
leftback = [0 -1 0];
front = [1 0 0];
theTimestamp = 2.3;
cvt = ViconThreeMarkers(rightback,...
    leftback,front,theTimestamp);
assertElementsAlmostEqual(cvt.getH,[1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1]);
assertElementsAlmostEqual([0 0 1 0; 1 -1 0 0; 0 0 0 -1; 1 1 1 1],cvt.getT);
quat = cvt.getQ

rightback = [1 1 2];
leftback = [1 -1 2];
front = [2 0 2];
theTimestamp = 2.3;
cvt = ViconThreeMarkers(rightback,...
    leftback,front,theTimestamp);
cvt.getQ
assertEqual(cvt.get0,[0 0 1 0; -1 1 0 0; 0 0 0 1; 1 1 1 1]);
assertElementsAlmostEqual(cvt.getH,[1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1]);

rightback = [0 0 -1];
leftback = [0 0 1];
front = [1 0 0];
theta = -pi/2;
H = [ 1     0           0        0
    0  cos(theta) sin(theta)  0
    0  -sin(theta)  cos(theta)  0
    0     0           0        1];
cvt = ViconThreeMarkers(rightback,...
    leftback,front,theTimestamp);
cvt.getQ

assertElementsAlmostEqual(cvt.getH,H);
%assertElementsAlmostEqual([1 -1 0 0; 0 0 0 -1; 0 0 1 0; 1 1 1 1],cvt.getT);

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
%assertElementsAlmostEqual(cvt.getH,H);
assertElementsAlmostEqual([1 -1 0 0; 0 0 0 -1; 0 0 1 0; 1 1 1 1],cvt.getT);
[errorQuat,errorEuler] = quaternionerror(cvt.getQ,[1 0 0 0]);
%assertElementsAlmostEqual(errorEuler,[-theta 0 0 ])
[errorQuat,errorEuler] = quaternionerror([1 0 0 0],cvt.getQ);
%assertElementsAlmostEqual(errorEuler,[theta 0 0 ])

function test_viconthreemarkers_readDataVicon
filename='test-data/test-data.h5';
runName = '/vicon';
[vtm_t] = ViconThreeMarkers.readDataVicon(filename,...
    runName,'RBO','LBO','FON');
vtm_t{1}.plotT()
assertEqual(size(vtm_t),[1 5136]);
assertElementsAlmostEqual(vtm_t{1}.getTimestamp, 0.008333333333333);
assertElementsAlmostEqual(vtm_t{2}.getTimestamp, 0.016666666666667);
current = vtm_t{1}.getTimestamp();
for i = 2:length(vtm_t)
    assertTrue(vtm_t{i}.getTimestamp>current);
    assertElementsAlmostEqual(1/(vtm_t{i}.getTimestamp-current),...
        120);
    assertElementsAlmostEqual(vtm_t{i}.getH*vtm_t{i}.getH'-eye(4),zeros(4));
    current = vtm_t{i}.getTimestamp();
end

function test_viconthreemarkers_readDataAdams
filename='test-data/test-data.h5';
runName = 'testrun';
[vtm_t] = ViconThreeMarkers.readDataAdams(filename,runName,'RBO','LBO','FON');
vtm_t{1}.plotT()
assertEqual(size(vtm_t),[1 501]);
current = vtm_t{1}.getTimestamp();
for i = 2:length(vtm_t)
    assertTrue(vtm_t{i}.getTimestamp>current);
    assertElementsAlmostEqual(1/(vtm_t{i}.getTimestamp-current),...
        33.333333333333336);
    current = vtm_t{i}.getTimestamp();
end

[vtm_t1] = ViconThreeMarkers.readDataAdams(filename,runName,'RBT','LBT','FTN');
vtm_t1{1}.plotT()
%ThreeMarkers.plotRun(vtm_t1);
assertEqual(size(vtm_t1),[1 501]);
assertEqual(vtm_t1{1}.getQ-vtm_t1{1}.getQ,[0 0 0 0]);
%vtm_t1{1}.getRPY(true)
%vtm_t{1}.getRPY(true)
%assertElementsAlmostEqual(vtm_t{1}.getQ-vtm_t1{1}.getQ,...
%    zeros(1,4));
%ThreeMarkers.plotRun(vtm_t);


function test_viconthreemarkers_objectcreation
filename='test-data/test-data.h5';
runName = 'testrun';
reader = adamsReader(filename,runName);
data = reader.readData(false);
t=data.Time;
display('Testing construction')
RBO = [data.RBOX data.RBOY data.RBOZ];
LBO = [data.LBOX data.LBOY data.LBOZ];
FON = [data.FONX data.FONY data.FONZ];

RBT = [data.RBTX data.RBTY data.RBTZ];
LBT = [data.LBTX data.LBTY data.LBTZ];
FTN = [data.FTNX data.FTNY data.FTNZ];
N=size(t,1);
for samplePoint = 1:N
    samplePoint;
    testRBO = RBO(samplePoint,:);
    testLBO = LBO(samplePoint,:);
    testFON = FON(samplePoint,:);
    mid = (testRBO+testLBO)/2;
    testRBT = RBT(samplePoint,:);
    testLBT = LBT(samplePoint,:);
    testFTN = FTN(samplePoint,:);
    %     assertElementsAlmostEqual(...
    %         pi/2-ThreeMarkers.getAngle(testRBO,testFON,mid),0);
    %     assertElementsAlmostEqual(...
    %         pi/2,ThreeMarkers.getAngle(testLBO,testFON,mid))
    
    pointsR = [(testRBO-mid)',(testLBO-mid)',(testFON-mid)',(testLBO-mid)'];
    v_rbt = ViconThreeMarkers(testRBO,testLBO,testFON,0);
    v_lbt = ViconThreeMarkers(testRBT,testLBT,testFTN,0);
    H = v_rbt.getH();
    H2 = v_lbt.getH();
    %Make sure the H matrix is invertable.
    assertElementsAlmostEqual(H*H'-eye(4),zeros(4));
    assertElementsAlmostEqual(H2*H2'-eye(4),zeros(4));
    pointsR(4,:) = 1;
    testRBO_0 = H'*pointsR;
    %plotSensor(testRBO_0,'b--');
    
    diffOrig1 = norm(testRBO-testLBO);
    diffOrig2 = norm(testFON-testLBO);
    diffOrig3 = norm(testRBO-testFON);
    
    RBO_0 = testRBO_0(1:3,1);
    LBO_0 = testRBO_0(1:3,2);
    FON_0 = testRBO_0(1:3,3);
    diffOrig1_0 = norm(RBO_0-LBO_0);
    diffOrig2_0 = norm(FON_0-LBO_0);
    diffOrig3_0 = norm(RBO_0-FON_0);
    %Make sure its a rigid body transform.
    assertElementsAlmostEqual(diffOrig1-diffOrig1_0,0);
    assertElementsAlmostEqual(diffOrig2-diffOrig2_0,0);
    assertElementsAlmostEqual(diffOrig3-diffOrig3_0,0);
end;

function [ret] =  chooseData(roll,pitch,yaw,adamsColumn)
if strcmp(adamsColumn,'Roll')==1
    ret = roll;
elseif strcmp(adamsColumn,'Pitch')==1
    ret = -pitch;
elseif strcmp(adamsColumn,'Yaw')==1
    ret = yaw;
elseif strcmp(adamsColumn,'yaw')==1
    ret = yaw;
end


function processRun(filename,runName,adamsColumns)
display(['==================================================']);
display(['Processing RUN:' runName adamsColumns]);
display(['==================================================']);
reader = adamsReader(filename,runName);
data = reader.readData(false);
%Horn
[vtm_t] = ViconThreeMarkers.readDataAdams(filename,runName,...
    'RBT','LBT','FTN');
%Screw
[vtms_t] = ViconThreeMarkers.readDataAdams(filename,runName,...
    'RBT','LBT','FTN','screw');
%kabsch
[vtmk_t] = ViconThreeMarkers.readDataAdams(filename,runName,...
    'RBT','LBT','FTN','kabsch');
for i = 1:length(adamsColumns)
    adamsColumn = adamsColumns{i}
    adamsData = data.(adamsColumn);
    CompareValue = cell(1,1);
    CompareValue{1} = adamsData';
    %CompareValue{2} = adamsData';
   
    MeasuredValue = cell(1,1);
    %Normal
    [roll,pitch,yaw,t] = ThreeMarkers.getAndPlotRPY(vtms_t,adamsColumns{i});
    MeasuredValue{1} = chooseData(roll,pitch,yaw,adamsColumn);
    
    %MeasuredValue{2} = chooseData(yaw,pitch,roll,adamsColumn);
   
    %screw
%     [roll,pitch,yaw,t] = ThreeMarkers.getRPYt(vtmk_t,true);
%     MeasuredValue{2} = chooseData(roll,pitch,yaw,adamsColumn);
%     [roll,pitch,yaw,t] = ThreeMarkers.getRPYt(vtmh_t,true);

%     figure
%     ThreeMarkers.plotRPY(roll,pitch,yaw,true,200,t,0);
%     title(['ROLL PITCH YAW:' runName])
%     MeasuredValue{3} = chooseData(roll,pitch,yaw,adamsColumn);
    rmserrorplot(CompareValue,MeasuredValue,['RMS ERROR: ' runName ': '...
        adamsColumn],true);
end
%figure
%ThreeMarkers.plotRun(vtmk_t,1.0);

function test_adams_rollpitchyaw
close all;
filename='test-data/test-data.h5';

% runName = 'adams/roll';
% adamsColumn = {'Roll'};
% processRun(filename,runName,adamsColumn);

%The Pitch is plotted wrong in Adams.
%runName = 'adams/pitch';
%adamsColumn = {'Pitch'};
%processRun(filename,runName,adamsColumn);

%The Yaw is plotted wrong in Adams.
% runName = 'adams/yaw';
% adamsColumn = {'yaw'};
% processRun(filename,runName,adamsColumn);
%
runName = 'adams/rollpitchyaw';
adamsColumn = {'Roll','Pitch','Yaw'};
processRun(filename,runName,adamsColumn);

%Incorrectly created file.
%runName = '/adams/rollpitchyawbig';
%adamsColumn = {'Roll','Pitch','Yaw'};
%processRun(filename,runName,adamsColumn);


function processRunCombined(filename,runName)
display(['==================================================']);
display(['Processing RUN:' runName]);
display(['==================================================']);
%kobasch
[one_t] = ViconThreeMarkers.readDataAdams(filename,runName,...
    'RB0','LB0','F0N');
[two_t] = ViconThreeMarkers.readDataAdams(filename,runName,...
    'RBT','LBT','FTN');
%ThreeMarkers.plotRun([one_t;two_t]);
[one_roll,one_pitch,one_yaw,t] = ThreeMarkers.getAndPlotRPY(one_t,...
    'ONE');
[two_roll,two_pitch,two_yaw,t] = ThreeMarkers.getAndPlotRPY(two_t,...
    'TWO');
diff_t = ThreeMarkers.cellminus(two_t,one_t);
[diff_roll,diff_pitch,diff_yaw,t] = ThreeMarkers.getAndPlotRPY(diff_t,...
    'DIFF');


function test_adams_rollpitchyaw_combined
close all;
filename='test-data/test-data.h5';
runName = '/adams/pitchyawcombined';
processRunCombined(filename,runName);
runName = '/adams/rollyawcombined';
processRunCombined(filename,runName);
