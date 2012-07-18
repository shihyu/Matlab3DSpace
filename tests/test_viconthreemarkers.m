function test_suite = test_viconthreemarkers
% MUST BE IN THE DIRECTORY WHERE THE TEST RUNS.
initTestSuite;

function test_ViconThreeMarkers
rightback = [0 -1 0];
leftback = [0 1 0];
front = [1 0 0];
theTimestamp = 2.3;
cvt = ViconThreeMarkers(rightback,...
    leftback,front,theTimestamp)
assertEqual(cvt.get0,[0 0 1 0; -1 1 0 0; 0 0 0 1; 1 1 1 1]);

assertEqual(cvt.getH,eye(4));
assertEqual(cvt.get0,cvt.getT);
assertEqual(cvt.getQ,[1 0 0 0]);
assertEqual(cvt.getTimestamp(),2.3);

rightback = [0 1 0];
leftback = [0 -1 0];
front = [1 0 0];
theTimestamp = 2.3;
cvt = ViconThreeMarkers(rightback,...
    leftback,front,theTimestamp);
assertElementsAlmostEqual(cvt.getH,[1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1]);
assertElementsAlmostEqual([0 0 1 0; 1 -1 0 0; 0 0 0 -1; 1 1 1 1],cvt.getT);
cvt.getQ

rightback = [1 1 2];
leftback = [1 -1 2];
front = [2 0 2];
theTimestamp = 2.3;
cvt = ViconThreeMarkers(rightback,...
    leftback,front,theTimestamp);
cvt.getQ
assertEqual(cvt.get0,[0 0 1 0; -1 1 0 0; 0 0 0 1; 1 1 1 1]);
assertEqual(cvt.getH,[1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1]);

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
parfor samplePoint = 1:N
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

function processRun(filename,runName,adamsColumn)
display(['==================================================']);
display(['Processing RUN:' runName adamsColumn]);
display(['==================================================']);

[vtm_t] = ViconThreeMarkers.readDataAdams(filename,runName,...
    'RBT','LBT','FTN');
reader = adamsReader(filename,runName);
data = reader.readData(false);
adamsData = data.(adamsColumn);
theTime = data.Time;
figure
title('Adams data');
plot(theTime,adamsData);
figure
title(['RUN PLOT:' runName]);
ThreeMarkers.plotRun(vtm_t);
figure
title(['ROLL PITCH YAW:' runName])
[roll,pitch,yaw,t] = ThreeMarkers.getRPYt(vtm_t,true);
ThreeMarkers.plotRPY(roll,pitch,yaw,true,200,t,2);

function test_viconthreemarkers_adams_rollpitchyaw
close all;
filename='test-data/test-data.h5';
runName = 'adams/roll';
adamsColumn = 'Roll';
processRun(filename,runName,adamsColumn);
runName = 'adams/pitch';
adamsColumn = 'Pitch';
processRun(filename,runName,adamsColumn);
runName = 'adams/yaw';
adamsColumn = 'yaw';
processRun(filename,runName,adamsColumn);
