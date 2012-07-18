function test_suite = test_viconthreemarkers
% MUST BE IN THE DIRECTORY WHERE THE TEST RUNS.
initTestSuite;

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
assertEqual(cvt.getQ,[1 0 0 0]);
assertEqual(cvt.getTimestamp(),2.3);

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
assertElementsAlmostEqual(errorEuler,[-theta 0 0 ])
[errorQuat,errorEuler] = quaternionerror([1 0 0 0],cvt.getQ);
assertElementsAlmostEqual(errorEuler,[theta 0 0 ])

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
assertEqual(size(vtm_t1),[1 501]);
assertEqual(vtm_t1{1}.getQ-vtm_t1{1}.getQ,[0 0 0 0]);
assertElementsAlmostEqual(vtm_t{1}.getQ-vtm_t1{1}.getQ,...
    [-0.011371862567676   0.126304881197798  ...
    -0.126304881197798   0.011371862567673]);
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