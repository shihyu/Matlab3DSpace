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
assertElementsAlmostEqual(errorEuler,[0 0 -theta])
[errorQuat,errorEuler] = quaternionerror([1 0 0 0],cvt.getQ);
assertElementsAlmostEqual(errorEuler,[0 0 theta])

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
