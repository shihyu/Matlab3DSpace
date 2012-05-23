function test_suite = test_quaternionthreemakers
% MUST BE IN THE DIRECTORY WHERE THE TEST RUNS.
initTestSuite;

function test_QuaternionsThreeMarkers
quat = [1,0,0,0,0];
qvt = QuaternionsThreeMarkers([quat(1,1:5)]);
assertEqual(qvt.getH,eye(4,4));
assertEqual(qvt.get0,qvt.getT);
assertEqual(qvt.getT,[1 -1 0 0; 0 0 1 0; 0 0 0 1; 1 1 1 1]);

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

