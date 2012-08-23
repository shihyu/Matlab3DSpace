function test_suite = test_quaternion3d
% MUST BE IN THE DIRECTORY WHERE THE TEST RUNS.
initTestSuite;

function test_Quat3D
quat = [1,0,0,0,0];
qvt = Quat3D([quat(1,1:5)]);
assertEqual(qvt.getH,eye(4,4));
assertEqual(qvt.get0,qvt.getT);
assertEqual(qvt.getT,[0 0 1 0; -1 1 0 0; 0 0 0 1; 1 1 1 1]);

function quatReadNonExistantNode
filename='test-data/test-data.h5';
runName = '/promove2';
[vtm_t] = Quat3D.readDataPromove(filename,...
    runName,1,10,200);

function test_promove3d_readData
filename='test-data/test-data.h5';
assertExceptionThrown(@quatReadNonExistantNode,...
    'Quat3D:readData')

runName = '/promove';
[vtm_t,sync] = Quat3D.readDataPromove(filename,runName,1,10,200);
assertFalse(any(sync))
vtm_t{1}.plotT()
assertEqual(size(vtm_t),[1 724]);

%ThreeD.plotRun(vtm_t(1:3));
%figure
size(vtm_t)
display('Testing change of Global Frame');
tm_est = ThreeD.getChangeOfGlobalReferenceFrames(vtm_t,...
   vtm_t,1,3)
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
assertTrue(max(roll)>0);
assertTrue(max(pitch)>0);
assertTrue(max(yaw)>0);
diff_t = ThreeD.cellminus(vtm_t(1:3),vtm_t(1:3));
[roll,pitch,yaw,t]=ThreeD.getRPYt(...
    diff_t,true);
ThreeD.plotRPY(roll,pitch,yaw,t,true,'timeseries','--r');
class(diff_t)
euler=diff_t{1}.getRPY(true)
assertEqual([0 0 0],euler)
assertElementsAlmostEqual(max(roll),0);
assertElementsAlmostEqual(max(pitch),0);
assertElementsAlmostEqual(max(yaw),0);
close all

function test_promove3d_readData_withSync
filename='test-data/test-data.h5';
runName = '/newpromovewithsync';

[vtm_t,sync] = Quat3D.readDataPromove(filename,runName,1,200,200);
assertTrue(any(sync))

