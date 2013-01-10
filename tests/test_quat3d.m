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
[vtm_t] = Quat3D.readDataSHDF(filename,...
    runName,'nodeId',1,...
    'Fs_wanted',10,'Fs_recorded',200);

function test_promove3d_readData_withSync
filename='test-data/test-data.h5';
runName = '/newpromovewithsync';

[vtm_t,sync] = Quat3D.readDataSHDF(filename,runName,'nodeId',1);
assertTrue(any(sync))

