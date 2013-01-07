%Tutorial
filename='../tests/test-data/test-data.h5';
runs = h5Reader.listRuns(filename);

runName = '/promove';
node1 = 1;
Fs1 = 200;
poolSize = matlabpool('size');
if poolSize == 0
    matlabpool local 4;
end
inDegrees=true;
%Read the quaternion data. This is static function is custom
%for CSV files that Intertia Technology outputs from their
%ProMoveGUI.
tm_t=Quat3D.readDataPromove(filename,runName,...
        node1,Fs1,Fs1);
    
%You can then callibrate the data. It takes the 'average'
%quaternion of the 10 quaternions starting at 100 samples
%into the experiment and then uses this as the zero frame.
%All the samples in the set are then inversely rotated by
%quaternion.
tm_t  = ThreeD.zeroTheRun(tm_t,100,10);

%Get the Roll Pitch and Yaw of the experiment.
[roll_t,pitch_t,yaw_t] = ThreeMarkers.getRPYt(tm_t,inDegrees);
%Plot this!
ThreeMarkers.plotRPY(roll_t,pitch_t,yaw_t,inDegrees,Fs1);