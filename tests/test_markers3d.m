function test_suite = test_markers3d
% MUST BE IN THE DIRECTORY WHERE THE TEST RUNS.
initTestSuite;

function test_Markers3D
rightback = [0 -1.766 0];
leftback = [0 1.766 0];
front = [1.766 0 0];
theTimestamp = 2.3;
cvt = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp);
assertEqual(cvt.get0,[...
    0 0 1 0;
    -1 1 0 0; 
    0 0 0 1; 
    1 1 1 1]);
assertElementsAlmostEqual(cvt.getH,eye(4));
assertElementsAlmostEqual(cvt.get0,cvt.getT);
assertElementsAlmostEqual(cvt.getQ,[1 0 0 0]);
assertEqual(cvt.getTimestamp(),2.3);
cvt.getRPY(true)

rightback = [0 1 0];
leftback = [0 -1 0];
front = [1 0 0];
theTimestamp = 2.3;
cvt = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp);
assertElementsAlmostEqual(cvt.getH,...
    [1 0 0 0;
    0 -1 0 0; 
    0 0 -1 0; 
    0 0 0 1]);
assertElementsAlmostEqual(...
    [0 0 1 0; 
        1 -1 0 0; 
        0 0 0 -1; 
        1 1 1 1],cvt.getT);
quat = cvt.getQ

rightback = [1 1 2];
leftback = [1 -1 2];
front = [2 0 2];
theTimestamp = 2.3;
cvt = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp);
cvt.getQ
assertEqual(cvt.get0,[0 0 1 0; 
    -1 1 0 0; 
    0 0 0 1; 
    1 1 1 1]);
assertElementsAlmostEqual(cvt.getH,[1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1]);
%Translate.
rightback = [4.4 5.5 -6.25];
leftback = [2.4 5.5 -6.25];
front = [3.4 5.5 -5.25];
theta = -pi/2;
H = roty(theta);
cvt = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp);
cvt.getQ
%assertElementsAlmostEqual(cvt.getH,H);
assertElementsAlmostEqual([1 -1 0 0; 0 0 0 -1; 0 0 1 0; 1 1 1 1],cvt.getT);

%Rotate on X
rightback = [0 0 1];
leftback = [0 0 -1];
front = [1 0 0];
theta = -pi/2;
H = rotx(theta);
cvt = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp);
cvt.getQ
cvt.getT
ThreeD.get0
expected = H*ThreeD.get0
assertElementsAlmostEqual(expected,cvt.getT);
assertElementsAlmostEqual(cvt.getH,H);

rightback = [0 0 -1];
leftback = [0 0 1];
front = [1 0 0];
theta = pi/2;
H = rotx(theta);
cvt = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp);
cvt.getQ
cvt.getT
assertElementsAlmostEqual(cvt.getH,H);

%Rotate on Y
rightback = [0 -1 0];
leftback = [0 1 0];
front = [0 0 1];
theta = -pi/2;
H = roty(theta);
cvt = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp);
assertElementsAlmostEqual(...
    [0 0 0 -1; 
        -1 1 0 0; 
        0 0 1 0; 
        1 1 1 1],cvt.getT);
assertElementsAlmostEqual(cvt.getH,H);

rightback = [0 -1 0];
leftback = [0 1 0];
front = [0 0 -1];
theta = pi/2;
H = roty(theta);
cvt = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp);
assertElementsAlmostEqual(...
    [0 0 0 1; 
        -1 1 0 0; 
        0 0 -1 0; 
        1 1 1 1],cvt.getT);
assertElementsAlmostEqual(cvt.getH,H);

%Rotate on Z
rightback = [-1 0 0];
leftback = [1 0 0];
front = [0 -1 0];
theta = -pi/2;
H = rotz(theta);
cvt = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp);
assertElementsAlmostEqual(...
    [-1 1 0 0; 
        0 0 -1 0; 
        0 0 0 1; 
        1 1 1 1],cvt.getT);
assertElementsAlmostEqual(cvt.getH,H);

rightback = [1 0 0];
leftback = [-1 0 0];
front = [0 1 0];
theta = pi/2;
H = rotz(theta);
cvt = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp);
assertElementsAlmostEqual(...
     [1 -1 0 0; 
        0 0 1 0; 
        0 0 0 1; 
        1 1 1 1],cvt.getT);
assertElementsAlmostEqual(cvt.getH,H);

[errorQuat,errorEuler] = quaternionerror(cvt.getQ,[1 0 0 0]);
[errorQuat,errorEuler] = quaternionerror([1 0 0 0],cvt.getQ);


function test_different_zeropoint
%With different base points.
rightback = [1 0 0];
leftback = [-1 0 0];
front = [0 1 0];
points_0 = [
        1 -1 0 0;
        0 0 1 0;
        0 0 0 1;
        1 1 1 1;
        ];
theTimestamp = 2.3;
 assertTrue(all(size(points_0)==[4 4]))
cvt = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp,'points_0',points_0);
cvtNorm = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp);
assertEqual(cvt.get0,[...
    0 0 1 0;
    -1 1 0 0; 
    0 0 0 1; 
    1 1 1 1]);
assertElementsAlmostEqual(cvt.getH,eye(4));
assertElementsAlmostEqual(cvtNorm.getH,[0 -1 0 0;1 0 0 0; 0 0 1 0;0 0 0 1]);
assertElementsAlmostEqual(cvt.get0,cvt.getT);
assertElementsAlmostEqual(cvt.getQ,[1 0 0 0]);
assertEqual(cvt.getTimestamp(),2.3);




function test_markerdiffs
%Pitch Yaw Difference;
H1=roty(pi/6);
H2=rotz(pi/3.4)*roty(pi/6);
H12 = rotz(pi/3.4);
point0 = ThreeD.get0;
point1 = H1*point0;
point2 = H2*point0;

marker1 = Markers3D('rightBack',point1(1:3,1)',...
    'leftBack',point1(1:3,2)',...
    'front',point1(1:3,3)',...
    'timeStamp',0);
marker2 = Markers3D('rightBack',point2(1:3,1)',...
    'leftBack',point2(1:3,2)',...
    'front',point2(1:3,3)',...
    'timeStamp',0);

assertElementsAlmostEqual(marker1.getRPY(false),[0 pi/6 0 ]);
assertElementsAlmostEqual(marker1.getH,H1);
assertElementsAlmostEqual(marker2.getH,H2);

%Pitch Yaw Difference with noise;
H1=roty(pi/6);
H2=rotz(pi/3.4)*roty(pi/6);
H12 = rotz(pi/3.4);
point0 = ThreeD.get0;
point1 = H1*point0
%1cm error in measurement =  20log10(100) = 40
point1 = awgn(point1,50,0)
point2 = H2*point0;

marker1 = Markers3D('rightBack',point1(1:3,1)',...
    'leftBack',point1(1:3,2)',...
    'front',point1(1:3,3)',...
    'timeStamp',0);
marker1k = Markers3D('rightBack',point1(1:3,1)',...
    'leftBack',point1(1:3,2)',...
    'front',point1(1:3,3)',...
    'timeStamp',0,'absoluteOrientationMethod','kabsch');
marker1s = Markers3D('rightBack',point1(1:3,1)',...
    'leftBack',point1(1:3,2)',...
    'front',point1(1:3,3)',...
    'timeStamp',0,'absoluteOrientationMethod','screw');
marker2 = Markers3D('rightBack',point2(1:3,1)',...
    'leftBack',point2(1:3,2)',...
    'front',point2(1:3,3)',...
    'timeStamp',0);
%0.013 radians = 0.75 degrees
assertElementsAlmostEqual(marker1.getRPY(false),[0 pi/6 0 ],'absolute',0.013);
assertElementsAlmostEqual(marker1k.getRPY(false),[0 pi/6 0 ],'absolute',0.013);
assertElementsAlmostEqual(marker1s.getRPY(false),[0 pi/6 0 ],'absolute',0.013);
assertElementsAlmostEqual(marker1.getH,H1,'absolute',0.015);
assertElementsAlmostEqual(marker1k.getH,H1,'absolute',0.015);
assertElementsAlmostEqual(marker1s.getH,H1,'absolute',0.015);
assertElementsAlmostEqual(marker2.getH,H2);



%From screw theory: H12=H2*(H1)^-1
marker12 = marker2-marker1;
assertElementsAlmostEqual(marker12.getRPY(false),[0 0 pi/3.4],'absolute',0.015);

%Roll Yaw Difference;
H1=rotx(pi/2);
H2=rotz(pi/5.2)*rotx(pi/2);
H12 = rotz(pi/5.2);
point0 = ThreeD.get0;
point1 = H1*point0;
point2 = H2*point0;

marker1 = Markers3D('rightBack',point1(1:3,1)',...    
    'leftBack',point1(1:3,2)',...
    'front',point1(1:3,3)',...
    'timeStamp',0);
marker2 = Markers3D('rightBack',point2(1:3,1)',...
    'leftBack',point2(1:3,2)',...
    'front',point2(1:3,3)',...
    'timeStamp',0);

assertElementsAlmostEqual(marker1.getRPY(false),[pi/2 0 0]);
assertElementsAlmostEqual(marker1.getH,H1);
assertElementsAlmostEqual(marker2.getH,H2);

%From screw theory: H12=H2*(H1)^-1
marker12 = marker2-marker1;
assertElementsAlmostEqual(marker12.getRPY(false),[0 0 pi/5.2]);


%Roll Pitch Yaw Difference;
H1=rotx(pi/2)*roty(pi/9);
H2=rotz(pi/5.2)*rotx(pi/2)*roty(pi/9);
H12 = rotz(pi/5.2);
point0 = ThreeD.get0;
point1 = H1*point0;
point2 = H2*point0;

marker1 = Markers3D('rightBack',point1(1:3,1)',...
    'leftBack',point1(1:3,2)',...
    'front',point1(1:3,3)',...
    'timeStamp',0);
marker2 = Markers3D('rightBack',point2(1:3,1)',...
    'leftBack',point2(1:3,2)',...
    'front',point2(1:3,3)',...
    'timeStamp',0);

assertElementsAlmostEqual(marker1.getH,H1);
assertElementsAlmostEqual(marker2.getH,H2);

%From screw theory: H12=H2*(H1)^-1
marker12 = marker2-marker1;
assertElementsAlmostEqual(marker12.getRPY(false),[0 0 pi/5.2]);

%Roll Pitch Yaw Difference with big offset and noise
H1=rotx(pi/2)*roty(pi/9);
H2=rotz(pi/5.2)*rotx(pi/2)*roty(pi/9);
H12 = rotz(pi/5.2);
point0 = ThreeD.get0;
point1 = H1*point0;
point2 = H2*point0;
point1 = awgn(point1,50,0)+1009
point2 = awgn(point2,50,0)+1019.45

marker1 = Markers3D('rightBack',point1(1:3,1)',...
    'leftBack',point1(1:3,2)',...
    'front',point1(1:3,3)',...
    'timeStamp',0);
marker2 = Markers3D('rightBack',point2(1:3,1)',...
    'leftBack',point2(1:3,2)',...
    'front',point2(1:3,3)',...
    'timeStamp',0);
marker1 = Markers3D('rightBack',point1(1:3,1)',...
    'leftBack',point1(1:3,2)',...
    'front',point1(1:3,3)',...
    'timeStamp',0);
marker1k = Markers3D('rightBack',point1(1:3,1)',...
    'leftBack',point1(1:3,2)',...
    'front',point1(1:3,3)',...
    'timeStamp',0,'absoluteOrientationMethod','kabsch');
marker1s = Markers3D('rightBack',point1(1:3,1)',...
    'leftBack',point1(1:3,2)',...
    'front',point1(1:3,3)',...
    'timeStamp',0,'absoluteOrientationMethod','screw');
marker2 = Markers3D('rightBack',point2(1:3,1)',...
    'leftBack',point2(1:3,2)',...
    'front',point2(1:3,3)',...
    'timeStamp',0);
%0.013 radians = 0.75 degrees
assertElementsAlmostEqual(marker1.getH,H1,'absolute',0.015);
assertElementsAlmostEqual(marker1k.getH,H1,'absolute',0.015);
assertElementsAlmostEqual(marker1s.getH,H1,'absolute',0.015);
assertElementsAlmostEqual(marker2.getH,H2,'absolute',0.015);

%From screw theory: H12=H2*(H1)^-1
marker12 = marker2-marker1;
assertElementsAlmostEqual(marker12.getRPY(false),[0 0 pi/5.2],'absolute',0.015);

%Pitch Yaw Difference; (45 degrees).
H1=roty(pi/4);
H2=rotz(pi/4)*roty(pi/4);
H12 = rotz(pi/4);
point0 = ThreeD.get0;
point1 = H1*point0;
point2 = H2*point0;

marker1 = Markers3D('rightBack',point1(1:3,1)',...
    'leftBack',point1(1:3,2)',...
    'front',point1(1:3,3)',...
    'timeStamp',0);
marker2 = Markers3D('rightBack',point2(1:3,1)',...
'leftBack',point2(1:3,2)',...
'front',point2(1:3,3)',...
'timeStamp',0);

assertElementsAlmostEqual(marker1.getRPY(false),[0 pi/4 0 ]);
assertElementsAlmostEqual(marker1.getH,H1);
assertElementsAlmostEqual(marker2.getH,H2);

figure
hold on;
marker1.plotT
marker2.plotT
%From screw theory: H12=H2*(H1)^-1
marker12 = marker2-marker1;
assertElementsAlmostEqual(marker12.getRPY(false),[0 0 pi/4]);

%Pitch Yaw Difference; (180 degrees).
H1=roty(pi);
H2=rotz(pi)*roty(pi);
H12 = rotz(pi);
point0 = ThreeD.get0;
point1 = H1*point0;
point2 = H2*point0;

marker1 = Markers3D('rightBack',point1(1:3,1)',...
    'leftBack',point1(1:3,2)',... 
    'front',point1(1:3,3)',...
    'timeStamp',0);
marker2 = Markers3D('rightBack',point2(1:3,1)',...
    'leftBack',point2(1:3,2)',...
    'front',point2(1:3,3)',...
    'timeStamp',0);

%assertElementsAlmostEqual(marker1.getRPY(false),[0 pi 0 ]);
assertElementsAlmostEqual(marker1.getH,H1);
assertElementsAlmostEqual(marker2.getH,H2);


function test_zeroinput
rightback = [0 0 0];
leftback = [0 0 0];
front = [0 0 0];
theTimestamp = 2.3;
cvt = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp);

function test_noinput
rightback = [];
leftback = [0 0 0];
front = [0 0 0];
theTimestamp = 2.3;
try
cvt = Markers3D('rightBack',rightback,...
    'leftBack',leftback,...
    'front',front,...
    'timeStamp',theTimestamp);
catch exception
    assertEqual(exception.identifier,'MATLAB:InputParser:ArgumentFailedValidation');
end



function test_vicon_create3DMarkersFromRawData
filename='test-data/test-data.h5';
runName = '/vicon';
points_0 = [
        1 -1 0 0;
        0 0 1 0;
        0 0 0 1];
% [vtm_t] = Markers3D.readDataVicon(filename,...
%     runName,'RBO','LBO','FON',points_0);
[rawData] = RawMarkers.readFromFile(filename,...
    runName,'RBO','LBO','FON');
%Intermediate steps
vtm_t = Markers3D.create3DMarkersFromRawData(rawData);
assertEqual(size(vtm_t),[1 389]);
vtm_t = Markers3D.create3DMarkersFromRawData(rawData,'adams',false);
% vtm_t{100}.plotT;
% figure
% ThreeD.plotRun(vtm_t,-1)
assertEqual(size(vtm_t),[1 389]);
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

function test_vicon3d_readDataAdams
filename='test-data/test-data.h5';
runName = 'testrun';
[rawData] = RawMarkers.readFromFile(filename,...
    runName,'RBO','LBO','FON','adams',true);
vtm_t = Markers3D.create3DMarkersFromRawData(rawData,'adams',true);
vtm_t{1}.plotT()
assertEqual(size(vtm_t),[1 501]);
current = vtm_t{1}.getTimestamp();
for i = 2:length(vtm_t)
    assertTrue(vtm_t{i}.getTimestamp>current);
    assertElementsAlmostEqual(1/(vtm_t{i}.getTimestamp-current),...
        33.333333333333336);
    current = vtm_t{i}.getTimestamp();
end

[rawData] = RawMarkers.readFromFile(filename,...
    runName,'RBT','LBT','FTN','adams',true);
vtm_t1 = Markers3D.create3DMarkersFromRawData(rawData,'adams',true);
vtm_t1{1}.plotT()
%ThreeD.plotRun(vtm_t1);
assertEqual(size(vtm_t1),[1 501]);
assertEqual(vtm_t1{1}.getQ-vtm_t1{1}.getQ,[0 0 0 0]);
%vtm_t1{1}.getRPY(true)
%vtm_t{1}.getRPY(true)
%assertElementsAlmostEqual(vtm_t{1}.getQ-vtm_t1{1}.getQ,...
%    zeros(1,4));
%ThreeD.plotRun(vtm_t);


function test_objectcreation
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
    %         pi/2-ThreeD.getAngle(testRBO,testFON,mid),0);
    %     assertElementsAlmostEqual(...
    %         pi/2,ThreeD.getAngle(testLBO,testFON,mid))
    
    pointsR = [(testRBO-mid)',(testLBO-mid)',(testFON-mid)',(testLBO-mid)'];
    v_rbt = Markers3D('rightBack',testRBO,...
    'leftBack',testLBO,...
    'front',testFON,...
    'timeStamp',0);
    v_lbt = Markers3D('rightBack',testRBT,...
    'leftBack',testLBT,...
    'front',testFTN,...
    'timeStamp',0);
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
[rawData] = RawMarkers.readFromFile(filename,...
    runName,'RBT','LBT','FTN','adams',true);
vtm_t = Markers3D.create3DMarkersFromRawData(rawData,'adams',true);
%ThreeD.plotRun(vtm_t,0.4);
for i = 1:length(adamsColumns)
    adamsColumn = adamsColumns{i}
    adamsData = data.(adamsColumn);
    CompareValue = cell(1,1);
    CompareValue{1} = adamsData';
   
    MeasuredValue = cell(1,1);
    %Normal
    [roll,pitch,yaw,t] = ThreeD.getAndPlotRPYt(vtm_t,...
        adamsColumns{i},false,'plotStyle','--o');
    MeasuredValue{1} = chooseData(roll,pitch,yaw,adamsColumn);
    
    [~,~,RMSVector] =...
        rmserrorplot(CompareValue,MeasuredValue,['RMS ERROR: ' runName ': '...
        adamsColumn],true)
    assertTrue(all(abs(RMSVector{1})<1.5e-4))
end
%figure
%ThreeD.plotRun(vtmk_t,0.1);

function test_adams_rollpitchyaw
close all;
filename='test-data/test-data.h5';

runName = 'adams/roll';
adamsColumn = {'Roll'};
processRun(filename,runName,adamsColumn);

%The Pitch is plotted using xzx euler angles..
% runName = 'adams/pitch';
% adamsColumn = {'Pitch'};
% processRun(filename,runName,adamsColumn);

%The Yaw is plotted wrong in Adams. Not sure why.
% runName = 'adams/yaw';
% adamsColumn = {'yaw'};
% processRun(filename,runName,adamsColumn);

%The Roll Pitch and Yaw in the adams file is with respect
%to the global origin and not within the rigid body.
% Thus they do not match up.
% runName = 'adams/rollpitchyaw';
% adamsColumn = {'Roll'}%,'Pitch','Yaw'};
% processRun(filename,runName,adamsColumn);

%Incorrectly created file.
%runName = '/adams/rollpitchyawbig';
%adamsColumn = {'Roll','Pitch','Yaw'};
%processRun(filename,runName,adamsColumn);


function processRunCombined(filename,runName)
display(['==================================================']);
display(['Processing RUN:' runName]);
display(['==================================================']);

reader = adamsReader(filename,runName);
data = reader.readData(false);

steeringAngle = data.SteeringAngle';


[rawData] = RawMarkers.readFromFile(filename,...
    runName,'RBO','LBO','FON','adams',true);
theChanger = Markers3D.create3DMarkersFromRawData(rawData,'adams',true);

[rawData] = RawMarkers.readFromFile(filename,...
    runName,'RBT','LBT','FTN','adams',true);
theStatic = Markers3D.create3DMarkersFromRawData(rawData,'adams',true);


% ThreeD.plotRun(theChanger,0.1);
% ThreeD.plotRun([theStatic;theChanger],0.4);
%  ThreeD.plotRun(theStatic,0.1);
[one_roll,one_pitch,one_yaw,t,theFigure] = ThreeD.getAndPlotRPYt(theChanger,...
    ['CHANGER (B) STATIC (R) DIFF (G) ' runName],false,'plotStyle','-o');
[two_roll,two_pitch,two_yaw,t] = ThreeD.getAndPlotRPYt(theStatic,...
    ['SENSOR TWO ' runName],theFigure,'plotStyle','--r*');
%STILL NOT SURE WHY! should be:
%diff_t = ThreeD.cellminus(theChanger,theStatic);
diff_t = ThreeD.cellInverseMultiply(theChanger,theStatic);
% notSureWhyDiff = ThreeD.cellMultiply(theChanger,theStatic); 
[diff_roll,diff_pitch,diff_yaw,t] = ThreeD.getAndPlotRPYt(diff_t,...
    ['SENSOR DIFFERENCE: ' runName ],theFigure,'plotStyle','--go');
% assertElementsAlmostEqual(diff_roll,zeros(1,...
%     length(diff_roll)));
% assertElementsAlmostEqual(diff_pitch,zeros(1,...
%     length(diff_pitch)));
% assertElementsAlmostEqual(max(abs(diff_yaw)),45,'relative',0.0001);
RMSVector=rmserrorplot({diff_yaw},...
    {steeringAngle},['RMS ERROR: ' runName ...
    ': SteeringAngle'],true);
assertTrue(all(RMSVector<3))

function test_adams_rollpitchyaw_combined
close all;
filename='test-data/test-data.h5';
runName = '/adams/pitchyawcombined';
processRunCombined(filename,runName);
% runName = '/adams/rollyawcombined';
% processRunCombined(filename,runName);
