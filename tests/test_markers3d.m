function test_suite = test_vicon3d
% MUST BE IN THE DIRECTORY WHERE THE TEST RUNS.
initTestSuite;

function test_Markers3D
rightback = [0 -1 0];
leftback = [0 1 0];
front = [1 0 0];
theTimestamp = 2.3;
cvt = Markers3D(rightback,...
    leftback,front,theTimestamp);
assertEqual(cvt.get0,[...
    0 0 1 0;
    -1 1 0 0; 
    0 0 0 1; 
    1 1 1 1]);
assertElementsAlmostEqual(cvt.getH,eye(4));
assertElementsAlmostEqual(cvt.get0,cvt.getT);
assertElementsAlmostEqual(cvt.getQ,[1 0 0 0]);
assertEqual(cvt.getTimestamp(),2.3);

rightback = [0 1 0];
leftback = [0 -1 0];
front = [1 0 0];
theTimestamp = 2.3;
cvt = Markers3D(rightback,...
    leftback,front,theTimestamp);
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
cvt = Markers3D(rightback,...
    leftback,front,theTimestamp);
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
cvt = Markers3D(rightback,...
    leftback,front,theTimestamp);
cvt.getQ
%assertElementsAlmostEqual(cvt.getH,H);
assertElementsAlmostEqual([1 -1 0 0; 0 0 0 -1; 0 0 1 0; 1 1 1 1],cvt.getT);

%Rotate on X
rightback = [0 0 1];
leftback = [0 0 -1];
front = [1 0 0];
theta = -pi/2;
H = rotx(theta);
cvt = Markers3D(rightback,...
    leftback,front,theTimestamp);
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
cvt = Markers3D(rightback,...
    leftback,front,theTimestamp);
cvt.getQ
cvt.getT
assertElementsAlmostEqual(cvt.getH,H);

%Rotate on Y
rightback = [0 -1 0];
leftback = [0 1 0];
front = [0 0 1];
theta = -pi/2;
H = roty(theta);
cvt = Markers3D(rightback,...
    leftback,front,theTimestamp);
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
cvt = Markers3D(rightback,...
    leftback,front,theTimestamp);
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
cvt = Markers3D(rightback,...
    leftback,front,theTimestamp);
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
cvt = Markers3D(rightback,...
    leftback,front,theTimestamp);
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
cvt = Markers3D(rightback,...
    leftback,front,theTimestamp,points_0);
cvtNorm = Markers3D(rightback,...
    leftback,front,theTimestamp);
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

marker1 = Markers3D(point1(1:3,1)',point1(1:3,2)',point1(1:3,3)',0);
marker2 = Markers3D(point2(1:3,1)',point2(1:3,2)',point2(1:3,3)',0);

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

marker1 = Markers3D(point1(1:3,1)',point1(1:3,2)',point1(1:3,3)',0);
marker1k = Markers3D(point1(1:3,1)',point1(1:3,2)',point1(1:3,3)',0,0,'kabsch');
marker1s = Markers3D(point1(1:3,1)',point1(1:3,2)',point1(1:3,3)',0,0,'screw');
marker2 = Markers3D(point2(1:3,1)',point2(1:3,2)',point2(1:3,3)',0);
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

marker1 = Markers3D(point1(1:3,1)',point1(1:3,2)',point1(1:3,3)',0);
marker2 = Markers3D(point2(1:3,1)',point2(1:3,2)',point2(1:3,3)',0);

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

marker1 = Markers3D(point1(1:3,1)',point1(1:3,2)',point1(1:3,3)',0);
marker2 = Markers3D(point2(1:3,1)',point2(1:3,2)',point2(1:3,3)',0);

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

marker1 = Markers3D(point1(1:3,1)',point1(1:3,2)',point1(1:3,3)',0);
marker2 = Markers3D(point2(1:3,1)',point2(1:3,2)',point2(1:3,3)',0);
marker1 = Markers3D(point1(1:3,1)',point1(1:3,2)',point1(1:3,3)',0);
marker1k = Markers3D(point1(1:3,1)',point1(1:3,2)',point1(1:3,3)',0,0,'kabsch');
marker1s = Markers3D(point1(1:3,1)',point1(1:3,2)',point1(1:3,3)',0,0,'screw');
marker2 = Markers3D(point2(1:3,1)',point2(1:3,2)',point2(1:3,3)',0);
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

marker1 = Markers3D(point1(1:3,1)',point1(1:3,2)',point1(1:3,3)',0);
marker2 = Markers3D(point2(1:3,1)',point2(1:3,2)',point2(1:3,3)',0);

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

marker1 = Markers3D(point1(1:3,1)',point1(1:3,2)',point1(1:3,3)',0);
marker2 = Markers3D(point2(1:3,1)',point2(1:3,2)',point2(1:3,3)',0);

%assertElementsAlmostEqual(marker1.getRPY(false),[0 pi 0 ]);
assertElementsAlmostEqual(marker1.getH,H1);
assertElementsAlmostEqual(marker2.getH,H2);


function test_zeroinput
rightback = [0 0 0];
leftback = [0 0 0];
front = [0 0 0];
theTimestamp = 2.3;
cvt = Markers3D(rightback,...
    leftback,front,theTimestamp);

function test_noinput
rightback = [];
leftback = [0 0 0];
front = [0 0 0];
theTimestamp = 2.3;
try
cvt = Markers3D(rightback,...
    leftback,front,theTimestamp);
catch exception
    assertEqual(exception.identifier,'Markers3D:Markers3D');
end



function test_vicon3d_readDataVicon
filename='test-data/test-data.h5';
runName = '/vicon';
points_0 = [
        1 -1 0 0;
        0 0 1 0;
        0 0 0 1];
% [vtm_t] = Markers3D.readDataVicon(filename,...
%     runName,'RBO','LBO','FON',points_0);
[vtm_t] = Markers3D.readDataVicon(filename,...
    runName,'RBO','LBO','FON',points_0);
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
[vtm_t] = Markers3D.readDataAdams(filename,runName,'RBO','LBO','FON');
vtm_t{1}.plotT()
assertEqual(size(vtm_t),[1 501]);
current = vtm_t{1}.getTimestamp();
for i = 2:length(vtm_t)
    assertTrue(vtm_t{i}.getTimestamp>current);
    assertElementsAlmostEqual(1/(vtm_t{i}.getTimestamp-current),...
        33.333333333333336);
    current = vtm_t{i}.getTimestamp();
end

[vtm_t1] = Markers3D.readDataAdams(filename,runName,'RBT','LBT','FTN');
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
    v_rbt = Markers3D(testRBO,testLBO,testFON,0);
    v_lbt = Markers3D(testRBT,testLBT,testFTN,0);
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
[vtm_t] = Markers3D.readDataAdams(filename,runName,...
    'RBT','LBT','FTN');
% ThreeD.plotRun(vtm_t,0.4);
for i = 1:length(adamsColumns)
    adamsColumn = adamsColumns{i}
    adamsData = data.(adamsColumn);
    CompareValue = cell(1,1);
    CompareValue{1} = adamsData';
   
    MeasuredValue = cell(1,1);
    %Normal
    [roll,pitch,yaw,t] = ThreeD.getAndPlotRPYt(vtm_t,...
        adamsColumns{i},false,'timeseries','--o');
    MeasuredValue{1} = chooseData(roll,pitch,yaw,adamsColumn);
    
    rmserrorplot(CompareValue,MeasuredValue,['RMS ERROR: ' runName ': '...
        adamsColumn],true);
end
%figure
%ThreeD.plotRun(vtmk_t,0.1);

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
adamsColumn = {'Roll'};%,'Pitch','Yaw'};
processRun(filename,runName,adamsColumn);

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



[theChanger] = Markers3D.readDataAdams(filename,runName,...
    'RBO','LBO','FON');
[theStatic] = Markers3D.readDataAdams(filename,runName,...
    'RBT','LBT','FTN');

% ThreeD.plotRun(theChanger,0.1);
% ThreeD.plotRun([theStatic;theChanger],0.4);
%  ThreeD.plotRun(theStatic,0.1);
[one_roll,one_pitch,one_yaw,t,theFigure] = ThreeD.getAndPlotRPYt(theChanger,...
    ['CHANGER (B) STATIC (R) DIFF (G) ' runName],false,'timeseries','-o');
[two_roll,two_pitch,two_yaw,t] = ThreeD.getAndPlotRPYt(theStatic,...
    ['SENSOR TWO ' runName],theFigure,'timeseries','--r*');
%STILL NOT SURE WHY! should be:
%diff_t = ThreeD.cellminus(theChanger,theStatic);
diff_t = ThreeD.cellInverseMultiply(theChanger,theStatic);
% notSureWhyDiff = ThreeD.cellMultiply(theChanger,theStatic); 
[diff_roll,diff_pitch,diff_yaw,t] = ThreeD.getAndPlotRPYt(diff_t,...
    ['SENSOR DIFFERENCE: ' runName ],theFigure,'timeseries','--go');
% assertElementsAlmostEqual(diff_roll,zeros(1,...
%     length(diff_roll)));
% assertElementsAlmostEqual(diff_pitch,zeros(1,...
%     length(diff_pitch)));
% assertElementsAlmostEqual(max(abs(diff_yaw)),45,'relative',0.0001);
rmserrorplot({diff_yaw},...
    {steeringAngle},['RMS ERROR: ' runName ...
    ': SteeringAngle'],true);


function test_adams_rollpitchyaw_combined
close all;
filename='test-data/test-data.h5';
runName = '/adams/pitchyawcombined';
processRunCombined(filename,runName);
% runName = '/adams/rollyawcombined';
% processRunCombined(filename,runName);

function test_filterMarkerData
close all;
assertEqual(1,1);
Markers3D.filterMarkerData([0.1,0.2,0.3,0;0.4,0.5,0.6,0.01],10,0.1,15,4);

expected = Markers3D.filterMarkerData([0.1,0.2,0.3,0;
    0.0,0.0,0.0,0.005;
    0.4,0.5,0.6,0.01],1,0.1,15,4);
%assertEqual(any(isnan(expected)),0)

filename='test-data/test-data.h5';
runName = '/vicon';
points_0 = [
        1 -1 0 0;
        0 0 1 0;
        0 0 0 1];
% [vtm_t] = Markers3D.readDataVicon(filename,...
%     runName,'RBO','LBO','FON',points_0);
[vtm_t] = Markers3D.readDataVicon(filename,...
    runName,'RBO','LBO','FON','doFilter',true);
display('Should not filter now');
[vtm_t] = Markers3D.readDataVicon(filename,...
    runName,'RBO','LBO','FON','doFilter',false);

function test_MissingMarkerdata
close all;
assertEqual(1,1);
Markers3D.filterMarkerData([0.1,0.2,0.3,0;0.4,0.5,0.6,0.01],10,0.1,15,4);

expected = Markers3D.filterMarkerData([0.1,0.2,0.3,0;
    0.0,0.0,0.0,0.005;
    0.4,0.5,0.6,0.01],1,0.1,15,4);
%assertEqual(any(isnan(expected)),0)

filename='test-data/test-data.h5';
runName = '/vicon';
points_0 = [
        1 -1 0 0;
        0 0 1 0;
        0 0 0 1];
% [vtm_t] = Markers3D.readDataVicon(filename,...
%     runName,'RBO','LBO','FON',points_0);
[vtm_t] = Markers3D.readDataVicon(filename,...
    runName,'RBO','LBO','FON','doFilter',true);
display('Should not filter now');
[vtm_t] = Markers3D.readDataVicon(filename,...
    runName,'RBO','LBO','FON','doFilter',false);



function test_findOutliers
markerdata = [0.01, 0.0, 0.03, 0.04, 0.05];
[markeroutliers] = Markers3D.findOutliers(markerdata,...
                100,200);
expected = [0.01, NaN, 0.03, 0.04, 0.05];
assertEqual(markeroutliers,expected);

markerdata = [0.02, 0.02, 0.03, 0.04, 0.05, 0.0, 0, 0, 0.1, 0.12, 0.13, 0, 0.16 ];
[markeroutliers] = Markers3D.findOutliers(markerdata,...
                1,200);
expected = [0.02, 0.02, 0.03, 0.04, 0.05, NaN, NaN, NaN, 0.1, 0.12, 0.13, NaN, 0.16];
assertEqual(markeroutliers,expected);

function test_fillGaps
markerOutliers = [0.02, 0.02, 0.03, 0.04, 0.05, NaN, NaN, NaN, 0.1, 0.12, 0.13, NaN, 0.16];
expected = [0.02, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.085, 0.1, 0.12, 0.13, 0.145, 0.16];
[filtermarkers] = Markers3D.fillGaps(markerOutliers,...
                10);   
assertEqual(filtermarkers,expected);

function test_findFillGaps
markerdata = [3,3.1,3.2,3.3,3.4,3.5;4,0,4.2,0,4.4,0;0,5.1,5.2,0,5.4,0]';
expected = [3,3.1,3.2,3.3,3.4,3.5;4,4.1,4.2,4.3,4.4,4.5;5,5.1,5.2,5.3,5.4,5.5]';
[filMarkers,gapArrayMarker] = Markers3D.findFillGaps(markerdata,...
    100, 100,200);
assertEqual(filMarkers,expected);



function  test_eraseNan
marker1 = [3,3.1,3.2, 3; 3.3,3.4,3.5, 3; 4,NaN,4.2, 3; 0,4.4,NaN,3; 0,5.1,5.2,3; NaN,5.4,NaN,3];
marker2 = [2,3.1,0, 3; 3,3.4,3.5, 3; 4,NaN,4.2, 3; 0,4.4,NaN,3; 0,5.1,5.2,3; NaN,5.4,NaN,3];
marker3 = [3,3.1,3.2, 3; 3.3,3.4,3.5, 3; 4,NaN,4.2, 3; 0,4.4,NaN,3; 0,5.1,5.2,3; NaN,5.4,NaN,3];
expected1 = [3,3.1,3.2, 3; 3.3,3.4,3.5, 3; 0,0,0,0; 0,0,0,0; 0,5.1,5.2,3; 0,0,0,0];
expected2 = [2,3.1,0, 3; 3,3.4,3.5, 3; 0,0,0,0; 0,0,0,0; 0,5.1,5.2,3; 0,0,0,0];
expected3 = [3,3.1,3.2, 3; 3.3,3.4,3.5, 3; 0,0,0,0; 0,0,0,0; 0,5.1,5.2,3; 0,0,0,0];


[marker1n,marker2n,marker3n] = Markers3D.eraseNan(marker1,marker2,marker3);

assertEqual([marker1n,marker2n,marker3n],[expected1,expected2,expected3]);

marker1 = [NaN, NaN, NaN, NaN; NaN, NaN, NaN, NaN; NaN, NaN, NaN, NaN; NaN, NaN, NaN, NaN; NaN, NaN, NaN, NaN; NaN, NaN, NaN, NaN];
marker2 = [2,3.1,0,3; 3,3.4,3.5,3; 4,NaN,4.2,3; 0,4.4,NaN,3; 0,5.1,5.2,3; NaN,5.4,NaN,3];
marker3 = [3,3.1,3.2,3; 3.3,3.4,3.5,3; 4,NaN,4.2,3; 0,4.4,NaN,3; 0,5.1,5.2,3; NaN,5.4,NaN,3];
expected1 = [0,0,0,0; 0,0,0,0;0,0,0,0; 0,0,0,0; 0,0,0,0; 0,0,0,0];
expected2 = [0,0,0,0; 0,0,0,0; 0,0,0,0; 0,0,0,0; 0,0,0,0; 0,0,0,0];
expected3 = [0,0,0,0; 0,0,0,0; 0,0,0,0; 0,0,0,0; 0,0,0,0; 0,0,0,0];


[marker1n,marker2n,marker3n] = Markers3D.eraseNan(marker1,marker2,marker3);

assertEqual([marker1n,marker2n,marker3n],[expected1,expected2,expected3]);
