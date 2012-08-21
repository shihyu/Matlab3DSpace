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
ThreeD.points_0
expected = H*ThreeD.points_0
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
%assertElementsAlmostEqual(errorEuler,[-theta 0 0 ])
[errorQuat,errorEuler] = quaternionerror([1 0 0 0],cvt.getQ);
%assertElementsAlmostEqual(errorEuler,[theta 0 0 ])

function test_markerdiffs
%Pitch Yaw Difference;
H1=roty(pi/6);
H2=rotz(pi/3.4)*roty(pi/6);
H12 = rotz(pi/3.4);
point0 = ThreeD.points_0;
point1 = H1*point0;
point2 = H2*point0;

marker1 = Markers3D(point1(1:3,1)',point1(1:3,2)',point1(1:3,3)',0);
marker2 = Markers3D(point2(1:3,1)',point2(1:3,2)',point2(1:3,3)',0);

assertElementsAlmostEqual(marker1.getRPY(false),[0 pi/6 0 ]);
assertElementsAlmostEqual(marker1.getH,H1);
assertElementsAlmostEqual(marker2.getH,H2);

%From screw theory: H12=H2*(H1)^-1
marker12 = marker2-marker1;
assertElementsAlmostEqual(marker12.getRPY(false),[0 0 pi/3.4]);

%Roll Yaw Difference;
H1=rotx(pi/2);
H2=rotz(pi/5.2)*rotx(pi/2);
H12 = rotz(pi/5.2);
point0 = ThreeD.points_0;
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
point0 = ThreeD.points_0;
point1 = H1*point0;
point2 = H2*point0;

marker1 = Markers3D(point1(1:3,1)',point1(1:3,2)',point1(1:3,3)',0);
marker2 = Markers3D(point2(1:3,1)',point2(1:3,2)',point2(1:3,3)',0);

assertElementsAlmostEqual(marker1.getH,H1);
assertElementsAlmostEqual(marker2.getH,H2);

%From screw theory: H12=H2*(H1)^-1
marker12 = marker2-marker1;
assertElementsAlmostEqual(marker12.getRPY(false),[0 0 pi/5.2]);

%Pitch Yaw Difference; (45 degrees).
H1=roty(pi/4);
H2=rotz(pi/4)*roty(pi/4);
H12 = rotz(pi/4);
point0 = ThreeD.points_0;
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
[vtm_t] = Markers3D.readDataVicon(filename,...
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


function test_vicon3d_objectcreation
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
% plot(steeringAngle);
% figure
%kobasch
[theChanger] = Markers3D.readDataAdams(filename,runName,...
    'RBO','LBO','FON');
[theStatic] = Markers3D.readDataAdams(filename,runName,...
    'RBT','LBT','FTN');
% ThreeD.plotRun(theChanger,0.1);
%ThreeD.plotRun([one_t;two_t],0.1);
%  ThreeD.plotRun(theStatic,0.1);
[one_roll,one_pitch,one_yaw,t,theFigure] = ThreeD.getAndPlotRPYt(theChanger,...
    ['CHANGER (B) STATIC (R) DIFF (G) ' runName],false,'timeseries','-o');
[two_roll,two_pitch,two_yaw,t] = ThreeD.getAndPlotRPYt(theStatic,...
    ['SENSOR TWO ' runName],theFigure,'timeseries','--r*');
diff_t = ThreeD.cellminus(theChanger,theStatic);

 achanger = theChanger{20};
 astatic = theStatic{20};
 thediff = diff_t{20}
 achanger.getRPY(false)
 astatic.getRPY(false)
 Hy =  roty(pi/4)
 astatic.getH
 Hzy = roty(pi/4)*rotz(pi/4)
 achanger.getH
 ourdiff = astatic'.*achanger;
 thediff.getRPY(false)
 ourdiff.getRPY(false)
 figure
 hold on
%  astatic.plotT
 achanger.plotT
 newChanger = thediff.*astatic;
 newChanger.plotT
 newChanger2= astatic.*ourdiff;
 newChanger2.plotT
 
 
notSureWhyDiff = ThreeD.inverseMultiply(theStatic,theChanger); 
[diff_roll,diff_pitch,diff_yaw,t] = ThreeD.getAndPlotRPYt(diff_t,...
    ['SENSOR DIFFERENCE: ' runName ],theFigure,'timeseries','--go');
[diff_roll_notsureWhy,diff_pitch,diff_yaw_notsureWhy,t] = ThreeD.getAndPlotRPYt(notSureWhyDiff,...
    ['SENSOR DIFFERENCE: ' runName ],theFigure,'timeseries','--mo');
assertElementsAlmostEqual(diff_roll_notsureWhy,zeros(1,...
    length(diff_roll_notsureWhy)));
assertElementsAlmostEqual(diff_pitch,zeros(1,...
    length(diff_pitch)));
assertElementsAlmostEqual(max(abs(diff_yaw_notsureWhy)),45,'relative',0.0001);
rmserrorplot({diff_yaw,diff_yaw_notsureWhy},...
    {steeringAngle,steeringAngle},['RMS ERROR: ' runName ...
    ': SteeringAngle'],true);


function test_adams_rollpitchyaw_combined
close all;
filename='test-data/test-data.h5';
runName = '/adams/pitchyawcombined';
processRunCombined(filename,runName);
runName = '/adams/rollyawcombined';
processRunCombined(filename,runName);
