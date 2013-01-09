function test_suite = test_vicon3d
% MUST BE IN THE DIRECTORY WHERE THE TEST RUNS.
initTestSuite;

function test_isNanOrZero
marker1 = [NaN,NaN,NaN];
result = RawMarkers.isNanOrZero(marker1)
assertTrue(result);

marker1 = [0.0,0.0,0.0];
result = RawMarkers.isNanOrZero(marker1)
assertTrue(result);

marker1 = [NaN,NaN,0.0];
result = RawMarkers.isNanOrZero(marker1)
assertTrue(result);

marker1 = [0.0,NaN,NaN];
result = RawMarkers.isNanOrZero(marker1)
assertTrue(result);

marker1 = [0.0,0.1,NaN];
result = RawMarkers.isNanOrZero(marker1)
assertFalse(result);

marker1 = [0.3,0.1,NaN];
result = RawMarkers.isNanOrZero(marker1)
assertFalse(result);
marker1 = [0.3,0.1,0.0];
result = RawMarkers.isNanOrZero(marker1)
assertFalse(result);

marker1 = [NaN,NaN,NaN;NaN,NaN,NaN]
marker1
result = RawMarkers.isNanOrZero(marker1)
assertEqual(result,[true;true]);

marker1 = [0.3,0.1,0.0;0.3,0.1,0.0];
result = RawMarkers.isNanOrZero(marker1)
assertEqual(result,[false;false]);

marker1 = [0.3,0.1,0.01232;0.3,0.1,0.0123213];
result = RawMarkers.isNanOrZero(marker1)
assertEqual(result,[false;false]);
%One Marker Missing.
marker1 = [NaN,NaN,NaN];
result = RawMarkers.isNanOrZero(marker1,'allOrSome',false)
assertTrue(result);

marker1 = [0.0,0.0,0.0];
result = RawMarkers.isNanOrZero(marker1,'allOrSome',false)
assertTrue(result);

marker1 = [NaN,NaN,0.0];
result = RawMarkers.isNanOrZero(marker1,'allOrSome',false)
assertTrue(result);

marker1 = [0.0,NaN,NaN];
result = RawMarkers.isNanOrZero(marker1,'allOrSome',false)
assertTrue(result);

marker1 = [0.0,0.1,NaN];
result = RawMarkers.isNanOrZero(marker1,'allOrSome',false)
assertTrue(result);

marker1 = [0.3,0.1,NaN];
result = RawMarkers.isNanOrZero(marker1,'allOrSome',false)
assertTrue(result);
marker1 = [0.3,0.1,0.0];
result = RawMarkers.isNanOrZero(marker1,'allOrSome',false)
assertTrue(result);

marker1 = [NaN,NaN,NaN;NaN,NaN,NaN]
marker1
result = RawMarkers.isNanOrZero(marker1,'allOrSome',false)
assertEqual(result,[true;true]);

marker1 = [0.3,0.1,0.0;0.3,0.1,0.0];
result = RawMarkers.isNanOrZero(marker1,'allOrSome',false)
assertEqual(result,[true;true]);

marker1 = [0.3,0.1,0.01232;0.3,0.1,0.0123213];
result = RawMarkers.isNanOrZero(marker1,'allOrSome',false)
assertEqual(result,[false;false]);

marker1 = [0.3,0.1,0.01232;0.3,0.1,0.0123213];
result = RawMarkers.isNanOrZero(marker1,'keepColumnLogicResult',true)
assertEqual(result,[false false false;false false false]);



function test_areTheMarkersWellSpaced
marker1 = [NaN,NaN,NaN];
marker2=[NaN,NaN,NaN];
marker3=[NaN,NaN,NaN];
result = RawMarkers.areTheMarkersWellSpaced([marker1,marker2,marker3])
assertEqual(result,[false,false,false]);

marker1 = [0,0,0];
marker2=[0,0,0];
marker3=[0,0,0];
result = RawMarkers.areTheMarkersWellSpaced([marker1,marker2,marker3])
% assertEqual(result,[true,true,]);

marker1 = [NaN,1,NaN];
marker2=[1,NaN,1];
marker3=[1,0,NaN];
result = RawMarkers.areTheMarkersWellSpaced([marker1,marker2,marker3])
assertEqual(result,[false,false,false]);


marker1 = [0,0,0];
marker2=[1,0,0];
marker3=[sin(pi/6),cos(pi/6),0];
markers = [marker1;marker2;marker3];
plot3(markers(:,1),markers(:,2),markers(:,3))
len1 =  norm(marker1-marker2)
len2 =  norm(marker1-marker3)
len3 = norm(marker2-marker3)
result = RawMarkers.areTheMarkersWellSpaced([marker1,marker2,marker3])
assertEqual(result,[true,true,true]);

marker1 = [0,0,0]+[1,1,1];
marker2=[1,0,0]+[1,1,1];
marker3=[sin(pi/6),cos(pi/6),0]+[1,1,1];
markers = [marker1;marker2;marker3];
len1 =  norm(marker1-marker2)
len2 =  norm(marker1-marker3)
len3 = norm(marker2-marker3)
result = RawMarkers.areTheMarkersWellSpaced([marker1,marker2,marker3])
assertEqual(result,[true,true,true]);

marker1 = [0,0,0]+[100,100,100];
marker2=[1,0,0]+[1,1,1];
marker3=[sin(pi/6),cos(pi/6),0]+[1,1,1];
markers = [marker1;marker2;marker3];
len1 =  norm(marker1-marker2)
len2 =  norm(marker1-marker3)
len3 = norm(marker2-marker3)
result = RawMarkers.areTheMarkersWellSpaced([marker1,marker2,marker3]);
assertEqual(result,[true,false,false]);

display('Weird Test')
marker1 = [0,0,0]+[1,1,1];
marker2=[1,0,0]*100+[1,1,1];
marker3=[sin(pi/6),cos(pi/6),0]*100+[1,1,1];
len1 =  norm(marker1-marker2)
len2 =  norm(marker1-marker3)
len3 = norm(marker2-marker3)
[result,distances] = RawMarkers.areTheMarkersWellSpaced([marker1,marker2,marker3])
assertElementsAlmostEqual(distances,[100,100,100])
assertEqual(result,[true,true,true]);

marker1 = [0,0,0]+[1,1,1]*100;
marker2=[1,0,0]*100+[1,1,1]*100;
marker3=[sin(pi/6),cos(pi/6),0]*100+[1,1,1]*100;
markers = [marker1;marker2;marker3];
len1 =  norm(marker1-marker2)
len2 =  norm(marker1-marker3)
len3 = norm(marker2-marker3)
[result,distances] = RawMarkers.areTheMarkersWellSpaced([marker1,marker2,marker3])
assertEqual(result,[true,true,true]);


marker1 = [0,0,0]+[1,1,1]*100;
marker2=[1,0,0]*100+[1,1,1]*100;
marker3=[sin(pi/6),cos(pi/6),0]*100+[1,1,1]*85;
markers = [marker1;marker2;marker3];
len1 =  norm(marker1-marker2)
len2 =  norm(marker1-marker3)
len3 = norm(marker2-marker3)
[result,distances] = RawMarkers.areTheMarkersWellSpaced([marker1,marker2,marker3])
assertEqual([len1,len2,len3],distances)
assertEqual(result,[false,true,false]);

marker1 = [1,1,1]
marker2=[20,0,0]
marker3=[20,50,0]
markers = [marker1;marker2;marker3];
len1 =  norm(marker1-marker2)
len2 =  norm(marker1-marker3)
len3 = norm(marker2-marker3)
result = RawMarkers.areTheMarkersWellSpaced([marker1,marker2,marker3])
<<<<<<< HEAD
assertEqual(result,[false,false,false]);

function test_findOutliers
markerdata = [0.01, 0.0, 0.03, 0.04, 0.05]';
[markeroutliers] = RawMarkers.findOutliers(markerdata);
expected = [0.01, NaN, 0.03, 0.04, 0.05]';
assertEqual(markeroutliers,expected);

markerdata = [0.02, 0.02, 0.03, 0.04, 0.05, 0.0, 0, 0, 0.1, 0.12, 0.13, 0, 0.16 ]';
[markeroutliers] = RawMarkers.findOutliers(markerdata);
expected = [0.02, 0.02, 0.03, 0.04, 0.05, NaN, NaN, NaN, 0.1, 0.12, 0.13, NaN, 0.16]';
assertEqual(markeroutliers,expected);

function test_fillGaps
markerOutliers = [0.02, 0.02, 0.03, 0.04, 0.05, NaN, NaN, NaN, 0.1, 0.12, 0.13, NaN, 0.16]';
expected = [0.02, 0.02, 0.03, 0.04, 0.05, 0.0625, 0.075, 0.0875, 0.1, 0.12, 0.13, 0.145, 0.16]';
[filtermarkers] = RawMarkers.fillGaps(markerOutliers);
assertEqual(filtermarkers,expected);

function test_findFillGaps
rawData = [3,3.1,3.2,3.3,3.4,3.5;4,0,4.2,0,4.4,0;0,5.1,5.2,0,5.4,0;3,3.1,3.2,3.3,3.4,3.5;4,0,4.2,0,4.4,0;0,5.1,5.2,0,5.4,0;3,3.1,3.2,3.3,3.4,3.5;4,0,4.2,0,4.4,0;0,5.1,5.2,0,5.4,0;0,0.1,0.2,0.3,0.4,0.5]';
expected = [3,3.1,3.2,3.3,3.4,3.5;4,4.1,4.2,4.3,4.4,NaN;NaN,5.1,5.2,5.3,5.4,NaN;3,3.1,3.2,3.3,3.4,3.5;4,4.1,4.2,4.3,4.4,NaN;NaN,5.1,5.2,5.3,5.4,NaN;3,3.1,3.2,3.3,3.4,3.5;4,4.1,4.2,4.3,4.4,NaN;NaN,5.1,5.2,5.3,5.4,NaN;0,0.1,0.2,0.3,0.4,0.5]';
[filMarkers,gapArrayMarker] = RawMarkers.findFillGaps(rawData);
assertEqual(filMarkers,expected);


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

