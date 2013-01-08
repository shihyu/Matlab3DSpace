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
assertEqual(result,[false,false,true]);