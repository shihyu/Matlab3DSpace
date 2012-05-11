function test_suite = test_quatutils
% MUST BE IN THE DIRECTORY WHERE THE TEST RUNS.
initTestSuite;

function test_quaternionerror
rightback = [1 0 0];
leftback = [-1 0 0];
front = [0 1 0];
theTimestamp = 2.3;
cvt = ViconThreeMarkers(rightback,...
        leftback,front,theTimestamp);
assertEqual(cvt.get0,[1 -1 0 0; 0 0 1 0; 0 0 0 1; 1 1 1 1]);

assertEqual(cvt.getH,eye(4));
assertEqual(cvt.get0,cvt.getT);
q2 = matrix2quaternion(eye(4));
[errorQuat,errorEuler] = quaternionerror(cvt.getQ,q2);
assertEqual(errorQuat,[1 0 0 0])
assertEqual(errorEuler,[0 0 0])

theta = -2*pi;
H = [ 1     0           0        0
      0  cos(theta) -sin(theta)  0
      0  sin(theta)  cos(theta)  0
      0     0           0        1];
 quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 0 0])


theta = -0.2*pi;
H = [ 1     0           0        0
      0  cos(theta) -sin(theta)  0
      0  sin(theta)  cos(theta)  0
      0     0           0        1];
 quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 0 theta])

theta = -1.2*pi;
H = [ 1     0           0        0
      0  cos(theta) -sin(theta)  0
      0  sin(theta)  cos(theta)  0
      0     0           0        1];
 quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 0 0.8*pi])


function test_slerp
display('Testing slerp:')
theta = -0.2*pi;
H = [ 1     0           0        0
      0  cos(theta) -sin(theta)  0
      0  sin(theta)  cos(theta)  0
      0     0           0        1];
quaternion = matrix2quaternion(H)'
quaternionNorm = quaternionnormalise(quaternion);
assertElementsAlmostEqual(quaternion,quaternionNorm)

startQ=ThreeMarkers([1 0 0 0 0]);
endQ=ThreeMarkers([quaternion 1.0]);


[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0])
assertElementsAlmostEqual(errorEuler,[0 0 theta])


midQ = [slerp(startQ.getQ, endQ.getQ, 0.5, eps)' 0.5]

midQ = ThreeMarkers(midQ);
ThreeMarkers.plotRun({startQ;...
    midQ;...
    endQ});

t = 0.0:0.01:1.0;
tm_t = cell(1,size(t,2));
spline_t = slerp(startQ.getQ,...
            endQ.getQ, t, eps);
figure
parfor i = 1:size(spline_t,2)
    tm_t{i} = ThreeMarkers([spline_t(:,i)' t(i)]);
end
ThreeMarkers.plotRun(tm_t);

startQ=ThreeMarkers([1 0 0 0]);
endQ=ThreeMarkers([-1 0 0 0]);
spline_t = slerp(startQ.getQ,...
            endQ.getQ, t, eps);
figure
parfor i = 1:size(spline_t,2)
    tm_t{i} = ThreeMarkers([spline_t(:,i)']);
end
ThreeMarkers.plotRun(tm_t);



function test_quaternion2euler()
quat = [1,0,0,0,0];
euler = quaternion2euler(quat,false)
assertEqual(euler,[0 0 0]);
eulerDeg = quaternion2euler(quat,true)
assertEqual(euler,[0 0 0]);
quat = [ cos(30/180*pi) sin(30/180*pi) 0 0];
format long;
euler = quaternion2euler(quat,false)
assertElementsAlmostEqual(euler,[0 0  1.047197551196598]);
eulerDeg = quaternion2euler(quat,true)
assertEqual(eulerDeg,[0 0 60]);