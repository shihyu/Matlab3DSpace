function test_suite = test_quatutils
% MUST BE IN THE DIRECTORY WHERE THE TEST RUNS.
initTestSuite;

function test_quaternionerror
rightback = [0 -1 0];
leftback = [0 1  0];
front = [1 0  0];
theTimestamp = 2.3;
cvt = Markers3D(rightback,...
        leftback,front,theTimestamp);
assertEqual(cvt.get0,[0 0 1 0; -1 1 0 0; 0 0 0 1; 1 1 1 1]);


assertEqual(cvt.getH,eye(4));
assertEqual(cvt.get0,cvt.getT);
q2 = matrix2quaternion(eye(4));
[errorQuat,errorEuler] = quaternionerror(cvt.getQ,q2);
assertEqual(errorQuat,[1 0 0 0])
assertEqual(errorEuler,[0 0 0])
%Z
theta = -2*pi;
H = rotz(theta);
quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 0 0])


theta = -0.2*pi;
H = rotz(theta);
quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[theta 0 0])

theta = -0.2*pi;
H = rotz(theta);
quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror([1 0 0 0],quaternion);
assertElementsAlmostEqual(errorEuler,[-theta 0 0])

theta = -0.7*pi;
H = rotz(theta);
quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror([1 0 0 0],quaternion);
assertElementsAlmostEqual(errorEuler,[-1.3*pi+2*pi 0 0])

theta = -1.2*pi;
H = rotz(theta);
 quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0.8*pi 0 0])

%Y
theta = -2*pi;
H = roty(theta);
quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 0 0])


theta = -0.2*pi;
H = roty(theta);
quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 theta 0])

theta = -0.5*pi;
H = roty(theta);
quaternion = matrix2quaternion(H)
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 -0.5*pi 0])

theta = 0.5*pi;
H = roty(theta);
quaternion = matrix2quaternion(H)
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 0.5*pi 0])


theta = -0.4*pi;
H = roty(theta);
quaternion = matrix2quaternion(H)
euler = invrpy(H);
%assertElementsAlmostEqual(euler,[0 -0.7*pi 0])
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
errorEulerQuat = quaternion2euler(errorQuat,false)
assertElementsAlmostEqual(errorEuler,[0 -0.4*pi 0]);

theta = 0.4*pi;
%Same as 0.8pi = the same as 0.5-0.3 = -0.2pi
H = roty(theta);
quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 0.4*pi 0])

%X
theta = -2*pi;
H = rotx(theta);
quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 0 0])


theta = -0.2*pi;
H = rotx(theta);
quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 0 theta])

theta = -1.2*pi;
H = rotx(theta);
 quaternion = matrix2quaternion(H);
[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0]);
assertElementsAlmostEqual(errorEuler,[0 0 0.8*pi])


function test_slerp
display('Testing slerp:')
theta = -0.2*pi;
H = rotz(theta);
quaternion = matrix2quaternion(H)
quaternionNorm = quaternionnormalise(quaternion);
assertElementsAlmostEqual(quaternion,quaternionNorm)

startQ=ThreeD([1 0 0 0]);
endQ=ThreeD(quaternion);


[errorQuat,errorEuler] = quaternionerror(quaternion,[1 0 0 0])
assertElementsAlmostEqual(errorEuler,[theta 0 0])


midQ = [slerp(startQ.getQ, endQ.getQ, 0.5, eps)']

midQ = ThreeD(midQ);
ThreeD.plotRun({startQ;...
    midQ;...
    endQ});

t = 0.0:0.01:1.0;
tm_t = cell(1,size(t,2));
spline_t = slerp(startQ.getQ,...
            endQ.getQ, t, eps);
figure
parfor i = 1:size(spline_t,2)
    tm_t{i} = Quat3D([spline_t(:,i)' t(i)]);
end
ThreeD.plotRun(tm_t);

startQ=ThreeD([1 0 0 0]);
endQ=ThreeD([-1 0 0 0]);
spline_t = slerp(startQ.getQ,...
            endQ.getQ, t, eps);
figure
parfor i = 1:size(spline_t,2)
    tm_t{i} = ThreeD([spline_t(:,i)']);
end
ThreeD.plotRun(tm_t);



function test_quaternion2euler
quat = [1,0,0,0,0];
euler = quaternion2euler(quat,false)
assertEqual(euler,[0 0 0]);
eulerDeg = quaternion2euler(quat,true)
assertEqual(euler,[0 0 0]);
quat = [ cos(30/180*pi) sin(30/180*pi) 0 0];
format long;
euler = quaternion2euler(quat,false)
assertElementsAlmostEqual(euler,[1.047197551196598 0 0]);
eulerDeg = quaternion2euler(quat,true)
assertEqual(eulerDeg,[60 0 0]);

function test_matrix2quaternion
H=rotx(pi/7)
quat = matrix2quaternion(H(1:3,1:3))
Hcalc = quaternion2matrix(quat);
assertElementsAlmostEqual(Hcalc-H,zeros(4))
qtm = ThreeD(quat)
assertElementsAlmostEqual(qtm.getH-H,zeros(4))
assertEqual(qtm.getQ,quat);
euler = qtm.getRPY(false)
assertElementsAlmostEqual([pi/7,0, 0],euler)

H = H*roty(pi/13);
quat = matrix2quaternion(H)
Hcalc = quaternion2matrix(quat);
assertElementsAlmostEqual(Hcalc-H,zeros(4))
qtm = ThreeD(quat)
assertElementsAlmostEqual(qtm.getH-H,zeros(4))
assertEqual(qtm.getQ,quat);
euler = qtm.getRPY(false)
assertElementsAlmostEqual([ pi/7,pi/13,0],euler)

H = H*rotz(pi/9);
quat = matrix2quaternion(H)
Hcalc = quaternion2matrix(quat);
assertElementsAlmostEqual(Hcalc-H,zeros(4))
qtm = ThreeD(quat)
assertElementsAlmostEqual(qtm.getH-H,zeros(4))
assertEqual(qtm.getQ,quat);
euler = qtm.getRPY(false)
assertElementsAlmostEqual([pi/7,pi/13, pi/9],euler)

%Now lets test multiplication.
H=rotx(pi/7)
quat = matrix2quaternion(H(1:3,1:3))
Hcalc = quaternion2matrix(quat);
assertElementsAlmostEqual(Hcalc-H,zeros(4))
qtm = ThreeD(quat)
assertElementsAlmostEqual(qtm.getH-H,zeros(4))
assertEqual(qtm.getQ,quat);
euler = qtm.getRPY(false)
assertElementsAlmostEqual([pi/7,0, 0],euler)

H = roty(pi/13);
quat = matrix2quaternion(H)
Hcalc = quaternion2matrix(quat);
assertElementsAlmostEqual(Hcalc-H,zeros(4))
qtm1 = ThreeD(quat)
assertElementsAlmostEqual(qtm1.getH-H,zeros(4))
assertEqual(qtm1.getQ,quat);
euler = qtm1.getRPY(false)
assertElementsAlmostEqual([0,pi/13, 0],euler)

H = rotz(pi/9);
quat = matrix2quaternion(H)
Hcalc = quaternion2matrix(quat);
assertElementsAlmostEqual(Hcalc-H,zeros(4))
qtm2 = ThreeD(quat)
assertElementsAlmostEqual(qtm2.getH-H,zeros(4))
assertEqual(qtm2.getQ,quat);
euler = qtm2.getRPY(false)
assertElementsAlmostEqual([0,0, pi/9],euler)

qtm4 = qtm.*qtm1;
H=rotx(pi/7)*roty(pi/13)
newTM = ThreeD(H);
assertElementsAlmostEqual(qtm4.getH-H,zeros(4))
assertElementsAlmostEqual(qtm4.getQ,newTM.getQ);
euler = qtm4.getRPY(false);
eulerNew = newTM.getRPY(false)
assertElementsAlmostEqual(eulerNew,euler)
assertElementsAlmostEqual([pi/7,pi/13, 0],euler)


qtm4 = qtm.*qtm1.*qtm2;
H=rotx(pi/7)*roty(pi/13)*rotz(pi/9)
newTM = ThreeD(H);
assertElementsAlmostEqual(qtm4.getH-H,zeros(4))
assertElementsAlmostEqual(qtm4.getQ,newTM.getQ);
euler = qtm4.getRPY(false);
eulerNew = newTM.getRPY(false)
assertElementsAlmostEqual(eulerNew,euler)
assertElementsAlmostEqual([pi/7,pi/13, pi/9],euler)
