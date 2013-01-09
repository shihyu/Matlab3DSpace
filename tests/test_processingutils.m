function test_suite = test_processingutils
initTestSuite;

function test_absor
H1=roty(pi/6);
H2=rotz(pi/3.4)*roty(pi/6);
H12 = rotz(pi/3.4);
point0 = ThreeD.get0;
point1 = H1*point0;
point2 = H2*point0;
H1_absor = absor(point0(1:3,:),point1(1:3,:));
assertElementsAlmostEqual(H1,H1_absor.M)
H2_absor = absor(point0(1:3,:),point2(1:3,:));
assertElementsAlmostEqual(H2,H2_absor.M)
H12_absor = absor(point1(1:3,:),point2(1:3,:));
assertElementsAlmostEqual(H12,H12_absor.M);
point2est = H12*point1;
assertEqual(point2est,point2);
H12_est = H2*(H1');
assertElementsAlmostEqual(H12,H12_est);



