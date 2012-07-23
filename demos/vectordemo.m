format compact;
clear all;
clc;
%Simple demo to find out the Roll Picth and Yaw angles of a vector.
q1 = [0,0,0];
q2=[1,1,1];
%Normalises the points.
q2_norm = ThreeMarkers.normWithOffset(q2,q1)
 
%Zero Frame:
q1_0 = [0,0,0];
%Positive vector along the x-axis;
q2_0 = [1,0,0];

%Get rotation matrix
R = absor([q1_0',q2_0'],[q1',q2_norm'])
quat = R.q
R = R.R;
%Lets test:
q2_calc = (R*q2_0')'
q2_norm

%Get the roll pitch and yaw from the Rotation matrixx
[euler] = invrpy(R);
%Convert to degrees from radians.
euler = euler/pi*180;
roll = euler(1)
pitch = euler(2)
yaw = euler(3)


%Or get it from the quaternion.
euler = quaternion2euler(quat,true,'zxz')

euler = quaternion2euler(quat,true)%Defaults to XYZ.
