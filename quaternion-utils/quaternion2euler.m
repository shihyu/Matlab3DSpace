function [ eulerAngles] = quaternion2euler(theQuaternion,inDegrees,...
    varargin)
%QUATERNION2EULER Get the Euler angles of the quaterion
%If an extra argument is give then set it to
% 'zxz' to get ZXZ euler angles.
%The default is xyz, the roll pitch and yaw.
if norm(theQuaternion) ~= 1.0
    theQuaternion = quaternionnormalise(theQuaternion);
end
if ((~isempty(varargin))&&(~isempty(varargin{1})))
    typeOfEuler = varargin{1};
else
    typeOfEuler = 'xyz';
end
q0 = theQuaternion(1);
q1 = theQuaternion(2);
q2 = theQuaternion(3);
q3 = theQuaternion(4);
if strcmp(typeOfEuler,'xyz')
%     display('Normal');
    roll = atan2(2*(q0*q1+q1*q3),1-2*(q1^2+q2^2));
    pitch = asin(2*(q0*q2-q3*q1));
    yaw = atan2(2*(q0*q3+q1*q2),1-2*(q2^2+q3^2));
    eulerAngles = [roll,pitch,yaw];
elseif strcmp(typeOfEuler,'zxz')
%     display('ZXZ')
    zz=atan2(2.*(q1.*q3 + q0.*q2),-2.*(q2.*q3 - q0.*q1));
    xx=acos(q0.^2 - q1.^2 - q2.^2 + q3.^2);
    zz1=atan2(2.*(q1.*q3 - q0.*q2),2.*(q2.*q3 + q0.*q1));
    eulerAngles = [zz,xx,zz1];
else
    error(message('matlab3dspace:Unknown transform',typeOfEuler));
end
if inDegrees
    eulerAngles = eulerAngles./pi*180;
end
end