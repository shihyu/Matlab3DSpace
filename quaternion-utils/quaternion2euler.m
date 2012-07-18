function [ eulerAngles] = quaternion2euler(theQuaternion,inDegrees)
    %QUATERNION2EULER Get the roll pitch and yaw of a quaternion
    if norm(theQuaternion) ~= 1.0
        theQuaternion = quaternionnormalise(theQuaternion);
    end
    q0 = theQuaternion(1);
    q1 = theQuaternion(2);
    q2 = theQuaternion(3);
    q3 = theQuaternion(4);
    roll = atan2(2*(q0*q1+q1*q3),1-2*(q1^2+q2^2));
    pitch = asin(2*(q0*q2-q3*q1));
    yaw = atan2(2*(q0*q3+q1*q2),1-2*(q2^2+q3^2));
    eulerAngles = [roll,pitch,yaw];
    if inDegrees
        eulerAngles = eulerAngles./pi*180;
    end
end