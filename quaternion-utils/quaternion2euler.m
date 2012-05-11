function [ eulerAngles] = quaternion2euler(q1,inDegrees)
    %QUATERNION2EULER Get the roll pitch and yaw of a quaternion
    eulerAngles = invrpy(quaternion2matrix(q1));
    if inDegrees==true
        %display('Converting to degrees')
        eulerAngles = eulerAngles./pi.*180;
    end
end