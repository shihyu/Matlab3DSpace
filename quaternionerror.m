function [ errorQuat,errorEuler ] = quaternionerror( q1,q2 )
%QUATERNIONERROR Summary of this function goes here
%   Detailed explanation goes here
errorQuat =  quaternionproduct(q1,...
        quaternionconjugate(q2));
errorEuler = invrpy(quaternion2matrix(errorQuat));
end

