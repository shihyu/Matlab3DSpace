function [ errorQuat,errorEuler ] = quaternionerror( q1,q2 )
%QUATERNIONERROR Calculates the error/difference between two quaternions.
if norm(q1) ~= 1.0
    q1 = quaternionnormalise(q1);
end
if norm(q2) ~= 1.0
    q2 = quaternionnormalise(q2);
end

errorQuat =  quaternionproduct(q1,...
    quaternionconjugate(q2))';
errorEuler = invrpy(quaternion2matrix(errorQuat));
end

