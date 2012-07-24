function [ errorQuat,errorEuler ] = quaternionerror( q1,q2 )
%QUATERNIONERROR Calculates the error/difference between two quaternions.
q1 = quaternionnormalise(q1);
q2 = quaternionnormalise(q2);
errorQuat =  quaternionproduct(q1,...
    quaternionconjugate(q2))';
errorEuler = invrpy(quaternion2matrix(errorQuat));
end

