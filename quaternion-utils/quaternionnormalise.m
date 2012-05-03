function [ q ] = quaternionnormalise(q)
%quatnormalise Normalises a quaternion.
 q = q./sqrt ( sum(q.^2));
end

