function [ q ] = quaternionnormalise(q)
%quatnormalise Normalises a quaternion.
q = q./sqrt ( q(1).^2 + q(2).^2+q(3).^2+q(4).^2);
%q = q./sqrt ( sum(q.^2));
end

