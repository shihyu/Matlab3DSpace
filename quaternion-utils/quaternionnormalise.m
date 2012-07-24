function [ q ] = quaternionnormalise(q)
%quatnormalise Normalises a quaternion.
%q = q./sqrt ( q(1).^2 + q(2).^2+q(3).^2+q(4).^2);
if norm(q)-1.0<eps(1)*10
    q = q./norm(q);
end
end

