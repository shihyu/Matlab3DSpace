function [Hdiff] = calculateRotDiff(H,H_old)
    Hdiff = sum(sum(abs(H(1:3,1:3)-H_old(1:3,1:3))));
end