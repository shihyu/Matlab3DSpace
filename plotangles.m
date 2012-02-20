function plotangles(t,first,second,theTitle )
%PLOTANGLES Summary of this function goes here
%   Detailed explanation goes here
hold on;
plot(t,first,'k-')
if size(second) == size(first)
    plot(t,second,'m-')
end
grid on;
if size(second) == size(first)
    ylim([-2*pi 2*pi])
end
title(theTitle);
end

