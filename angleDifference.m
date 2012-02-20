function [ angDiff ] = angleDifference(x,y)
%ANGLEDIFFERENCE Summary of this function goes here
%   Detailed explanation goes here
    angDiff = atan2(sin(x-y), cos(x-y));
end

