function [ zeroframe ] = getPromoveZeroFrame()
%GETPROMOVEZEROFRAME gets the Promve Zero Frame.

%   The Vicon points sit here.
%        cvt.points_psi_v = [ cvt.rightback;
%                             cvt.leftback;
%                             cvt.front;
%                             cvt.crosspoint];

%Constructed point in the zero reference frame.
zeroframe = [1 0 0 1;
    -1 0 0 1;
    0 -1 0 1;
    0 0 -1 1]';

end

