function [ points_psi_v,points_psi_0,angles] = getViconCoordinates( rightback,...
    leftback,...
    front)
%GETVICONCOORDINATES Summary of this function goes here
%   Detailed explanation goes here

midpoint = (rightback+leftback)/2;
crosspoint = cross(leftback-midpoint,front-midpoint)+midpoint;

angles = [];
angles(1) = acos(dot(crosspoint-midpoint,...
    front-midpoint)/(norm(crosspoint-midpoint)*norm(front-midpoint)))*180/pi;
angles(2) = acos(dot(crosspoint-midpoint,...
    leftback-midpoint)/(norm(crosspoint-midpoint)*norm(leftback-midpoint)))*180/pi;
angles(3) = acos(dot(crosspoint-midpoint,...
    rightback-midpoint)/(norm(crosspoint-midpoint)*norm(rightback-midpoint)))*180/pi;
angles(4) = acos(dot(front-midpoint,...
    leftback-midpoint)/(norm(front-midpoint)*norm(leftback-midpoint)))*180/pi;
angles(5) = acos(dot(front-midpoint,...
    rightback-midpoint)/(norm(front-midpoint)*norm(rightback-midpoint)))*180/pi;

points_psi_v = [rightback/norm(rightback-midpoint);
    leftback/norm(leftback-midpoint);
    front/norm(front-midpoint);
    crosspoint/norm(crosspoint-midpoint)];

%Constructed point in the zero reference frame.
points_psi_0 = [1 0 0;
    -1 0 0;
    0 1 0;
    0 0 1];

%not a perfect 60 degree Triangle... so rotate on the XY plane to get the
%actual front marker in the zero frame.
rotAngle = acos(dot(front-midpoint,...
    leftback-midpoint)/(norm(front-midpoint)*norm(leftback-midpoint)))
yValue = points_psi_0(1,1:2)*[cos(rotAngle) -sin(rotAngle); sin(rotAngle) cos(rotAngle)]
points_psi_0(3,1:2) = yValue;

%Create the screw theory compliant points.
points_psi_0(:,4) = 1;
points_psi_v(:,4) = 1;

points_psi_0 = points_psi_0';
points_psi_v = points_psi_v';
end

