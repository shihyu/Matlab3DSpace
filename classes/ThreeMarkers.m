classdef ThreeMarkers < handle
    %THREEMARKERS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        points_0 = [
            1 -1 0 0;
            0 0 1 0;
            0 0 0 1;
            1 1 1 1];
    end
    properties (Access = protected)
        timestamp = 0;
        points_T = zeros(4);
        H_0_T = zeros(4);
        quaternion = [0 0 0 0];
    end
    methods (Access = protected,Static)
        function [normedPoint] = normWithOffset(point,reference)
            normedPoint = (point-reference)/norm(point-reference)+reference;
        end
        
        function plot(point,style)
            plotSensor(point,style);
        end
        
    end
    methods (Static)
        function angle = getAngle(marker1,marker2,zeropoint)
            angle = acos(dot(marker1-zeropoint,...
                marker2-zeropoint)/...
                (norm(marker1-zeropoint)*norm(marker2-zeropoint)));
        end
        
        function [ angDiff ] = angleDifference(x,y)
            %ANGLEDIFFERENCE Summary of this function goes here
            %   Detailed explanation goes here
            angDiff = atan2(sin(x-y), cos(x-y));
        end
        
        function [ errorQuat,errorEuler ] = quaternionerror(q1,q2)
            %QUATERNIONERROR Calculates Error between two Quaternions.
            errorQuat =  quaternionproduct(q1,...
                quaternionconjugate(q2));
            errorEuler = invrpy(quaternion2matrix(errorQuat));
        end
        
        function [Hdiff] = calculateRotDiff(H,H_old)
            Hdiff = sum(sum(abs(H(1:3,1:3)-H_old(1:3,1:3))));
        end
        
        function [points_0] = get0()
            points_0 = ThreeMarkers.points_0;
        end
    end
    
    methods
        function [timestamp] = getTimeStamp(tm)
            timestamp = tm.timestamp;
        end
        
        function [points_T] = getT(tm)
            points_T = tm.points_T;
        end
        
        function [H_0_T] = getH(tm)
            H_0_T = tm.H_0_T;
        end
        
        function [quaternion]=getQ(tm)
            quaternion = tm.quaternion;
        end
        
        function [rotmat] = getRotationMatrix(tm)
            rotmat = tm.H_0_T(1:3,1:3);
        end
        
        function plot_T(tm)
            tm.plot(tm.points_T,'--k');
        end
        function plot_0(tm)
            tm.plot(tm.points_0,'--m');
        end
    end
end

