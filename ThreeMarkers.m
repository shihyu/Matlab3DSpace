classdef ThreeMarkers < handle
    %THREEMARKERS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        points_psi_0 = [
            1 -1 0 0;
            0 0 1 0;
            0 0 0 1;
            1 1 1 1];
    end
    properties
        timestamp = 0;
        points_psi_v = zeros(4);
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
        
        function [ errorQuat,errorEuler ] = quaternionerror( q1,q2 )
            %QUATERNIONERROR Calculates Error between two Quaternions.
            errorQuat =  quaternionproduct(q1,...
                quaternionconjugate(q2));
            errorEuler = invrpy(quaternion2matrix(errorQuat));
        end
    end
    
    methods
        function [points_0] = getZeroFrame(tm)
            points_0 = tm.points_psi_0;
        end
        
        function [H_0_T] = getHMatrix(tm)
            H_0_T = tm.H_0_T
        end
        
        function [quaternion]=getQuaternion(tm)
            quaternion = tm.quaternion
        end
        
        function [rotmat] = getRotationMatrix(tm)
            rotmat = tm.H_0_T(1:3,1:3);
        end
    end
end

