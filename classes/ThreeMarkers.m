classdef ThreeMarkers < handle
    %THREEMARKERS Base class that builds homogenous matrixs,
    % quaternions, global reference frame and plot functions
    % for 3D data.
    
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
        quaternion = [1 0 0 0];
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
            errorQuat =  quatnormalize(quatmultiply(q1,...
                quatconj(q2)));
            errorEuler = invrpy(quaternion2matrix(errorQuat));
        end
        
        function [Hdiff] = calculateRotDiff(H,H_old)
            Hdiff = sum(sum(abs(H(1:3,1:3)-H_old(1:3,1:3))));
        end
        
        function [points_0] = get0()
            points_0 = ThreeMarkers.points_0;
        end
        
        function plotRun(tm_t)
            %PLOTRUN Plays back a run.
            for i = 1:size(tm_t,2)
                tm_t(i).plotT();
                grid on
                axis([-2 2 -2 2 -2 2]);
                drawnow;
            end
        end
        
        function [metrics] = calculateSyncMetrics(tm_t)
            tm_t0 = [tm_t(1) tm_t ];
            metrics = zeros(size(tm_t));
            parfor i = 1:size(tm_t,2)
                metrics(i) = ThreeMarkers.calculateRotDiff(...
                    tm_t(i).getH,...
                    tm_t0(i).getH);
            end
        end
    end
    
    methods
        function qtm = ThreeMarkers(quaternion)
            qtm.H_0_T = quaternion2matrix(quaternion);
            qtm.quaternion = quaternion;
            qtm.points_T = qtm.H_0_T*qtm.points_0;
            qtm.timestamp = 0.0;
        end
        
        
        
        function r = minus(obj1,obj2)
            % MINUS Implement obj1 - obj2 for Quaternions: The
            % difference/error between them.
            r = ThreeMarkers(ThreeMarkers.quaternionerror(...
                obj1.getQ,obj2.getQ));
        end
        
        function r = ctranspose(obj1)
            % CTRANSPOSE Gets the conjugate.
            r = ThreeMarkers(quatnormalize(quatconj(obj1.getQ)));
        end
        
        function isEqual = eq(obj1,obj2)
            isEqual = (obj1.getQ == obj2.getQ);
        end
        
        function product = times(obj1,obj2)
            size(obj1.getQ)
            product = ThreeMarkers(quatnormalize(quatmultiply(obj1.getQ,...
                obj2.getQ)));
        end
        
        function showQ = display(tm)
            showQ = tm.getQ;
        end
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
        
        function plotT(tm)
            tm.plot(tm.points_T,'--k');
        end
        function plot0(tm)
            tm.plot(tm.points_0,'--m');
        end
        
        function [quat] = toNumeric(tm)
            quat = [ tm.getQ() tm.getTimeStamp ];
        end
    end
end

