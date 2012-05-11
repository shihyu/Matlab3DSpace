classdef ThreeMarkers <  matlab.mixin.Heterogeneous
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
            normedPoint = (point-reference)/norm(point-reference)+...
                reference;
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
        
        function [Hdiff] = calculateRotDiff(H,H_old)
            Hdiff = sum(sum(abs(H(1:3,1:3)-H_old(1:3,1:3))));
        end
        
        function [points_0] = get0()
            points_0 = ThreeMarkers.points_0;
        end
        
        function plotRun(tm_t)
            %PLOTRUN Plays back a run.
            parallelPlots = size(tm_t,1);
            m=ceil(parallelPlots/2);
            if parallelPlots == 1
                n=1;
            else
                n=2;
            end
            for i = 1:size(tm_t,2)
                for j = 1:parallelPlots
                    subplot(m,n,j);
                    tm_t{j,i}.plotT();
                    grid on
                    axis([-2 2 -2 2 -2 2]);
                end
                drawnow;
            end
        end
        
        function diff = cellminus(obj1,obj2)
            % CELL MINUS Implement obj1 - obj2 for ThreeMarkers: The
            % difference/error between them.
            %display(class(obj1(1)))
           
            %display('Calculating error (vector):')
            diff = cell(1,size(obj1,2));
            parfor i = 1:size(obj1,2)
                diff{i} =ThreeMarkers(quaternionerror(...
                    obj1{i}.getQ,obj2{i}.getQ));
            end     
        end
        
        function [roll,pitch,yaw,diff_t] = getDiff(tm_t1,tm_t2,inDegrees)
            %GETDIFF Same as cellminus() but calculates the roll pitch
            %yaw values during the calculation as well.
            minSize = min(size(tm_t1,2),size(tm_t2,2));
            roll = zeros(1,minSize());
            pitch = zeros(1,minSize());
            yaw = zeros(1,minSize());
            diff_t = cell(1,minSize());
            parfor i=1:minSize
                diff= tm_t1{i}-tm_t2{i};
                euler = diff.getRPY(inDegrees);
                roll(i)=euler(1);
                pitch(i)=euler(2);
                yaw(i)=euler(3);
                diff_t{i}=diff;
            end
        end
        
        function [roll,pitch,yaw,diff_t]= plotDiff(tm_t1,tm_t2,...
                inDegrees,Fs)
            [roll,pitch,yaw,diff_t] = ThreeMarkers.getDiff(tm_t1,...
                    tm_t2,inDegrees);
            minSize = min(size(tm_t1,2),size(tm_t2,2));
            YMAX = max(max(abs(roll)),max(abs(pitch)));
            YMAX = max(YMAX,max(abs(yaw)));
            YMIN=-YMAX;
            if inDegrees
                YLABEL='(degrees)';
            else
                YLABEL='(radians)';
            end
            XLABEL=['1/Fs Fs=' num2str(Fs)];
            t = 0:1/Fs:(minSize-1)/Fs;
            hold on;
            subplot(3,1,1);
            plot(t,roll);
            grid on;
             hold on;
            %ylim([YMIN YMAX])
            title(['ROLL: maximum error: ' num2str(max(abs(roll)))]);
            ylabel(YLABEL);
            %xlabel(XLABEL);
            subplot(3,1,2);
            plot(t,pitch);
            hold on;
            grid on;          
            %ylim([YMIN YMAX])
            title(['PITCH: maximum error: ' num2str(max(abs(pitch)))]);
            ylabel(YLABEL);
            %xlabel(XLABEL);
            subplot(3,1,3);
            plot(t,yaw);
            grid on;
            hold on;
            %ylim([YMIN YMAX])
            title(['YAW: maximum error: ' num2str(max(abs(yaw)))]);
            ylabel(YLABEL);
            xlabel(XLABEL);
        end
        
        function [tm_est] = getChangeOfGlobalReferenceFrames(tm_t_0,...
                tm_t_1,startIndex,numberOfSamples)
            % GETCHANGEOFGLOBALREFERENCEFRAMES Get the estimated change of
            % reference frame matrix.
            tm_t_0 = tm_t_0(:,startIndex:startIndex+numberOfSamples-1);
            tm_t_1 = tm_t_1(:,startIndex:startIndex+numberOfSamples-1);
%             size(tm_t_0)
            p_0 = cell(1,numberOfSamples);
            p_1 = cell(1,numberOfSamples);
            parfor i = 1:numberOfSamples
                p_0{i} = tm_t_0{i}.getT;
                p_1{i} = tm_t_1{i}.getT;
                %p_1(:,(i-1)*4+1:i*4) = tm_t_1(i).getT;
            end
            H_1_0_est = cell2mat(p_0)*pinv(cell2mat(p_1));
            H_1_0_est(1:3,4) = 0;
            H_1_0_est(4,1:3) = 0;
            H_1_0_est(4,4) = 1;
            %display('Estimated H')
            %H_1_0_est
            tm_est = ThreeMarkers(matrix2quaternion(H_1_0_est)');
            
        end
        
        function [metrics] = calculateSyncMetrics(tm_t)
            % CALCULATESYNCMETRICS Calculates the
            % metric used for synchronisation algorithmns.
            tm_t0 = [tm_t(1) tm_t ];
            metrics = zeros(size(tm_t));
            parfor i = 1:size(tm_t,2)
                metrics(i) = ThreeMarkers.calculateRotDiff(...
                    tm_t{i}.getH,...
                    tm_t0{i}.getH);
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
        
        
        function diff = minus(obj1,obj2)
            % MINUS Implement obj1 - obj2 for ThreeMarkers: The
            % difference/error between them, they can also be row vectors.
            %
            %display(class(obj1(1)))
            diff = ThreeMarkers(quaternionerror(...
                    obj1.getQ,obj2.getQ));
        end
         
        function r = ctranspose(obj1)
            % CTRANSPOSE Gets the conjugate.
            r = ThreeMarkers(...
                quaternionconjugate(obj1.getQ)');
        end
        
        function isEqual = eq(obj1,obj2)
            isEqual = (obj1.getQ == obj2.getQ);
        end
        
        function product = times(obj1,obj2)
            %             product = ThreeMarkers(quatnormalize(quatmultiply(obj1.getQ,...
            %                 obj2.getQ)));
            product = ThreeMarkers(...
                quaternionproduct(obj1.getQ,...
                obj2.getQ)');
            
        end
        
        function [tm_2est] = mtimes(...
                tm_est,tm_t2)
            % MTIMES only works if tm_est is a ThreeMarker.
            minSize = size(tm_t2,2);
            tm_2est = cell(1,minSize);
            %display(['IS SCALAR:' num2str(isscalar(tm_est))]);
            if isscalar(tm_est)
                parfor i=1:minSize
                    tm_2est{i} = tm_est.*tm_t2{i};
                end
            else
                error('matlab3Dspace:mtimes',['MTIMES only works for first ThreeMarker as a'...
                    'scalar: size(tm_est): ' num2str(size(tm_est))]);
                    
            end
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
        
        function [euler]=getRPY(tm,inDegrees)
            euler = quaternion2euler(...
                tm.getQ,inDegrees);
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

