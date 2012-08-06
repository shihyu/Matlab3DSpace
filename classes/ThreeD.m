classdef ThreeD <  matlab.mixin.Heterogeneous
    %THREEMARKERS Base class that builds homogenous matrixs,
    % quaternions, global reference frame and plot functions
    % for 3D data.
    
    properties (Constant)
        %The Format:[ rightback;
                %leftback;
                %front;
                %crosspoint]
        points_0 = [
            0 0 1 0;
            -1 1 0 0;
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
               
        function plot3DPoint(point,style)
            plotSensor(point,style);
        end
        
        function plotAngle(t,data,type,theLabel)
        %PLOTANGLE Used in th plotRPY function.
          if type==0
                plot(t,data);
            elseif type==1
                stem(t,data);
            else
                ts = timeseries(data,t);
                ts.TimeInfo.Units = theLabel;
                ts.plot('--mo','MarkerSize',5);
          end
        end
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %STATIC FUNCTIONS: Operate using ThreeD.theFunctionName...
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods (Static)
        function [normedPoint] = normWithOffset(point,reference)
            %NORMWITHOFFSET Calculates and normalises the point with the
            %reference vector.
            
            
            %Make sure you do not divide by zero.
            if abs(point-reference) > repmat(eps, size(point))
                %display('Standard');
                normedPoint = (point-reference)/norm(point-reference)+...
                    reference;
            %Reference point is zero
            elseif (all(reference < repmat(eps, size(reference)))) && ...
                    (all(point > repmat(eps, size(point))))
                %display('Reference zero');
                normedPoint = point/norm(point);
            %Point is zero.
            else
                %display('Point Zero');
                normedPoint = point;                
            end
        end
        
        function [t,tm_t] = sortAccordingToTimestamp(t,tm_t)
            %SORTS the run according to the timestamps.
            %TESTS NEED TO BE WRITTEN.
            [t,indexes] = sort(t);
            tm_t = tm_t(indexes);
        end
        
        function angle = getAngle(marker1,marker2,zeropoint)
            angle = acos(dot(marker1-zeropoint,...
                marker2-zeropoint)/...
                (norm(marker1-zeropoint)*norm(marker2-zeropoint)));
        end
        
        function [ angDiff ] = angleDifference(x,y)
            %ANGLEDIFFERENCE Get difference between two angles.
            angDiff = atan2(sin(x-y), cos(x-y));
        end
        
        function [Hdiff] = calculateRotDiff(H,H_old)
            %CALCULATEROTDIFF Calculates a simple
            %metric to use for syncrhonisation.
            Hdiff = sum(sum(abs(H(1:3,1:3)-H_old(1:3,1:3))));
        end
        
        function [points_0] = get0()
            %GET0 Gets the zero Frame (global reference frame)
            %used for this sample.
            points_0 = ThreeD.points_0;
        end
        
        function plotRun(tm_t,varargin)
            %PLOTRUN Plays back a run
            %tm_t must be a row cell vector. If you would like to
            %plot values side by side then create a matrix of
            %row cell vectors, with each row being a run to plot.
            %Optional argument of a number can be used to make the plot
            %pauze between plots if you want to get a better understanding
            %of what is happening.
           
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
                    if (~isempty(varargin))&&(~isempty(varargin{1}))
                        pause(varargin{1});
                    end
                end
                drawnow;
            end
        end
        
        function diff = cellminus(obj1,obj2)
            % CELL MINUS Implement obj1 - obj2 for ThreeD when using
            % cells.
            %display(class(obj1(1)))
            
            %display('Calculating error (vector):')
            minSize = min(size(obj1,2),size(obj2,2));
            diff = cell(1,minSize);
            parfor i = 1:minSize
                diff{i} = obj1{i}-obj2{i};
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
        
        function [roll_t,pitch_t,yaw_t,t] = getRPYtInit(tm_t)
            %GETRPYTINIT(tm_t)
            %Initialise the GETRPYT function. This is used if you
            %would like to create your own parforloop to
            %process the data. Normally just use getRPYt(tm_t,inDegrees)
            minSize = size(tm_t,2);
            roll_t = zeros(1,minSize());
            pitch_t = zeros(1,minSize());
            yaw_t = zeros(1,minSize());
            t = zeros(1,minSize());
        end
        
        function [roll_i,pitch_i,yaw_i]=getRPYtProcess(tm_t,inDegrees,i)
            %GETRPYTPROCESS(tm_t)
            %Processing step for the GETRPYT function. This is used if you
            %would like to create your own parforloop to
            %process the data. Normally just use getRPYt(tm_t,inDegrees)
            euler = tm_t{i}.getRPY(inDegrees);
            roll_i=euler(1);
            pitch_i=euler(2);
            yaw_i=euler(3);
        end
        
        function [roll_t,pitch_t,yaw_t,t] = getRPYt(tm_t,inDegrees)
            %GETRPYT(tm_t,inDegrees) Get the Roll, Pitch and Yaw
            %of the run tm_t and set inDegrees to true if you
            %would like to see the results in degrees. False for radians.
            [roll_t,pitch_t,yaw_t,t] = ThreeD.getRPYtInit(tm_t);
            minSize = size(tm_t,2);
            parfor i=1:minSize
                euler = tm_t{i}.getRPY(inDegrees);
                roll_t(i)=euler(1);
                pitch_t(i)=euler(2);
                yaw_t(i)=euler(3);
                t(i) = tm_t{i}.getTimestamp();
            end
            %Sort data in ascending orders.
            t_orig = t;
            [t,roll_t] = ThreeD.sortAccordingToTimestamp(t,roll_t);
            [t,pitch_t] = ThreeD.sortAccordingToTimestamp(t_orig,...
                pitch_t);
            [t,yaw_t] = ThreeD.sortAccordingToTimestamp(t_orig,...
                yaw_t);
        end
        
        function plotRPY(roll,pitch,yaw,inDegrees,Fs,varargin)
            %PLOTRPY(roll,pitch,yaw,inDegrees,Fs,t,type)
            %Type
            %PLOT the Roll Pitch and Yaw for the run.
            YMAX = max(max(abs(roll)),max(abs(pitch)));
            YMAX = max(YMAX,max(abs(yaw)));
            YMIN=-YMAX;
            typeOfPlot=1;
            if length(varargin)>0
                t = varargin{1};
                if length(varargin)==2
                    typeOfPlot=varargin{2};
                end
            else
                minSize = length(roll);
                t = 0:1/Fs:(minSize-1)/Fs;
                typeOfPlot=0;
            end
            if inDegrees
                YLABEL='(degrees)';
            else
                YLABEL='(radians)';
            end
            XLABEL=['1/Fs Fs=' num2str(Fs)];
            XMIN = min(t);
            XMAX = max(t);
            
            hold on;
           
            subplot(3,1,1);
            ThreeD.plotAngle(t,roll,typeOfPlot,YLABEL)
            hold on;
            grid on;
            xlim([XMIN XMAX])
            title(['ROLL(y): maximum angle: ' num2str(max(abs(roll)))]);
            ylabel(YLABEL);
            subplot(3,1,2);
            ThreeD.plotAngle(t,pitch,typeOfPlot,YLABEL)
            grid on;
            hold on;
            xlim([XMIN XMAX])
            title(['PITCH(x): maximum angle: ' num2str(max(abs(pitch)))]);
            ylabel(YLABEL);
            xlabel(XLABEL);
            subplot(3,1,3);
            ThreeD.plotAngle(t,yaw,typeOfPlot,YLABEL)
            grid on;
            hold on;
            xlim([XMIN XMAX])
            title(['YAW(z): maximum angle: ' num2str(max(abs(yaw)))]);
            ylabel(YLABEL);
        end
        
        function [roll,pitch,yaw,t] = getAndPlotRPYt(theRun_t,theTitle,...
                varargin)
            %GETANDPLOTRYP Gets and plots the RPY for the run.
            [roll,pitch,yaw,t] = ThreeD.getRPYt(theRun_t,true);   
            figure('visible','on','WindowStyle','docked',...
                'Name',theTitle);
            ThreeD.plotRPY(roll,pitch,yaw,true,200,t,varargin{:});
        end
        
        function [tm_est] = getChangeOfGlobalReferenceFrames(tm_t_0,...
                tm_t_1,startIndex,numberOfSamples)
            % GETCHANGEOFGLOBALREFERENCEFRAMES(tm_t_0,tm_t_1,startIndex,
            %numberOfSamples) Get the estimated change of
            % reference frame ThreeMarker object.
            %tm_t_0 - The base measurements which you would like to use
            %to get the estimated ThreeD object with respect to.
            %tm_t_1 - the measurements that you would like to get
            %the change of reference frame for.
            %startIndex - where in the data you would like to use to
            %estimate the change.
            %numberOfSamples - the number of samples from the startIndex
            %that you would like to use for the estimation.
            %RETURN tm_est - The ThreeD object that represents
            % the change from tm_t_1 to tm_t0. Thus use it as tm_est*tm_t_1
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
            tm_est = ThreeD(matrix2quaternion(H_1_0_est));
            
        end
        
        function [tm_est] = ...
                callibrateInit(tm_t,startIndex,numberOfSamples)
            %CALLIBRATEINIT Init loop for callibrate loop
            N=numberOfSamples+startIndex;
            zeroRun = cell(1,N);
            parfor i = 1:N
                zeroRun{i} = ThreeD([1 0 0 0]);
            end
            tm_est = getChangeOfGlobalReferenceFrames(zeroRun,...
                tm_t,startIndex,numberOfSamples);
        end
        
        function [tm_t_c] = ...
                callibrate(tm_t,startIndex,numberOfSamples)
            %CALLIBRATE Changes the samples to the zero frame using
            %an estimate of the initial orientation from the start of
            %the run where the sensor was held still in a known
            %direction.
            %
            %You can then callibrate the data. It takes the 'average'
            %quaternion of the numberOfSamples quaternions starting at
            %startIndex samples
            %into the experiment and then uses this as the zero frame.
            %All the samples in the set are then inversely rotated by
            %this estimated quaternion.
            %
            % Returns the tm_t(startIndex:length(tm_t)).
            tm_est = ThreeD.callibrateInit(...
                tm_t,startIndex,numberOfSamples);
            tm_t_c = tm_t;
            parfor i =  startIndex:size(tm_t,2)
                tm_t_c{i} = tm_est.*tm_t{i};
            end
            tm_t_c = tm_t_c(startIndex:length(tm_t_c));
        end
        
        function [metrics] = calculateSyncMetrics(tm_t)
            % CALCULATESYNCMETRICS Calculates the simple
            % metric used for synchronisation algorithmns.
            tm_t0 = [tm_t(1) tm_t ];
            metrics = zeros(size(tm_t));
            parfor i = 1:size(tm_t,2)
                metrics(i) = ThreeD.calculateRotDiff(...
                    tm_t{i}.getH,...
                    tm_t0{i}.getH);
            end
        end
        
        function [tm_t,t] = resample(tm_t,Fs_wanted,Fs_current)
            %RESAMPLE Resamples a cell of ThreeD from the Fs_current
            %to the Fs_wanted.
            quats = zeros(length(tm_t),4);
            parfor i = 1:length(tm_t)
                quats(i,:) = tm_t{i}.getQ;
            end
%             quats
            quats = resample(quats,Fs_wanted,Fs_current);
%             quats
            N = size(quats,1);
            tm_t = cell(1,N);
            t = 0:1/Fs_wanted:(N-1)/Fs_wanted;
            parfor i = 1:N
%                 i
%                 quats(i,:)
                theQuat = ThreeD(quats(i,:));
                theQuat = theQuat.setTimestamp(t(i));
                tm_t{i} = theQuat;
            end
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %METHODS FROM BELOW HERE: Operate only on the objects!
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        function qtm = ThreeD(quaternionOrH)
            %THREEMARKER(quaternion) Creates the ThreeMarker object
            % which takes a 4x1 or 1x4 quaternion vector as input.
            if size(quaternionOrH) == [1,4]
                qtm.H_0_T = quaternion2matrix(quaternionOrH);
                qtm.quaternion = quaternionnormalise(quaternionOrH);
            elseif size(quaternionOrH) == [4,4]
                %Inconsistent use of order of multiplication found
                %somewhere in the code, between H and quaternions.
                qtm.H_0_T = quaternionOrH;
                %Not sure why but for the H sent from ViconThreeD
                %we need to get the quaternion from the inverse matrix.
                qtm.quaternion = matrix2quaternion(quaternionOrH');
            else
                error('matlab3dspace:threemarkers',...
                    ['Wrong size for input quaternion or H matrix' ...
                    mat2str(size(quaternionOrH))...
                    ' should be 1x4 or 4x4' ]);
            end
            qtm.points_T = qtm.H_0_T*qtm.points_0;
            qtm.timestamp = 0.0;            
        end
        
        
        function diff = minus(obj1,obj2)
            % MINUS Implement obj1 - obj2 for ThreeD: The
            % difference/error between them, they can also be row vectors.
            % Keeps the first ones time stamp.
            
            %display(class(obj1(1)))
            diff = ThreeD(quaternionerror(...
                obj1.getQ,obj2.getQ));
            %             display(['The timestamp: ' theTimestamp]);
            diff = diff.setTimestamp(obj1.getTimestamp);
        end
        
        function r = ctranspose(obj1)
            % CTRANSPOSE Operator Gets the quaternion conjugate.
            r = ThreeD(...
                quaternionconjugate(obj1.getQ));
            r = r.setTimestamp(obj1.getTimestamp);
        end
        
        function isEqual = eq(obj1,obj2)
            % EQUAL Operator Objects are equal if their quaternions
            %are equal.
            isEqual = (obj1.getQ == obj2.getQ);
        end
        
        function product = times(obj1,obj2)
            %SCALAR multiplication operator
            %Time stamp of second argument is maintained.
            product = ThreeD(...
                quaternionproduct(obj1.getQ,...
                obj2.getQ)');
            product = product.setTimestamp(obj2.getTimestamp());
            
        end
        
        function [tm_2est] = mtimes(...
                tm_est,tm_t2)
            % MTIMES MATRIX MULIPLICATION OPERATOR only works if tm_est
            %is a scalar ThreeMarker.
            minSize = size(tm_t2,2);
            tm_2est = cell(1,minSize);
            %display(['IS SCALAR:' num2str(isscalar(tm_est))]);
            if isscalar(tm_est)
                parfor i=1:minSize
                    tm_2est{i} = tm_est.*tm_t2{i};
                end
            else
                error('matlab3Dspace:mtimes',...
                    ['MTIMES only works for first ThreeMarker as a'...
                    'scalar: size(tm_est): ' num2str(size(tm_est))]);
                
            end
        end
        
        function showQ = display(tm)
            %THE DEFAULT DIPLAY FOR THREEMARKERS OBJECT (Quaternion Shown)
            showQ = [ tm.getQ() tm.getTimestamp ];
        end
        function [timestamp] = getTimestamp(tm)
            %GETTIMESTAMP(tm) Gets the time stamp for the THREMARKERS
            %OBJECT
            timestamp = tm.timestamp;
        end
        
        function [tm] = setTimestamp(tm,timestamp)
            %SETTIMESTAMP(tm) Sets the time stamp for the THREMARKERS
            %OBJECT
            tm.timestamp = timestamp;
        end
        
        function [points_T] = getT(tm)
            %GETT(tm) gets the repesentation of the 0-frame rotated by the
            %quaternion.
            points_T = tm.points_T;
        end
        
        function [H_0_T] = getH(tm)
            %GETH(tm) Gets the Homogenous (Screw theory) rotation matrix of
            %that represents the quaternion.
            H_0_T = tm.H_0_T;
        end
        
        function [euler]=getRPY(tm,inDegrees)
            %GETRPY(tm,inDegrees) Gets the Roll Pitch and Yaw of the
            %object, set inDegrees to true to get the values in degrees.
            euler = quaternion2euler(tm.getQ,inDegrees,'xyz');
%             euler = invrpy(tm.getH);
%             if inDegrees
%                 euler = euler/pi*180;
%             end
        end
        
        function [quaternion]=getQ(tm)
            %GETQ(tm) Get the quaternion for the object.
            quaternion = tm.quaternion;
        end
        
        function [rotmat] = getRotationMatrix(tm)
            %GETROTATIONMATRIC Get the rotation matrix for the object.
            rotmat = tm.H_0_T(1:3,1:3);
        end
        
        function plotT(tm)
            %PLOTT(tm) Plot the zero frame rotated by the rotation matrix,
            %ie the object at T.
            tm.plot3DPoint(tm.points_T,'--k');
        end
        function plot0(tm)
            %PLOT0(tm) Plot the zero frame.
            tm.plot3DPoint(tm.points_0,'--m');
        end
        
        function [quat] = toNumeric(tm)
            %TONUMERIC(tm) How to display the object numerically.
            quat = [ tm.getQ() tm.getTimestamp ];
        end
    end
end

