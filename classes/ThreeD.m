classdef ThreeD <  matlab.mixin.Heterogeneous
    %THREEMARKERS Base class that builds homogenous matrixs,
    % quaternions, global reference frame and plot functions
    % for 3D data.
    %
    %It does not use translational data at this stage.
    
    properties (Constant)
        %The Format:[ rightback;
        %leftback;
        %front;
        %crosspoint]
        %This is the default layout for the three marker system.
        default_points_0 = ...
            [
            0 0 1 0;
            -1 1 0 0;
            0 0 0 1;
            1 1 1 1
            ];
        %This is the layout when the leftback and rightback are found on
        %the x-axis.
        x_base_frame = ...
            [
            1 -1 0 0;
            0 0 1 0;
            0 0 0 1;
            1 1 1 1
            ];
    end
    properties (Access = protected)
        %The Format:[ rightback;
        %leftback;
        %front;
        %crosspoint]
        points_0 = ThreeD.default_points_0;
        timestamp = 0;
        points_T = zeros(4);
        H_0_T = zeros(4);
        quaternion = [1 0 0 0];
    end
    methods (Access = protected,Static)
        
        function plot3DPoint(point,style)
            plotSensor(point,style);
        end
        
        function plotAngle(t,data,plotType,theLabel,plotStyle)
            %PLOTANGLE Used in the plotRPY function.
            %t is the time to plot against, data is the data and plotType
            %is either 'normal' for plot, 'stem' for stem plot or
            %'timeseries' for timeseries plot.
            if strcmp(plotType,'normal')==1
                plot(t,data);
            elseif strcmp(plotType,'stem')==1
                stem(t,data);
            elseif strcmp(plotType,'timeseries')==1
                ts = timeseries(data,t);
                ts.TimeInfo.Units = theLabel;
                ts.plot(plotStyle,'MarkerSize',5);
            else
                error('ThreeD:plotAngleTypeNotCorrect',['The plotType'...
                    ' should be either stem, normal or timeseries']);
            end
        end
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %STATIC FUNCTIONS: Operate using ThreeD.theFunctionName...
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods (Static)
        function [one_t,two_t] = synchroniseWithRespectToRPY(varargin)
            %SYNCHRONISEWITHRESPECTTORPY
            %Looks at the roll,pitch and yaw for each of the sensors
            %and then synchronises with the one that has the hightest
            %correlation using the roll, pitch or yaw.
            p = inputParser;
            p.addParamValue('roll_one',-1,@isvector);
            p.addParamValue('pitch_one',-1,@isvector);
            p.addParamValue('yaw_one',-1,@isvector);
            p.addParamValue('one_t',-1,@isvector);
            p.addParamValue('roll_two',-1,@isvector);
            p.addParamValue('pitch_two',-1,@isvector);
            p.addParamValue('yaw_two',-1,@isvector);
            p.addParamValue('two_t',-1,@isvector);
            p.addParamValue('Fs_plot',-1,@isscalar);
            parse(p,varargin{:});
            
            roll_one= ...
                p.Results.roll_one ;
            pitch_one         = ...
                p.Results.pitch_one ;
            yaw_one        = ...
                p.Results.yaw_one ;
            one_t         = ...
                p.Results.one_t ;
            roll_two         = ...
                p.Results. roll_two;
            pitch_two         = ...
                p.Results.pitch_two ;
            yaw_two         = ...
                p.Results.yaw_two ;
            two_t         = ...
                p.Results.two_t ;
            Fs_plot        = ...
                p.Results.Fs_plot;
            
            
            [one_t_r,two_t_r,rollMax] = ThreeD.synchronise(...
                'metric_1',roll_one,...
                'metric_2',roll_two,...
                'data_1',one_t,...
                'data_2',two_t,...
                'Fs',Fs_plot,...
                'theStart',1);
            [one_t_p,two_t_p,pitchMax] = ThreeD.synchronise(...
                'metric_1',pitch_one,...
                'metric_2',pitch_two,...
                'data_1',one_t,...
                'data_2',two_t,...
                'Fs',Fs_plot,...
                'theStart',1);
            [one_t_y,two_t_y,yawMax] = ThreeD.synchronise(...
                'metric_1',yaw_one,...
                'metric_2',yaw_two,...
                'data_1',one_t,...
                'data_2',two_t,...
                'Fs',Fs_plot,...
                'theStart',1);
            totalMax = max([rollMax pitchMax yawMax]);
            if rollMax == totalMax
                display('SYNCING ON ROLL ANGLE');
                one_t = one_t_r;
                two_t = two_t_r;
            elseif pitchMax == totalMax
                display('SYNCING ON PITCH ANGLE');
                one_t = one_t_p;
                two_t = two_t_p;
            else
                display('SYNCING ON YAW ANGLE');
                one_t = one_t_y;
                two_t = two_t_y;
            end
            display('FINISHED SYNCING WITH RESPECT TO RPY');
        end
        
        function [ data_1,data_2,corrValue] = synchronise(varargin)
            %SYNCHRONISE Performs simple cross-correlation synchronisation between two
            %signals using the metric_1 and metric_2 as the correlation signal.
            %if the metricIsAlsoStartOfExperiment is true then both experiments
            %are shifted to start at the first non-zero element of the metric
            %parameters.
            p = inputParser;
            p.addParamValue('metric_1',-1,@isvector);
            p.addParamValue('metric_2',-1,@isvector);
            p.addParamValue('data_1',-1,@isvector);
            p.addParamValue('data_2',-1,@isvector);
            p.addParamValue('Fs',-1,@isscalar);
            p.addParamValue('theStart',-1,@isscalar);
            addOptional(p,'metricIsAlsoStartOfExperiment',false);
            parse(p,varargin{:});
            
            metricIsAlsoStartOfExperiment = ...
                p.Results.metricIsAlsoStartOfExperiment;
            theStart = ...
                p.Results.theStart;
            Fs= ...
                p.Results.Fs;
            metric_1= ...
                p.Results.metric_1;
            metric_2= ...
                p.Results.metric_2;
            data_1= ...
                p.Results.data_1;
            data_2= ...
                p.Results.data_2;
            
            if metricIsAlsoStartOfExperiment
                shift1 = find(metric_1);
                shift2 = find(metric_2);
                shift1 = shift1(1);
                shift2 = shift2(1);
                metric_1 = metric_1(shift1:length(metric_1));
                metric_2 = metric_2(shift2:length(metric_2));
                data_1 = data_1(shift1:length(data_1));
                data_2 = data_2(shift2:length(data_2));
            end
            
            %display('Syncrohonising:');
            N1 = size(data_1,2);
            N2 = size(data_2,2);
            N=max(N1,N2);
            
            %Synchronise over a subset of the matrix's
            metric_1 = metric_1(theStart:N1);
            metric_2 = metric_2(theStart:N2);
            
            
            N = N-(theStart-1);
            N1 = N1-(theStart-1);
            N2 = N2-(theStart-1);
            t = [0:1/Fs:N/Fs-1/Fs];
            figRPYAR_IMU_sync=figure('visible','on','WindowStyle','docked',...
                'Name','SYNCHRONISE');
            subplot(2,1,1);
            hold on;
            plot(t(1:N1),metric_1(1:N1),'--k');
            plot(t(1:N2),metric_2(1:N2),'--m');
            title('ORIGINAL METRICS:');
            grid on;
            
            subplot(2,1,2);
            hold on;
            %Get correlation.
            [R12] = xcorr(metric_1',metric_2');
            [corrValue,max_lags_12]=max(R12);
            % display(['SIZE correlation: R12:' num2str(size(R12,1))...
            %     ' MAX12:' num2str(max_12)  ...
            %     ' MAXLAGS12:' num2str(max_lags_12) ' N:' num2str(N)])
            %Adjust input data only if it needs to be shifted.
            if (max_lags_12 == 1)
                corrValue = 0;
                return
            elseif (max_lags_12 < N)
                Nstart = N-max_lags_12+1;
                theTitle = ['Synchronising: ONE(Black) ahead of TWO(Magenta): Shifting:' ...
                    num2str(Nstart)];
                %display(theTitle);
                data_2 = data_2(Nstart:size(data_2,2));
                metric_2 = metric_2(Nstart:size(metric_2,2));
                
                N2 = size(metric_2,2);
                
            else
                Nstart = max_lags_12-N+1;
                theTitle = ['Synchronising: TWO(Magenta) ahead of ONE(Black): Shifting:' ...
                    num2str(Nstart)];
                %display(theTitle);
                data_1 = data_1(Nstart:size(data_1,2));
                metric_1 = metric_1(Nstart:size(metric_1,2));
                N1 = size(metric_1,2);
            end
            grid on;
            % Threshold = max(metric_1(theStart:theStart+numberOfSamples));
            %
            % newStart = find(metric_1>Threshold*2,1);
            % newStart = newStart -100;
            % if newStart <0
            %     newStart = 0;
            % end
            %
            % data_1 = data_1(newStart:size(data_1,2));
            % data_2 = data_2(newStart:size(data_2,2));
            %
            plot(t(1:N1),metric_1,'--k')
            plot(t(1:N2),metric_2,'--m')
            N=min(length(metric_1),length(metric_2));
            [R12] = xcorr(metric_1(1:N)',metric_2(1:N)','coeff');
            [corrValue]=max(R12);
            title(theTitle)
        end
            
        function [t_new,tm_t_new]= setStartTime(t,tm_t, startTime, endTime, Fs)
            % SETSTARTTIME Sets the starttime of an object
            % Can be used to make objects of different sensors the same
            % length
            t_step = 1/Fs;
            integerTest = startTime/t_step;
            integerTest2 = endTime/t_step;
            if ~mod(integerTest,1) ==0 || ~mod(integerTest2,1) ==0
                    warning('starting or ending sample is not dividable by the time step')
            end
            [b, startSample] = min(abs(t - startTime));
            [b, endSample] = min(abs(t - endTime));
            
            if isempty(startSample) || isempty(endSample)
                warning('startTime or endTime is not in the timestamps')
            end
            
            t_new = t(startSample:endSample);
            tm_t_new = tm_t(startSample:endSample);
        end
        
        function [jointAngle_t] = ...
                getjointangles(sensor1_t,sensor2_t,angleName,...
            callibrateStart,varargin)
            p = inputParser;
            p.addOptional('doPlot',false);
            p.parse(varargin{:});
            
            doPlot= ...
                p.Results.doPlot ;
            %GETJOINTANGLES Calculates the angles between two sensors. Sensor 1 is
            %the base sensor. Anglename is a name, so should be given
            %between '.
            display(['%%%%%%%%%%%%%%Processing Angle:' angleName ' %%%%%%%%%%%%%%'])
            sensor2Style = '--b.';
            sensor1Style = '--r.';
            jointAnglePlotStyle = '--m.';
            %ThreeD.plotRun([sensor1_t;sensor2_t]);
            figures =[];
            if doPlot
                ThreeD.getAndPlotRPYt(sensor2_t,...
                    ['GETJOINTANGLES: Uncalibrated original sensor position: Sensor 1(' sensor1Style...
                    ') Sensor 2 (' sensor2Style ')' ],false,'plotStyle',sensor2Style);
                ThreeD.getAndPlotRPYt(sensor1_t,...
                    '',unCallibratedOrignalFigure,'plotStyle',sensor1Style);
            end
            %joint Angle:
            % psiS = Steering Column Frame
            % psiR = Roll sensor frame, on the bicycle frame.
            % psi0 = Origin frame
            % For a point q
            % q_psi0 = H_S0*q_psiS => We Have, sensor2_t
            % q_psi0 = H_R0*q_psiR => We have, sensor1_t
            % q_psiR = HSR*q_psiS => We want H12 => Now lets solve:
            % H_SR =  (H_R0)^-1*HS0
            %In code: sensor2_t = H_RS and sensor1_t = H_R0
            % Thus joint_angle = H_SR
            % jointAngle_t = sensor1_t'*sensor2_t
            jointAngle_t = ThreeD.cellInverseMultiply(sensor1_t,sensor2_t);
            if doPlot
                ThreeD.getAndPlotRPYt(jointAngle_t,...
                    ['GETJOINTANGLES: Calculated '  angleName '(un-callibrated).' ],...
                    false,'plotStyle',jointAnglePlotStyle);
            end
            %Callibrate to see what happens! We want to make this frame the origin...
%             jointAngle_t = ThreeD.zeroTheRun(jointAngle_t,callibrateStart,30);
%             if doPlot
%                 ThreeD.getAndPlotRPYt(jointAngle_t,...
%                     ['GETJOINTANGLES: Calulated '  angleName '(callibrated).' ],...
%                     false,'plotStyle',jointAnglePlotStyle);
%             end
        end
        
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
            sizeTm = size(tm_t);
            tm_t_s = tm_t(indexes);
            if any(size(tm_t_s) ~= sizeTm)
                %                 display('Matrix')
                tm_t_s = tm_t(indexes,:);
            end
            tm_t = tm_t_s;
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
            points_0 = ThreeD.default_points_0;
        end
        
        function plotRun(tm_t,varargin)
            %PLOTRUN Plays back a run
            %tm_t must be a row cell vector. If you would like to
            %plot values side by side then create a matrix of
            %row cell vectors, with each row being a run to plot.
            %Optional argument of a 'timeoutBetweenPlots' can be used to make the plot
            %pauze between plots if you want to get a better understanding
            %of what is happening.
            p = inputParser;
            addOptional(p,'timeoutBetweenPlots',0.0);
            parse(p,varargin{:});
            timeoutBetweenPlots = p.Results.timeoutBetweenPlots;
            
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
                    if (timeoutBetweenPlots ~= 0.0)
                        if timeoutBetweenPlots <0
                            pause
                        else
                            pause(timeoutBetweenPlots);
                        end
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
        
        function theRun = cellTranspose(theRun)
            % CELLTRANSPOSE Gets the quaternion conjugate on the run.
            % cells.
            parfor i = 1:length(theRun)
                theRun{i} = theRun{i}';
            end
        end
        
        function diff = cellInverseMultiply(obj1,obj2)
            % cellInverseMultiply Implement obj1'*obj2 for ThreeD when using
            % cells.
            %display(class(obj1(1)))
            
            %display('Calculating error (vector):')
            minSize = min(size(obj1,2),size(obj2,2));
            diff = cell(1,minSize);
            parfor i = 1:minSize
                diff{i} = obj1{i}'.*obj2{i};
            end
        end
        
        function diff = cellMultiply(obj1,obj2)
            % cellMultiply Implement obj1*obj2 for ThreeD when using
            % cells.
            %display(class(obj1(1)))
            
            %display('Calculating error (vector):')
            minSize = min(size(obj1,2),size(obj2,2));
            diff = cell(1,minSize);
            parfor i = 1:minSize
                diff{i} = obj1{i}.*obj2{i};
            end
        end
        
        function [roll,pitch,yaw,diff_t] = getDiff(tm_t1,tm_t2,varargin)
            %GETDIFF Same as cellminus() but calculates the roll pitch
            %yaw values during the calculation as well.
            %
            p = inputParser;
            addOptional(p,'inDegrees',0.0);
            parse(p,varargin{:});
            inDegrees = p.Results.inDegrees;
            
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
        
        function [roll_t,pitch_t,yaw_t,t] = getRPYt(tm_t,varargin)
            %GETRPYT(tm_t,inDegrees) Get the Roll, Pitch and Yaw
            %of the run tm_t and set inDegrees to true if you
            %would like to see the results in degrees. False for radians.
            if isempty(tm_t)
                error('ThreeD:getRPYt',...
                    ['Make sure that you are using a valid run',...
                    num2str(tm_t)]);
            end
            p = inputParser;
            addOptional(p,'inDegrees',true);
            parse(p,varargin{:});
            inDegrees = p.Results.inDegrees;
            
            minSize = size(tm_t,2);
            roll_t = zeros(1,minSize);
            pitch_t = zeros(1,minSize);
            yaw_t = zeros(1,minSize);
            t = zeros(1,minSize);
            
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
        
        function plotRPY(roll,pitch,yaw,t,varargin)
            %PLOTRPY(roll,pitch,yaw,inDegrees,Fs,t,type)
            %
            %PLOT the Roll Pitch and Yaw for the run.
            %OPTIONAL PARAMTERS:
            % inDegrees - Plot using degrees.
            % plotType - @see ThreeD.plotAngle()
            % plotStyle - @see ThreeD.plotAngle()
            p = inputParser;
            p.addOptional('inDegrees',true);
            p.addOptional('plotType','timeseries');
            p.addOptional('plotStyle','--*b');
            p.parse(varargin{:});
            
            inDegrees = ...
                p.Results.inDegrees ;
            plotType = ...
                p.Results.plotType ;
            plotStyle = ...
                p.Results.plotStyle;
            YMAX = max(max(abs(roll)),max(abs(pitch)));
            YMAX = max(YMAX,max(abs(yaw)));
            YMIN=-YMAX;
            if inDegrees
                YLABEL='(deg)';
            else
                YLABEL='(radians)';
            end
            XLABEL=[' t '];
            XMIN = min(t);
            XMAX = max(t);
            subplot(3,1,1);
            ThreeD.plotAngle(t,roll,plotType,YLABEL,plotStyle)
            hold on;
            grid on;
            %             xlim([XMIN XMAX])
            title(['ROLL(x): maximum angle: ' num2str(max(abs(roll)))]);
            ylabel(YLABEL);
            subplot(3,1,2);
            ThreeD.plotAngle(t,pitch,plotType,YLABEL,plotStyle)
            grid on;
            hold on;
            %             xlim([XMIN XMAX])
            title(['PITCH(y): maximum angle: ' num2str(max(abs(pitch)))]);
            ylabel(YLABEL);
            subplot(3,1,3);
            ThreeD.plotAngle(t,yaw,plotType,YLABEL,plotStyle)
            grid on;
            hold on;
            %             xlim([XMIN XMAX])
            title(['YAW(z): maximum angle: ' num2str(max(abs(yaw)))]);
            ylabel(YLABEL);
            xlabel(XLABEL);
        end
        
        function plotRS(roll,steer,Fs,varargin)
            %PLOTRPY(roll,steer,inDegrees,Fs,t,type)
            %Type
            %PLOT the Roll and Steering angle for the run.
            p = inputParser;
            p.addOptional('inDegrees',true);
            p.addOptional('plotType','timeseries');
            p.addOptional('t',-1);
            p.parse(varargin{:});
            
            inDegrees = ...
                p.Results.inDegrees;
             plotType   = ...
                p.Results.plotType ;
             t = ...
                p.Results.t ;
            
            YMAX = max(max(abs(roll)),max(abs(steer)));
            YMIN=-YMAX;
            if t == -1
                minSize = length(roll);
                t = 0:1/Fs:(minSize-1)/Fs;
                plotType=0;
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
            
            subplot(2,1,1);
            ThreeMarkers.plotAngle(t,roll,plotType,YLABEL)
            hold on;
            grid on;
            xlim([XMIN XMAX])
            title(['ROLL(y): maximum angle: ' num2str(max(abs(roll)))]);
            ylabel(YLABEL);
            subplot(2,1,2);
            ThreeMarkers.plotAngle(t,steer,plotType,YLABEL)
            grid on;
            hold on;
            xlim([XMIN XMAX])
            title(['STEER(x): maximum angle: ' num2str(max(abs(steer)))]);
            ylabel(YLABEL);
        end
        
        function [roll,pitch,yaw,t,theFigure] = ...
                getAndPlotRPYt(theRun_t,theTitle,theFigure,varargin)
            %GETANDPLOTRYP Gets and plots the RPY for the run.
            % Set theFigure to the figure handler if you would like to plot
            % on the same figure or to false if you want a new figure.
            [roll,pitch,yaw,t] = ThreeD.getRPYt(theRun_t);
            if theFigure==false
                theFigure = figure('visible','on','WindowStyle','docked',...
                    'Name',theTitle);
            else
                set(0,'CurrentFigure',theFigure)
                %                 display('Using same figure');
            end
            t_shift = [0 t];
            t_diff = t-t_shift(1:length(t_shift)-1);
            Fs = mean(1./t_diff(2:length(t_diff)));
            %t = t - t(1);
            ThreeD.plotRPY(roll,pitch,yaw,t,...
                varargin{:});
        end
        
        function [Fs,Variance] = estimateFsAndVariance(t)
            %ESTIMATEFSANDVARIANCE(t) Estimate the Frequency of the samples
            %and the variance.
            t_shift = [0 t];
            t_diff = t-t_shift(1:length(t_shift)-1);
            Fs_t = 1./t_diff(2:length(t_diff));
            Fs = mean(Fs_t);
            Variance =var(Fs_t);
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
            p_0mat = cell2mat(p_0);
            p_1mat = cell2mat(p_1);
            p_0mat(isnan(p_0mat))=0;
            p_1mat(isnan(p_1mat))=0;
            absorRet = absor(p_0mat(1:3,:),p_1mat(1:3,:));
            %             H_1_0_est = absorRet.M;
            %             H_1_0_est(1:3,4) = 0;
            %             H_1_0_est(4,1:3) = 0;
            %             H_1_0_est(4,4) = 1;
            %display('Estimated H')
            %H_1_0_est
            tm_est = ThreeD(absorRet.q');
            tm_est = tm_est';
            
        end
        
        function [tm_t_c,tm_est] = ...
                zeroTheRun(tm_t,startIndex,numberOfSamples)
            %ZEROTHERUN Changes the samples to the zero frame using
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
            %The timestamps are readjusted to be zero from the startIndex
            %position.
            %
            % Returns the tm_t(startIndex:length(tm_t)).
            N=numberOfSamples+startIndex;
            zeroRun = cell(1,N);
            parfor i = 1:N
                zeroRun{i} = ThreeD([1 0 0 0]);
            end
            tm_est = ThreeD.getChangeOfGlobalReferenceFrames(zeroRun,...
                tm_t,startIndex,numberOfSamples);
            tm_t_c = tm_t(startIndex:length(tm_t));
            timestamp1=tm_t{startIndex}.getTimestamp;
            parfor i =  1:length(tm_t_c)
                newTm = tm_est.*tm_t_c{i};
                newTm = newTm.setTimestamp(newTm.getTimestamp...
                    -timestamp1)
                tm_t_c{i} = newTm;
            end
        end
        
        function [tm_t,t] = resample(tm_t,t_wanted)
            %RESAMPLE Resamples a cell of ThreeD
            %The cell of ThreeD is resampled to the timestamps
            %specified in the vector t_wanted.
            display(['Resample the data']);
            quats = zeros(length(tm_t),4);
            t = zeros(length(tm_t),1);
            parfor i = 1:length(tm_t)
                quats(i,:) = tm_t{i}.getQ;
                t(i) = tm_t{i}.getTimestamp;
            end
            %             quats
            quat_ts = timeseries(quats,t);
            quats = resample(quat_ts,t_wanted,'zoh');
            
            %             quats
            N = length(t_wanted);
            tm_t = cell(1,N);
            parfor i = 1:N
                %                 i
                %                 quats(i,:)
                if any(isnan(quats.Data(i,:)))
                    %SHOULD DO THIS MORE CLEVERLY
                    %display('NAN')
                    warning('ThreeD:resample',['Incorrect resampling:' num2str(i)]);
                    theQuat = ThreeD([1 0 0 0]);
                else
                    theQuat = ThreeD(quats.Data(i,:));
                end
                theQuat = theQuat.setTimestamp(quats.Time(i));
                tm_t{i} = theQuat;
            end
            t = quats.Time;
        end
        
        function [tm_t] = changeStartTime(tm_t,newStartTime)
            %CHANGESTARTTIME(tm_t,newStartTime) Change the run to have the
            %new start time.
            timestamp1=tm_t{1}.getTimestamp;
            parfor i = 1:length(tm_t)
                tm = tm_t{i}.setTimestamp(tm_t{i}.getTimestamp...
                    -timestamp1+newStartTime);
                tm_t{i} = tm;
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
                qtm.H_0_T = quaternionOrH;
                qtm.quaternion = matrix2quaternion(quaternionOrH);
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
            %Adams uses zxz.
            %euler = quaternion2euler(tm.getQ,inDegrees,'zxz');
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
        
        function [points_0] = getPoint0(tm)
            %GET0 Gets the zero Frame (global reference frame)
            %used for this sample.
            points_0 = tm.points_0;
        end
        
        function [points_0] =setPoint0(tm,points_0)
            %GET0 Gets the zero Frame (global reference frame)
            %used for this sample.
            tm.points_0 = points_0;
        end
        
        function [quat] = toNumeric(tm)
            %TONUMERIC(tm) How to display the object numerically.
            quat = [ tm.getQ() tm.getTimestamp ];
        end
        
    end
end


