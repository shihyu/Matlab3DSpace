classdef RawMarkers
    %RAWMARKERS This class is used to package all the reading
    % of raw C3D Three Marker data from SofieHDFFormat.
    % Various proxessing
    methods(Static)
        function [rawDataNanErased, t_beforeErased] = eraseNan(rawData)
            N=size(rawData,1);
            CountDroppedMarkers = 0;
            rawDataNanErased = [];
            for i = 1:N
                if (any(isnan(rawData(i,1:10))))
                    CountDroppedMarkers = CountDroppedMarkers +1;
                    
                else rawDataNanErased2 = rawData(i,1:10) ;
                    rawDataNanErased = [ rawDataNanErased;rawDataNanErased2];
                end
            end
            display(['Dropped Markers =', num2str(CountDroppedMarkers)]);
            if ((CountDroppedMarkers/N)*100) > 5
                warning('Markers3D:readDataVicon','Dropped Markers is more than 5 percent');
            end
            t_beforeErased = rawData(:,10);
        end
        
        function [yesOrNo] = isNanOrZero(marker,varargin)
            p = inputParser;
            addOptional(p,'allOrSome',true);
            addOptional(p,'keepColumnLogicResult',false);
            parse(p,varargin{:});
            allOrSome = p.Results.allOrSome;
            keepColumnLogicResult = p.Results.keepColumnLogicResult;
            if keepColumnLogicResult
                yesOrNo = isnan(marker)|(marker==0);
                return;
            end
            if allOrSome
                display('All Markers dropped - NaN or Zero')
                yesOrNo = all(isnan(marker)|(marker==0),2);
            else
                yesOrNo = any(isnan(marker)|(marker==0),2);
            end
        end
        
        function [spacedData] = removeNotWellSpaced(rawData,...
                yesOrNoSpaced)
            
            countNotWellSpaced =0;
            spacedData = zeros(size(rawData));
            N = size(rawData,1);
            for i = 1:N
                if yesOrNoSpaced(i)==0;
                    spacedData(i,1:9)=NaN;
                    countNotWellSpaced = countNotWellSpaced +1;
                else spacedData(i,1:9)=rawData(i,1:9);
                end
            end
            spacedData(:,10)=rawData(:,10);
            display(['Number of Not-well-spaced triangles =', num2str(countNotWellSpaced)]);
            if ((countNotWellSpaced/N)*100) > 5
                warning('RawMarkers:removeNotWellSpaced','Dropped Markers is more than 5 percent');
            end
        end
        
        function [yesOrNo,distances]=...
                areTheMarkersWellSpaced(rawData,varargin)
            %areTheMarkersWellSpaced Tests to see if the markers
            %are in a good triangle.
            % RETURNS
            % yesOrNo = [RBLB-RBFT,RBLB-FTLB,FTLB-RBLT];
            % or when withRespectToMean is true:
            % yesOrNo = [mRBLB-RBLB,mRBFT-RBFT,mFTLB-FTLB];
            % distances = [RBLB,RBFT,FTLB]
            p = inputParser;
            addOptional(p,'lengthThreshold',14.0);
            addOptional(p,'plotTheDistances',false);
            addOptional(p,'sensorName','sensorName');
            addOptional(p,'plotBoxPlot',false);
            addOptional(p,'withRespectToMean',false)
            
            parse(p,varargin{:});
            lengthThreshold = p.Results.lengthThreshold;
            plotTheDistances = p.Results.plotTheDistances;
            sensorName = p.Results.sensorName;
            plotBoxPlot = p.Results.plotBoxPlot;
            withRespectToMean = p.Results.withRespectToMean;
            
            RBLB = sqrt(sum((rawData(:,1:3) - rawData(:,4:6)).^2,2));
            RBFT = sqrt(sum((rawData(:,1:3) - rawData(:,7:9)).^2,2));
            FTLB = sqrt(sum((rawData(:,7:9) - rawData(:,4:6)).^2,2));
            
            distances = [RBLB,RBFT,FTLB];
            mRBLB = nanmean(RBLB);
            mRBFT = nanmean(RBFT);
            mFTLB = nanmean(FTLB);
            
            
            if withRespectToMean
                yesOrNo = [abs(mRBLB - RBLB),abs(mRBFT-RBFT),...
                    abs(mFTLB-FTLB)];
                yesOrNo = yesOrNo<= lengthThreshold;
            else
                yesOrNo = [abs(mRBLB - RBFT),abs(RBLB - FTLB),...
                    abs(FTLB - RBFT)];
                yesOrNo = yesOrNo<= lengthThreshold;
            end
            %yesOrNo = RawMarkers.isNanOrZero(rawData(:,1:9));
            
            if plotBoxPlot
                figure('visible','on','WindowStyle','docked',...
                    'Name',[sensorName '- Boxplot of lengths of the sides of the triangle']);
                boxplot([RBLB,FTLB,RBFT],'labels',{['RBLB - mean is = ' num2str(nanmean(RBLB))],...
                    ['FTLB - mean = ' num2str(nanmean(FTLB))],['RBFT - mean = ' num2str(nanmean(RBFT))]});
                grid on;
                ylabel('Absolute length of the triangle side in mm.')
            end
            if plotTheDistances
                figure('visible','on','WindowStyle','docked',...
                    'Name',[sensorName '- Lengths of the sides of the triangle over time']);
                plot(rawData(:,10),distances,'-*');
                                yesOrNoPlot=double(yesOrNo);
                                yesOrNoPlot(yesOrNoPlot==1)=NaN;
                                yesOrNoPlot=yesOrNoPlot+mRBLB;
                                hold on
                plot(rawData(:,10),yesOrNoPlot, 'sm');
                legend('RBLB','RBFT','FTLB','Outlier');
                hold on
                ymin = 0;
                ymax = 80;
               % ylim([ymin ymax]);
                hline=(refline([0,mRBLB])); hline2=(refline([0,mRBFT]));
                hline3=(refline([0,mFTLB]));
                set(hline,'Color', 'b','LineWidth',2); set(hline2,'Color', 'g','LineWidth',2);
                set(hline3,'Color', 'r','LineWidth',2);
                grid on;
                ylabel('Absolute length of the triangle side in mm.')
                xlabel('Time in sec.')
                title(['Lengths of the triangle sides plotted against the time, threshold = ' num2str(lengthThreshold) ' mm']);
            end
        end
        
        function [yesOrNoAll,yesOrNoSome,yesOrNoOutlier] = ...
                areMarkersDropped(rawData,varargin)
            %areMarkersDropped  To test for dropped markers
            %or bad measurements. Markers are Dropped when they are all
            %zero or Nan or when some of them are (in the raw markerdata
            %matrix of 9 columns (x,y,z of the three markers)).
            %Also it is checked if they contain outliers
            
            p = inputParser;
            addOptional(p,'yesOrNoPlot',false);
            addOptional(p,'sensorName','sensorName');
            addOptional(p,'keepColumnLogicResult',false);
            addOptional(p,'maxVelJump',1000.0);
            addOptional(p,'Fs',100);            
            parse(p,varargin{:});
            yesOrNoPlot = p.Results.yesOrNoPlot;
            sensorName = p.Results.sensorName;
            keepColumnLogicResult = p.Results.keepColumnLogicResult;
            maxVelJump = p.Results.maxVelJump;
            Fs = p.Results.Fs
            
            yesOrNoAll = RawMarkers.isNanOrZero(rawData(:,1:9));
            yesOrNoSome = RawMarkers.isNanOrZero(rawData(:,1:9),...
                'allOrSome',false);
            
            for i = 1:9
                markerOutliers = RawMarkers.findOutliers(rawData(:,i),'maxVelJump',...
                    maxVelJump,'Fs', Fs);
            end
            
            yesOrNoOutlier = RawMarkers.isNanOrZero(markerOutliers,...
                'allOrSome',false);
            
            
            if yesOrNoPlot
                yesOrNoAll=double(yesOrNoAll);
                yesOrNoSome=double(yesOrNoSome);
                yesOrNoOutlier=double(yesOrNoOutlier);

                yesOrNoAll(yesOrNoAll==0)=NaN; %set all zeroes to NaN
                yesOrNoSome(yesOrNoSome==0)=NaN; %set all zeroes to NaN
                yesOrNoOutlier(yesOrNoOutlier==0)=NaN; %set all zeroes to NaN

                
                figure('visible','on','WindowStyle','docked',...
                    'Name',[sensorName ' - raw Data with dropped samples']);
                
                subplot(3,1,1);
                plot(rawData(:,10),rawData(:,1), '--*b');
                hold on
                plot(rawData(:,10),rawData(:,2), '--*k');
                plot(rawData(:,10),rawData(:,3), '--*c');
                plot(rawData(:,10),yesOrNoAll, 'sr');
                plot(rawData(:,10),yesOrNoSome, 'og');
                plot(rawData(:,10),yesOrNoOutlier, '<m');
                %  plot(rawData(:,10),yesOrNoSpaced(:,1), '+y');
                title([sensorName '- Right Back']);
                xlabel('Time (sec)');
                ylabel('Distance (mm)');
                legend('raw Data x','raw Data y','raw Data z','all NaN or Zero',...
                    'some NaN or Zero', 'some Outliers');
                
                subplot(3,1,2);
                plot(rawData(:,10),rawData(:,4), '--*b');
                hold on
                plot(rawData(:,10),rawData(:,5), '--*k');
                plot(rawData(:,10),rawData(:,6), '--*c');
                plot(rawData(:,10),yesOrNoAll, 'sr');
                plot(rawData(:,10),yesOrNoSome, 'og');
                plot(rawData(:,10),yesOrNoOutlier, '<m');
                %  plot(rawData(:,10),yesOrNoSpaced(:,1), '+y');
                title([sensorName '- Left Back']);
                xlabel('Time (sec)');
                ylabel('Distance (mm)');
                
                subplot(3,1,3);
                plot(rawData(:,10),rawData(:,7), '--*b');
                hold on
                plot(rawData(:,10),rawData(:,8), '--*k');
                plot(rawData(:,10),rawData(:,9), '--*c');
                plot(rawData(:,10),yesOrNoAll, 'sr');
                plot(rawData(:,10),yesOrNoSome, 'og');
                plot(rawData(:,10),yesOrNoOutlier, '<m');
                % plot(rawData(:,10),yesOrNoSpaced(:,1), '+y');
                title([sensorName '- front']);
                xlabel('Time (sec)');
                ylabel('Distance (mm)');
            end
        end
        
        function [markerOutliers] = findOutliers(markerData,varargin)
            %FINDOUTLIERS finds the gaps and outliers in a markerdata array
            % The array has size: (N,1)
            % This function should be called from findAndFillGaps to use
            % on a [N,10] rawMarker matrix.
            % maxGap is the maximum gap between two values
            % maxVeljump the maximum velocity jump
            % The output array, markerOutliers contains NaNs at the outlier
            % or gap positions.
            p = inputParser;
            addOptional(p,'maxVeljump',1000); %max velocity jump in mm/sec
            addOptional(p,'Fs',100);
            parse(p,varargin{:});
            maxVeljump = p.Results.maxVeljump;
            Fs = p.Results.Fs;
            
            dt = 1/Fs;
            %% find outliers
            markerData = markerData'; % make it a row array
            jumpLeft = markerData - markerData(:, [2 1:end-1]); % derivative on the left side
            jumpRight = markerData(:, [2:end end-1]) - markerData; % derivative on the right side
            isJumpLeft = (~(abs(jumpLeft) < maxVeljump*dt)); % jump on the leftside ~ is used to also include NaN as positive result
            isJumpright = (~(abs(jumpRight) < maxVeljump*dt)); % jump on the right side
            isOpposite = (sign(jumpLeft) ~= sign(jumpRight)); % jumps are opposite
            
            markerData(:, isJumpLeft & isJumpright & isOpposite) = NaN;
            
            %% Set all exact zeroes to NaN
            markerData(markerData==0)=NaN;
            
            markerOutliers = markerData';
        end
        
        function [filMarkers,gapArrayMarker] = fillGaps(gapData,varargin)
            %% FILLGAPS finds the gaps in the data and fill the gaps where possible
            % the gaps are marked with a NaN.
            % The array markerOutliers has size: (N,1)
            % This function should be called from findAndFillGaps to use
            % on a [N,10] rawMarker matrix.
            % The output array, filMarkers is a (N,1) array containing the
            % interpolated marker data.
            
            p = inputParser;
            addOptional(p,'maxGap',50); %max gap to interpolate
            addOptional(p,'interpMethod', 'cubic');
            parse(p,varargin{:});
            maxGap = p.Results.maxGap;
            interpMethod = p.Results.interpMethod;
            
            gapData = gapData';
            nFrames = length(gapData);
            % make a n x 2 matrix containg on every row the beginning and the ending of
            % a gap.
            gapArrayMarker = [find(diff(isnan([0; gapData(1,:)'])) > 0) ...
                find(diff(isnan([gapData(1,:)'; 0])) < 0)];
            % loop the gaps and fill them
            for iGap = 1:size(gapArrayMarker, 1); % loop all the gaps
                isAtBeginning = gapArrayMarker(iGap, 1) == 1; % check if the gap is at the beginning
                isAtEnd = gapArrayMarker(iGap, 2) == nFrames; % check if the gap is at the end
                isTooLarge = gapArrayMarker(iGap, 2) - gapArrayMarker(iGap, 1) > maxGap; % gap is too large to interpolate
                if ~isAtBeginning && ~isAtEnd && ~isTooLarge % only interpolate if none of these conditions are true
                    gapData(:, gapArrayMarker(iGap, 1) - 1 : gapArrayMarker(iGap, 2) + 1) = ...
                        interp1([gapArrayMarker(iGap, 1) - 1 gapArrayMarker(iGap, 2) + 1], ...
                        [gapData(:, gapArrayMarker(iGap, 1) - 1) gapData(:, gapArrayMarker(iGap, 2) + 1)]', ...
                        gapArrayMarker(iGap, 1) - 1 : gapArrayMarker(iGap, 2) + 1, interpMethod,'extrap');
                end
            end
            filMarkers = gapData';
        end
        
        function [interpData] = interpolateGaps(gapData,rawData,varargin)
            %INTERPOLATEGAPS interpolates the gaps in the matrix gapData
            %It uses the function fillGaps and loops over the 9 columns of
            %the matrix
            p = inputParser;
            addOptional(p,'maxGap',30); %max gap to interpolate
            addOptional(p,'interpMethod', 'cubic');
            addOptional(p,'plotOrNot',false);
            addOptional(p,'sensorName','sensorName');
            parse(p,varargin{:});
            maxGap = p.Results.maxGap;
            interpMethod = p.Results.interpMethod;
            plotOrNot = p.Results.plotOrNot;
            sensorName = p.Results.sensorName;
            
            interpData = zeros(length(gapData(:,1)),10);
            for i = 1:9
                [interpData(:,i),gapArrayMarker] = RawMarkers.fillGaps(gapData(:,i));
                %  [interpData(:,i),gapArrayMarker] = RawMarkers.fillGaps(markerOutliers);
            end
            interpData(:,10) = gapData(:,10);
            
            if plotOrNot
                figure('visible','on','WindowStyle','docked',...
                    'Name',[sensorName,'Raw data and interpolated data']);
                
                for i = 1:9
                    subplot(3,3,i)
                    plot(rawData(:,10),rawData(:,i), '--*b');
                    hold on
                    plot(rawData(:,10),interpData(:,i), 'sr');
                    xlabel('Time (sec)');
                    ylabel('Distance (mm)');
                    
                end
            end
        end
        
        
        
        function [gapData] = findGaps(rawData)
            %FINDFILLGAPS loops over the 9 columns of the matrix rawData...
            %finds the gaps and outliers and remove them by making them NaN
            
            gapData = zeros(length(rawData(:,1)),10);
            for i = 1:9
                gapData(:,i) = RawMarkers.findOutliers(rawData(:,i));
            end
            gapData(:,10) = rawData(:,10);
        end
        
        function [rawDataFilt] = filterRawData(rawData,varargin)
            %filterRawData filters the [N,10] rawData matrix calling
            %the function filterMarkerData
            p = inputParser;
            addOptional(p,'freqLowPass',15);
            addOptional(p,'orderLowPass',4);
            addOptional(p,'Fs',100);
            addOptional(p,'plotOrNot',false);
            addOptional(p,'sensorName','sensorName');
            parse(p,varargin{:});
            freqLowPass = p.Results.freqLowPass;
            orderLowPass = p.Results.orderLowPass;
            Fs = p.Results.Fs;
            plotOrNot = p.Results.plotOrNot;
            sensorName = p.Results.sensorName;
            
            rawDataFilt = zeros(length(rawData(:,1)),10);
            for i = 1:9
                rawDataFilt(:,i) = RawMarkers.filterMarkerData(rawData(:,i));
            end
            rawDataFilt(:,10) = rawData(:,10);
            %rawDataFilt
            if plotOrNot
                figure('visible','on','WindowStyle','docked',...
                    'Name',[sensorName,'Interpolated and filtered data']);
                for i = 1:9
                    subplot(3,3,i)
                    plot(rawData(:,10),rawData(:,i), '--*b');
                    hold on
                    plot(rawData(:,10),rawDataFilt(:,i), '-r', 'LineWidth', 3);
                    %title([sensorName '- raw data and filter data']);
                    xlabel('Time (sec)');
                    ylabel('Distance (mm)');
                end
            end
            
            
            
            
        end
        
        function [markerFilt] = filterMarkerData(markerData,varargin)
            % low-pass filter the data
            % this function works on an [N,1] array (markerdata)
            % the function filterRawData should be used to loop trough the
            % matrix
            % Fs the sampling frequency of the data.
            % freqLowPass = 15; % the low pass filter frequency [Hz]
            % orderLowpass = 4; % order of the filter
            p = inputParser;
            addOptional(p,'freqLowPass',15);
            addOptional(p,'orderLowPass',4);
            addOptional(p,'Fs',100);
            parse(p,varargin{:});
            freqLowPass = p.Results.freqLowPass;
            orderLowPass = p.Results.orderLowPass;
            Fs = p.Results.Fs;
            
            display(['Data filtered with butterworth low pass filter', ' - freqLowPass = ', num2str(freqLowPass)...
                , 'orderLowPass =', num2str(orderLowPass)]);
            
            
            markerData = markerData';
            dt = 1/Fs;
            markerFilt = nan(size(markerData)); % NaN vector for the marker data
            omegaLow = 2 * freqLowPass * dt;
            [B, A]=butter(orderLowPass, omegaLow, 'low'); % Butterworth filter
            % make a n x 2 matrix containg on every row the beginning and the ending of
            % a continues set of data.
            dataArray = [find(diff(~isnan([NaN; markerData(1,:)'])) > 0) ...
                find(diff(~isnan([markerData(1,:)'; NaN])) < 0)];
            
            for iData = 1:size(dataArray, 1); % loop all the gaps
                if dataArray(iData,2) - dataArray(iData,1) > orderLowPass * 3 % length data must be 3 times the filter order
                    markerFilt(:, dataArray(iData,1) : dataArray(iData,2)) = ...
                        filtfilt(B, A, markerData(:, dataArray(iData,1) : dataArray(iData,2))')';
                end
            end
            markerFilt = markerFilt';
            
        end
        
        
        function [rawData] = readFromFile(filename,runName,...
                rightBackName,leftBackName,frontName,varargin)
            %READFROMFILE Read the raw data from a file
            %RETURNS rawData = [rightback,leftback,front,t];
            %Optional Parameter:
            %  adams - Set true to read adams file.
            %Parameter Parsing.
            p = inputParser;
            addOptional(p,'adams',false);
            parse(p,varargin{:});
            adams = p.Results.adams;
            if adams
                reader = adamsReader(filename,runName);
                data = reader.readData(false);
                t=data.Time;
                
                RBO = [
                    data.([rightBackName 'X']),...
                    data.([rightBackName 'Y']),...
                    data.([rightBackName 'Z'])];
                LBO = [
                    data.([leftBackName 'X']),...
                    data.([leftBackName 'Y']),...
                    data.([leftBackName 'Z'])];
                FON = [
                    data.([frontName 'X']),...
                    data.([frontName 'Y']),...
                    data.([frontName 'Z'])];
                rawData = [RBO,LBO,FON,t];
            else
                reader = c3dReader(filename,runName);
                rightBack = reader.readMarker(rightBackName);
                leftBack = reader.readMarker(leftBackName);
                front = reader.readMarker(frontName);
                rawData = [rightBack(:,1:3),leftBack(:,1:3),...
                    front(:,1:3),front(:,4)];
            end
        end
    end
end

