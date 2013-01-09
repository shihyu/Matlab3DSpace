classdef RawMarkers
    %RAWMARKERS This class is used to package all the reading
    % of raw C3D Three Marker data from SofieHDFFormat.
    % Various proxessing
    methods(Static)
        function [marker1n,marker2n,marker3n] = eraseNan(marker1,marker2,marker3)
            N=size(marker1,1);
            for i = 1:N
                if (any(isnan(marker1(i,1:4)))) || ...
                        (any(isnan(marker2(i,1:4)))) || ...
                        (any(isnan(marker3(i,1:4))))
                    marker1n(i,1:4) =0;
                    marker2n(i,1:4) =0;
                    marker3n(i,1:4) =0;
                else marker1n(i,:) = marker1(i,:);
                    marker2n(i,:) = marker2(i,:);
                    marker3n(i,:) = marker3(i,:);
                end
            end
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
        
        function [yesOrNo,distances]=...
                areTheMarkersWellSpaced(rawData,varargin)
            %areTheMarkersWellSpaced Tests to see if the markers
            %are in a good triangle.
            % RETURNS
            % yesOrNo = [RBLB-RBFT,RBLB-FTLB,FTLB-RBLT];
            % distances = [RBLB,RBFT,FTLB]
            p = inputParser;
            addOptional(p,'lengthThreshold',14.0);
            addOptional(p,'plotTheDistances',false);
            parse(p,varargin{:});
            lengthThreshold = p.Results.lengthThreshold;
            plotTheDistances = p.Results.plotTheDistances;
            
            RBLB = sqrt(sum((rawData(:,1:3) - rawData(:,4:6)).^2,2))
            RBFT = sqrt(sum((rawData(:,1:3) - rawData(:,7:9)).^2,2))
            FTLB = sqrt(sum((rawData(:,7:9) - rawData(:,4:6)).^2,2))

            
            if plotTheDistances
                figure;
                subplot(3,3,1);
                plot(RBLB')
            end
            
            distances = [RBLB,RBFT,FTLB];
            yesOrNo = [abs(RBLB - RBFT),abs(RBLB - FTLB),...
                abs(FTLB - RBFT)];
            yesOrNo = yesOrNo<= lengthThreshold;
        end
        
        function [yesOrNoAll,yesOrNoSome] = ...
                areMarkersDropped(rawData,varargin)
            %areMarkersDropped  To test for dropped markers
            %or bad measurements.
            yesOrNoAll = RawMarkers.isNanOrZero(rawData(:,1:9));
            yesOrNoSome = RawMarkers.isNanOrZero(rawData(:,1:9),...
                'allOrSome',false);
            
            p = inputParser;
            addOptional(p,'yesOrNoPlot',false);
            addOptional(p,'sensorName','sensorName');
            addOptional(p,'keepColumnLogicResult',false);
            parse(p,varargin{:});
            yesOrNoPlot = p.Results.yesOrNoPlot;
            sensorName = p.Results.sensorName;
            keepColumnLogicResult = p.Results.keepColumnLogicResult;
            
            if yesOrNoPlot
                yesOrNoAll=double(yesOrNoAll);
                yesOrNoSome=double(yesOrNoSome);
                yesOrNoAll(yesOrNoAll==0)=NaN; %set all zeroes to NaN
                yesOrNoSome(yesOrNoSome==0)=NaN; %set all zeroes to NaN
                
                subplot(3,1,1);
                plot(rawData(:,10),rawData(:,1:3), '--*b');
                hold on
                plot(rawData(:,10),yesOrNoAll(:,1), '*r');
                plot(rawData(:,10),yesOrNoSome(:,1), '*g');
                title([sensorName '- Right Back raw data with dropped samples']);
                xlabel('Time (sec)');
                ylabel('Distance (mm)');
                
                subplot(3,1,2);
                plot(rawData(:,10),rawData(:,4:6), '--*b');
                hold on
                plot(rawData(:,10),yesOrNoAll(:,1), '*r');
                plot(rawData(:,10),yesOrNoSome(:,1), '*g');
                title([sensorName '- Left Back raw data with dropped samples']);
                xlabel('Time (sec)');
                ylabel('Distance (mm)');
                
                subplot(3,1,3);
                plot(rawData(:,10),rawData(:,7:9), '--*b');
                hold on
                plot(rawData(:,10),yesOrNoAll(:,1), '*r');
                plot(rawData(:,10),yesOrNoSome(:,1), '*g');
                title([sensorName '- front raw data with dropped samples']);
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
            
            isJumpLeft = any(~(abs(jumpLeft) < maxVeljump*dt)); % jump on the leftside ~ is used to also include NaN as positive result
            isJumpright = any(~(abs(jumpRight) < maxVeljump*dt)); % jump on the right side
            isOpposite = any(sign(jumpLeft) ~= sign(jumpRight)); % jumps are opposite
            
            markerData(:, isJumpLeft & isJumpright & isOpposite) = NaN;
            
            %% Set all exact zeroes to NaN
            markerData(markerData==0)=NaN;
            
            markerOutliers = markerData';
        end
        
        function [filMarkers,gapArrayMarker] = fillGaps(markerOutliers,varargin)
            %% FILLGAPS finds the gaps in the data and fill the gaps where possible
            % the gaps are marked with a NaN.
            % The array markerOutliers has size: (N,1)
            % This function should be called from findAndFillGaps to use
            % on a [N,10] rawMarker matrix.
            % The output array, filMarkers is a (N,1) array containing the
            % interpolated marker data.
            
            p = inputParser;
            addOptional(p,'maxGap',10); %max gap to interpolate
            addOptional(p,'interpMethod', 'linear');
            parse(p,varargin{:});
            maxGap = p.Results.maxGap;
            interpMethod = p.Results.interpMethod;
            
            markerOutliers = markerOutliers';
            nFrames = length(markerOutliers);
            % make a n x 2 matrix containg on every row the beginning and the ending of
            % a gap.
            gapArrayMarker = [find(diff(isnan([0; markerOutliers(1,:)'])) > 0) ...
                find(diff(isnan([markerOutliers(1,:)'; 0])) < 0)];
            % loop the gaps and fill them
            for iGap = 1:size(gapArrayMarker, 1); % loop all the gaps
                isAtBeginning = gapArrayMarker(iGap, 1) == 1; % check if the gap is at the beginning
                isAtEnd = gapArrayMarker(iGap, 2) == nFrames; % check if the gap is at the end
                isTooLarge = gapArrayMarker(iGap, 2) - gapArrayMarker(iGap, 1) > maxGap; % gap is too large to interpolate
                if ~isAtBeginning && ~isAtEnd && ~isTooLarge % only interpolate if none of these conditions are true
                    markerOutliers(:, gapArrayMarker(iGap, 1) - 1 : gapArrayMarker(iGap, 2) + 1) = ...
                        interp1([gapArrayMarker(iGap, 1) - 1 gapArrayMarker(iGap, 2) + 1], ...
                        [markerOutliers(:, gapArrayMarker(iGap, 1) - 1) markerOutliers(:, gapArrayMarker(iGap, 2) + 1)]', ...
                        gapArrayMarker(iGap, 1) - 1 : gapArrayMarker(iGap, 2) + 1, interpMethod);
                end
            end
            filMarkers = markerOutliers';
        end
        
        function [interpData,gapArrayMarker] = findFillGaps(rawData,varargin)
            %FINDFILLGAPS loops over the 9 columns of the matrix rawData...
            %finds the gaps and outliers and
            %fills the gaps, using interpolation
            
            p = inputParser;
            addOptional(p,'maxGap',10); %max gap to interpolate
            addOptional(p,'interpMethod', 'linear');
            addOptional(p,'maxVeljump',1000); %max velocity jump in mm/sec
            addOptional(p,'Fs',100);
            addOptional(p,'plotOrNot',false);
            parse(p,varargin{:});
            maxGap = p.Results.maxGap;
            interpMethod = p.Results.interpMethod;
            maxVeljump = p.Results.maxVeljump;
            Fs = p.Results.Fs;
            plotOrNot = p.Results.plotOrNot;
            
            interpData = zeros(length(rawData(:,1)),10);
            for i = 1:9
                markerOutliers = RawMarkers.findOutliers(rawData(:,i));
                [interpData(:,i),gapArrayMarker] = RawMarkers.fillGaps(markerOutliers);
            end
            interpData(:,10) = rawData(:,10);
            if plotOrNot
                for i = 1:9
                    subplot(9,3,i)
                    plot(rawData(:,10),rawData(:,i), '--*b');
                    hold on
                    plot(rawData(:,10),interpData(:,i), '*r');
                    title([rightBackName '- raw data and interpolated data']);
                    xlabel('Time (sec)');
                    ylabel('Distance (mm)');
                end
            end
        end
        
        function [rawDataFilt] = filterRawData(rawData,varargin)
            %filterRawData filters the [N,10] rawData matrix calling
            %the function filterMarkerData
            addOptional(p,'freqLowPass',15);
            addOptional(p,'orderLowPass',4);
            addOptional(p,'Fs',100);
            parse(p,varargin{:});
            freqLowPass = p.Results.freqLowPass;
            orderLowPass = p.Results.orderLowPass;
            Fs = p.Results.Fs;
            
            rawDataFilt = zeros(length(rawData(:,1)),10);
            for i = 1:9
                markerOutliers = RawMarkers.filterMarkerData(rawData(:,i));
                rawDataFilt(:,i) = RawMarkers.fillGaps(markerOutliers);
            end
            rawDataFilt(:,10) = rawDataFilt(:,10);
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
            
            
            dt = 1/Fs;
            markerFilt = nan(size(markerData)); % NaN matrix for the marker data
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
            display(['Data filtered with butterworth low pas filter', 'freqLowPass =', num2str(freqLowPass)...
                , 'orderLowPass =', num2str(orderLowPass)]);
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

