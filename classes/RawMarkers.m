classdef RawMarkers < handle
    %RAWMARKERS This class is used to package all the reading
    % of raw C3D Three Marker data from SofieHDFFormat.
    % Various proxessing
    
    properties
        a=0;
        b=0;
    end
    
    methods(Static)
         function markerdataFilt = filterAxis(markerdata, time,...
                maxGap,maxVeljump,...
                freqLowPass,orderLowpass)
            %FILTERAXIS Processes the raw marker data comming from a motion capture
            %system.
            %   INPUT:
            % maxGap (10):
            % maximal number of frames that are allowed to be interpolated
            % maxVeljump (0.1):
            % maximal acceleration that is allowed for markers, otherwise markers are
            % considdered as outliers  % [m/s]
            % freqLowPass (15):
            % the low pass filter frequency [Hz]
            % orderLowpass (4):
            % order of the filter
            %       markerdata matrix (n x 1) n frames [meters]
            %       dt scalar sampling time [seconds]
            %   OUTPUT:
            %       markerdatefilt matrix with the same elements as the markerdata
            markerdata = markerdata';
            
            
            %% data and filter characteristics
            nFrames = size(markerdata, 2); % number of frames
            Fs = ThreeD.estimateFsAndVariance(time');
            dt = 1/Fs;
            
            %% find outliers
            jumpLeft = markerdata - markerdata(:, [2 1:end-1]); % derivative on the left side
            jumpRight = markerdata(:, [2:end end-1]) - markerdata; % derivetive on the right side
            
            isJumpLeft = any(~(abs(jumpLeft) < maxVeljump*dt)); % jump on the leftside ~ is used to also include NaN as positive result
            isJumpright = any(~(abs(jumpRight) < maxVeljump*dt)); % jump on the right side
            isOpposite = any(sign(jumpLeft) ~= sign(jumpRight)); % jumps are opposite
            
            markerdata(:, isJumpLeft & isJumpright & isOpposite) = NaN;
            
            markerdata(markerdata==0)=NaN;
            
            %% find and fill gaps
            % finds the gaps in the data and fill the gaps where possible
            
            % make a n x 2 matrix containg on every row the beginning and the ending of
            % a gap.
            gapArray = [find(diff(isnan([0; markerdata(1,:)'])) > 0) ...
                find(diff(isnan([markerdata(1,:)'; 0])) < 0)];
            
            % loop the gaps and fill them
            for iGap = 1:size(gapArray, 1); % loop all the gaps
                isAtBeginning = gapArray(iGap, 1) == 1; % check if the gap is at the beginning
                isAtEnd = gapArray(iGap, 2) == nFrames; % check if the gap is at the end
                isTooLarge = gapArray(iGap, 2) - gapArray(iGap, 1) > maxGap; % gap is too large to interpolate
                if ~isAtBeginning && ~isAtEnd && ~isTooLarge % only interpolate if none of these conditions are true
                    markerdata(:, gapArray(iGap, 1) - 1 : gapArray(iGap, 2) + 1) = ...
                        interp1([gapArray(iGap, 1) - 1 gapArray(iGap, 2) + 1], ...
                        [markerdata(:, gapArray(iGap, 1) - 1) markerdata(:, gapArray(iGap, 2) + 1)]', ...
                        gapArray(iGap, 1) - 1 : gapArray(iGap, 2) + 1)';
                end
            end
            
            %% low-pass filter the data
            markerdataFilt = nan(size(markerdata)); % NaN matrix for the marker data
            omegaLow = 2 * freqLowPass * dt;
            [B, A]=butter(orderLowpass, omegaLow, 'low'); % Butterworth filter
            
            % make a n x 2 matrix containg on every row the beginning and the ending of
            % a continues set of data.
            dataArray = [find(diff(~isnan([NaN; markerdata(1,:)'])) > 0) ...
                find(diff(~isnan([markerdata(1,:)'; NaN])) < 0)];
            
            for iData = 1:size(dataArray, 1); % loop all the gaps
                if dataArray(iData,2) - dataArray(iData,1) > orderLowpass * 3 % length data must be 3 times the filter order
                    markerdataFilt(:, dataArray(iData,1) : dataArray(iData,2)) = ...
                        filtfilt(B, A, markerdata(:, dataArray(iData,1) : dataArray(iData,2))')';
                end
            end
            
        end
        
        function rawMarkerData = filterMarkerData(rawMarkerData,maxGap,maxVeljump,...
                freqLowPass,orderLowpass)
            %FILTERMARKERDATA
            % maxGap (10):
            % maximal number of frames that are allowed to be interpolated
            % maxVeljump (0.1):
            % maximal acceleration that is allowed for markers, otherwise markers are
            % considdered as outliers  % [m/s]
            % freqLowPass (15):
            % the low pass filter frequency [Hz]
            % orderLowpass (4):
            % order of the filter
            
            
            time = rawMarkerData(:,4);
            for i = 1:3
                rawMarkerData(:,i) = Markers3D.filterAxis(...
                    rawMarkerData(:,i),time,maxGap,maxVeljump,...
                    freqLowPass,orderLowpass);
            end
            
            
        end
        
        
        function [yesOrNo] = areTheMarkersWellSpaced(marker1,...
                marker2,marker3,varargin)
            %ARETHEMARKERSWELLSPACED Tests to see if the markers
            %are in a good triangle. To test for dropped markers
            %or bad measurements.
            
            p = inputParser;
            addOptional(p,'lengthThreshold',14.55000001);
            parse(p,varargin{:});
            lengthThreshold = p.Results.lengthThreshold;
            
            
            RBLB = norm(marker1(1,1:3) - marker2(1,1:3));
            RBFT = norm(marker1(1,1:3) - marker3(1,1:3));
            FTLB = norm(marker3(1,1:3) - marker2(1,1:3));
            yesOrNo = false;
            
            if (any(isnan(marker1(1,1:3)))) || ...
                    (any(isnan(marker2(1,1:3)))) || ...
                    (any(isnan(marker3(1,1:3))))
                display('All Markers dropped - NaN')
                return
            elseif (any(marker1(1,1:3)==0)) || ...
                    (any(marker2(1,1:3)==0)) || ...
                    (any(marker3(1,1:3)==0))
                display('All Markers dropped -zeroes')
                return
            elseif (abs(RBLB - RBFT) > lengthThreshold) || ...
                    (abs(RBLB - FTLB) > lengthThreshold) || ...
                    (abs(FTLB - RBFT) > lengthThreshold)
                display('All Markers dropped -not a triangle')
                return
            end
            yesOrNo=true;
        end
        
        
        function [markeroutliers] = findOutliers(markerdata,...
                maxVeljump, Fs)
            %FINDOUTLIERS finds the gaps and outliers in a markerdata array
            % The array has size: (N,1)
            % maxGap is the maximum gap between two values
            % maxVeljump the maximum velocity jump
            % the output array, markeroutliers contains NaNs at the outlier
            % positions.
            
            dt = 1/Fs;
            %% find outliers
            markerdata = markerdata'; % make it a row array
            jumpLeft = markerdata - markerdata(:, [2 1:end-1]); % derivative on the left side
            jumpRight = markerdata(:, [2:end end-1]) - markerdata; % derivetive on the right side
            
            isJumpLeft = any(~(abs(jumpLeft) < maxVeljump*dt)); % jump on the leftside ~ is used to also include NaN as positive result
            isJumpright = any(~(abs(jumpRight) < maxVeljump*dt)); % jump on the right side
            isOpposite = any(sign(jumpLeft) ~= sign(jumpRight)); % jumps are opposite
            
            markerdata(:, isJumpLeft & isJumpright & isOpposite) = NaN;
            
            %% Set all exact zeroes to NaN
            markerdata(markerdata==0)=NaN;
            
            markeroutliers = markerdata;
        end
        
        
        
        
        function [filMarkers,gapArrayMarker] = fillGaps(markerOutliers,...
                maxGap, interpMethod)
            %% FILLGAPS finds and fills gaps
            % finds the gaps in the data and fill the gaps where possible
            % the gaps are marked with a NaN
            nFrames = length(markerOutliers);
            % make a n x 2 matrix containg on every row the beginning and the ending of
            % a gap.
            gapArrayMarker = [find(diff(isnan([0; markerOutliers(1,:)'])) > 0) ...
                find(diff(isnan([markerOutliers(1,:)'; 0])) < 0)];
            % markerOutliers = markerOutliers'
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
                        filMarkers = markerOutliers;
        end
        
        function [vtm_t,gapArray] = dropQuaternions(rightBack,leftBack,front,N)
            %MARKED MISSING MARKERS
            CountDroppedMarkers = 0;
            [rightBack,leftBack,front] = Markers3D.eraseNan(rightBack,leftBack,front);
            
            display('Check if the markers are well spaced');
            
            
            %     rightBack
            %             size(rightBack)
            %             size(leftBack)
            %             size(front)
            %             N
            parfor i = 1:N
                if ~Markers3D.areTheMarkersWellSpaced(rightBack(i,1:3),...
                        leftBack(i,1:3),...
                        front(i,1:3));
                    vtm_t{i} = -1;
                    CountDroppedMarkers = CountDroppedMarkers +1;
                else
                    vtm = Markers3D(rightBack(i,1:3),...
                        leftBack(i,1:3),front(i,1:3),rightBack(i,4:4));
                    vtm_t{i} = vtm;
                end
            end
            display(['Dropped Quaternions =', num2str(CountDroppedMarkers)]);
            if ((CountDroppedMarkers/N)*100) > 5
                warning('Markers3D:readDataVicon','Dropped Quaternions is more than 5 percent');
            end
            %REMOVE THE MARKED MARKERS
            % If it is a class put it into the array, otherwise not (if it
            % is -1)
            vtm_tmp = {};
            gapArray = [];
            
            for i = 1:N
                if isa(vtm_t{i},'Markers3D')
                    vtm_tmp = {vtm_tmp{:} vtm_t{i}};
                else
                    gapArray = [gapArray, i];
                end
            end
            vtm_t = vtm_tmp;
            display(['Array dropped Quaternions =', num2str(gapArray)]);
        end
        
        
        
        function [filMarkers,gapArrayMarker] = findFillGaps(markerdata,...
                maxGap, maxVeljump,Fs)
            %FINDFILLGAPS loops over the three coordinates of
            %the markerdata, finds the gaps and outliers and
            %fills the gaps, using interpolation
            %Input is a [Nx3] matrix
            
            filMarkers = zeros(length(markerdata(:,1)),3);
            for i = 1:3
                markerOutliers = Markers3D.findOutliers(markerdata(:,i),...
                    maxVeljump, Fs);
                [filMarkers(:,i),gapArrayMarker] = Markers3D.fillGaps(markerOutliers,...
                    maxGap,'linear');
            end

        end
        
        function [marker1n,marker2n,marker3n] = eraseNan(marker1,marker2,marker3)
            N=size(marker1,1);
            for i = 1:N
                if (any(isnan(marker1(i,1:4)))) || ...
                        (any(isnan(marker2(i,1:4)))) || ...
                        (any(isnan(marker3(i,1:4))))
                    marker1n(i,1:4) =0;
                    marker2n(i,1:4) =0;
                    marker3n(i,1:4) =0;
                else   marker1n(i,:) = marker1(i,:);
                    marker2n(i,:) = marker2(i,:);
                    marker3n(i,:) = marker3(i,:);
                end
            end
        end
    end
    
    methods
        function rw = RawMarkers(a,b)
            rw.a=a;
            rw.b=b;
        end
    end
    
end

