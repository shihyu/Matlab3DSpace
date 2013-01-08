classdef RawMarkers
    %RAWMARKERS This class is used to package all the reading
    % of raw C3D Three Marker data from SofieHDFFormat.
    % Various proxessing
    methods(Static)
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
           
            RBLB = norm(rawData(:,1:3) - rawData(:,4:6));
            RBFT = norm(rawData(:,1:3) - rawData(:,7:9));
            FTLB = norm(rawData(:,7:9) - rawData(:,4:6));
            
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
    
        function [rawData] = readFromC3DFile(filename,runName,...
                rightBackName,leftBackName,frontName)
            %READFROMFILE Read the raw data from a file
            %RETURNS rawData = [rightback,leftback,front,t];
            reader = c3dReader(filename,runName)
            rightBack = reader.readMarker(rightBackName);
            leftBack = reader.readMarker(leftBackName);
            front = reader.readMarker(frontName);
            rawData = [rightBack(:,1:3),leftBack(:,1:3),...
                front(:,1:3),front(:,4)];
        end
        
        function [rawData] = readFromAdamsFile(filename,runName,...
                rightBackName,leftBackName,frontName)
            %READDATAADAMS Reads the ADAMS export file three markers in
            % A 'Time' column must be present.
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
            rawData = [RBO,LBO,FON,t'];
        end
    end
end

