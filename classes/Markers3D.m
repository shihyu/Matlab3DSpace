classdef Markers3D < ThreeD
    %Markers3D Handles an equilateral triangle configuration
    % of Vicon markers or any other marker systems.
    % maxGap:
    % maximal number of frames that are allowed to be interpolated
    % maxVeljump:
    % maximal acceleration that is allowed for markers, otherwise markers are
    % considdered as outliers  % [m/s]
    methods (Static)
       
        function [vtm_t,gapArray] = readDataVicon(filename,runName,...
                rightBackName,leftBackName,frontName,varargin)
            %READDATA Reads the VICON three markers in
            % and creates the ViconThreeMarker object.
            %A different point_0 (base frame) can be sent in as the first
            %variable argument for this function.
            %The option value of
            % 'kabsch' or 'screw' can be added to use a different method to
            % estimate the rotation matrices.
            
            %Parameter Parsing.
            p = inputParser;
            addOptional(p,'doFilter',false);
            parse(p,varargin{:});
            doFilter = p.Results.doFilter;
            
            reader = c3dReader(filename,runName)
            rightBack = reader.readMarker(rightBackName);
            leftBack = reader.readMarker(leftBackName);
            front = reader.readMarker(frontName);
            
            
            if doFilter
                display('Find outliers raw Marker Data');
                display('Fill gaps in raw Marker data');
                
                [rightBackGap,rBGapArray] = Markers3D.findFillGaps(rightBack,...
                    50,1, 200);
                [leftBackGap,lBGapArray] = Markers3D.findFillGaps(leftBack,...
                    50,1, 200);
                [frontGap,foGapArray] = Markers3D.findFillGaps(front,...
                    50,1, 200);
                

                N = length(rightBackGap);
                display(['Interpolated markers in RB is =', num2str(size(rBGapArray, 1))]);
                display(['Interpolated markers in LB is =', num2str(size(lBGapArray, 1))]);
                display(['Interpolated markers in FO is =', num2str(size(foGapArray, 1))]);
                InterpolatedMarkers = size(rBGapArray, 1)+size(lBGapArray, 1)+ ...
                size(foGapArray, 1);
            if ((InterpolatedMarkers/(3*N))*100) > 5
                warning('Markers3D:readDataVicon','Interpolated markers in raw data is more than 5 percent');
            end 
                
                %               size(rightBackGap)
                %               size(rightBack)
                %
                %              length(rightBack(:,4))
                rightBack = [rightBackGap,rightBack(:,4)];
                leftBack = [leftBackGap,leftBack(:,4)];
                front = [frontGap,front(:,4)];
                
            end
            %             display('Right Back')
            %             display(size(rightBack));
            %             display('Left Back')
            %             display(size(leftBack));
            %             display('Front')
            %             display(size(front));
            N=size(rightBack,1);
            vtm_t = cell(1,N);
            if ~isempty(varargin)
                varargin = varargin{:};
            end
            
            [vtm_t,gapArray] = Markers3D.dropQuaternions(rightBack,leftBack,front,N);

        end
        
        
        
        function [vtm_t,Fs] = readDataAdams(filename,runName,...
                rightBackName,leftBackName,frontName,varargin)
            %READDATA Reads the ADAMS export file three markers in
            % and creates the ViconThreeMarker object.
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
            
            N=size(t,1);
            Fs = 1/(t(2)-t(1));
            vtm_t = cell(1,N);
            if (~isempty(varargin))
                varargin = varargin{:};
            end
            parfor i = 1:N
                vtm = Markers3D(RBO(i,1:3),...
                    LBO(i,1:3),FON(i,1:3),t(i),varargin);
                vtm_t{i} = vtm;
            end
        end
        
    end
    
    
    methods
        function vtm = Markers3D(rightback,leftback,...
                front,timestamp,varargin)
            %VICONTHREEMARKERS(rightback,leftback,front,timestamp)
            %Calculates and creates the quaternion of the plane represented
            %by the three markers. rightback,leftback and front are Nx3
            %matrices containg the x,y and z measurements of the
            %point/marker in 3D space. Front is on the positive Y axis,
            %rightback and leftback are on the X axis.
            if isempty(rightback)
                error('Markers3D:Markers3D',...
                    'Rightback is empty');
            end
            midpoint = (rightback+leftback)/2;
            front = ThreeD.normWithOffset(front,midpoint);
            rightback = ThreeD.normWithOffset(rightback,midpoint);
            leftback = ThreeD.normWithOffset(leftback,midpoint);
            %n=AXB=>A front , B = left. N=Positive Z axis...
            crosspointTmp = cross(front-midpoint,...
                leftback-midpoint)+midpoint;
            crosspoint = ThreeD.normWithOffset(crosspointTmp,midpoint);
            %not a perfect 60 degree Triangle... so rotate on the
            %XY plane to get the
            %actual front marker in the zero frame.
            %TODO test this transform.
            %              rotAngle = ThreeD.getAngle(front,...
            %                  leftback,...
            %                  0);
            %              yValue = our_point_0(3,1:2)*[cos(rotAngle-pi/2) -...
            %                  sin(rotAngle-pi/2); sin(rotAngle-pi/2) cos(rotAngle-pi/2)];
            %              our_point_0(3,1:2) = yValue;
            %Create the normalized matrix of the points.
            points_T = [ rightback' leftback' front' crosspoint'];
            if ((~isempty(varargin))&&(~isempty(varargin{1})))&&...
                    all(size(varargin{1})==[4 4])
                points_0 = varargin{1};
            else
                points_0 = ThreeD.get0;
            end
            if ((~isempty(varargin))&&(length(varargin)==2)...
                    &&(~isempty(varargin{2})))
                if (strcmp(varargin{2},'screw')==1)
                    %Create the screw theory compliant points for Vikon
                    points_T(4,:) = 1;
                    %Get the homogenous matrix for these points
                    H_T_0 = points_0/points_T;
                    %Remove translation
                    H_T_0(1:3,4)  = [0 0 0]';
                    %and error
                    H_T_0(4,1:3)  = [0 0 0];
                    %H_0_T = H_T_0';
                    H_0_T = invht(H_T_0);
                elseif (strcmp(varargin{2},'kabsch')==1)
                    display(['KABSCH:' varargin{2}])
                    [H_0_T] = Kabsch(points_0(1:3,:),...
                        points_T(1:3,:));
                    H_0_T(4,1:3)=[0 0 0];
                    H_0_T(:,4)=[0 0 0 1]';
                end
            else
                [rotInfo] = absor(points_0(1:3,1:4),...
                    points_T(1:3,1:4));
                H_0_T = rotInfo.q';
            end
            vtm@ThreeD(H_0_T);
            vtm.timestamp = timestamp;
        end
    end
end

