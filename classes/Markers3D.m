classdef Markers3D < ThreeD
    %Markers3D Handles an equilateral triangle configuration
    % of Vicon markers or any other marker systems.
    % maxGap:
    % maximal number of frames that are allowed to be interpolated
    % maxVeljump:
    % maximal acceleration that is allowed for markers, otherwise markers are
    % considdered as outliers  % [m/s]
    methods (Static)
        
        
        function vtm_t = create3DMarkersFromRawData(rawData,varargin)
            %CREATES3DMARKERFROMRAWDATA
            % Creates the 3D markers from the raw data and
            % returns a cell containing all the data.
            %
            % OPTIONAL PARAMETERS
            %  adams - Then the ThreeD.default_points_0 is used.
            % absoluteOrientationMethod - @see ThreeD.Markers3D
            
            %Parameter Parsing.
            p = inputParser;
            addOptional(p,'adams',false);
            addOptional(p,'absoluteOrientationMethod',false)
            parse(p,varargin{:});
            absoluteOrientationMethod = p.Results.absoluteOrientationMethod;
            adams = p.Results.adams;
            
            if adams
                display('Using X_BASE_FRAME');
                points_0 = ThreeD.x_base_frame
            else
                points_0 = ThreeD.default_points_0;
            end
            
            N=size(rawData,1);
            t = rawData(:,10);
            vtm_t = cell(1,N);
            parfor i = 1:N
                vtm = Markers3D('rightBack',rawData(i,1:3),...
                    'leftBack',rawData(i,4:6),...
                    'front',rawData(i,7:9),...
                    'timeStamp',t(i),...
                    'points_0',points_0,...
                    'absoluteOrientationMethod',absoluteOrientationMethod);
                
                vtm_t{i} = vtm;
            end
        end
    end
    
    
    methods
        function vtm = Markers3D(varargin)
            %VICONTHREEMARKERS(rightback,leftback,front,timestamp)
            %Calculates and creates the quaternion of the plane represented
            %by the three markers. 
            % PARAMETERS:
            %   rightback,leftback and front are Nx3
            %   matrices containg the x,y and z measurements of the
            %   point/marker in 3D space. Front is on the positive Y axis,
            %   rightback and leftback are on the X axis.
            % Optional PARAMETERS:
            %   points_0 - The zero reference frame for the absolute
            %      orientation calculation.
            %   absoluteOrientationMethod - The method to use to calculate
            %      the absolute orientation. Options are 'screw','kabsch'.
            %      Default uses absor.
            %
            % RETURN A 3D marker.
            
            %Parameter Parsing.
            p = inputParser;
            p.addParamValue('rightback',-1,@isvector);
            p.addParamValue('leftback',-1,@isvector);
            p.addParamValue('front',-1,@isvector);
            p.addParamValue('timeStamp',-1,@isvector);

            addOptional(p,'points_0',ThreeD.default_points_0);
            addOptional(p,'absoluteOrientationMethod',false)
            parse(p,varargin{:});
            points_0 = p.Results.points_0;
            absoluteOrientationMethod = p.Results.absoluteOrientationMethod;
            rightback  = ...
                p.Results.rightback;
            leftback  = ...
                p.Results.leftback;
            front = ...
                p.Results.front;
            timeStamp = ...
                p.Results.timeStamp;
    
            midpoint = (rightback+leftback)/2
            front = ThreeD.normWithOffset(front,midpoint)
            rightback = ThreeD.normWithOffset(rightback,midpoint)
            leftback = ThreeD.normWithOffset(leftback,midpoint)
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
       
            if (strcmp(absoluteOrientationMethod,'screw')==1)
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
            elseif (strcmp(absoluteOrientationMethod,'kabsch')==1)
                    display(['KABSCH:' absoluteOrientationMethod])
                    [H_0_T] = Kabsch(points_0(1:3,:),...
                        points_T(1:3,:));
                    H_0_T(4,1:3)=[0 0 0];
                    H_0_T(:,4)=[0 0 0 1]';
            else
                [rotInfo] = absor(points_0(1:3,1:4),...
                    points_T(1:3,1:4));
                H_0_T = rotInfo.q';
            end
            vtm@ThreeD(H_0_T);
            vtm.timestamp = timeStamp;
        end
    end
end

