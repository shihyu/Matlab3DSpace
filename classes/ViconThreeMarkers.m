classdef ViconThreeMarkers < ThreeMarkers
    %VICONTHREEMARKERS Handles an equilateral triangle configuration
    % of Vicon markers or any other marker systems.
    
    methods (Static)
        function [vtm_t] = readDataVicon(filename,runName,...
                rightBackName,leftBackName,frontName,varargin)
            %READDATA Reads the VICON three markers in
            % and creates the ViconThreeMarker object. The option value of
            % 'kabsch' or 'screw' can be added to use a different method to
            % estimate the rotation matrices.
            reader = c3dReader(filename,runName)
            rightBack = reader.readMarker(rightBackName);
            leftBack = reader.readMarker(leftBackName);
            front = reader.readMarker(frontName);
%             display('Right Back')
%             display(size(rightBack));
%             display('Left Back')
%             display(size(leftBack));
%             display('Front')
%             display(size(front));
            N=size(rightBack,1);
            vtm_t = cell(1,N);
            parfor i = 1:N
%                 i
                vtm = ViconThreeMarkers(rightBack(i,1:3),...
                    leftBack(i,1:3),front(i,1:3),rightBack(i,4),varargin);
                vtm_t{i} = vtm;
            end
        end
        
        function [vtm_t,Fs] = readDataAdams(filename,runName,...
                rightBackName,leftBackName,frontName,varargin)
            %READDATA Reads the ADAMS export file three markers in
            % and creates the ViconThreeMarker object.
            % A 'Time' column must be present.
            reader = adamsReader(filename,runName);
            data = reader.readData(false);
            t=data.Time;
            %HACK, must generalise the readData function for adamsReader.
            if strcmp(rightBackName,'RBO')||strcmp(rightBackName,'RB0')
                display('Reading RBO')
                RBO = [data.RBOX data.RBOY data.RBOZ];
                LBO = [data.LBOX data.LBOY data.LBOZ];
                FON = [data.FONX data.FONY data.FONZ];
            else
                display('Reading RBT')
                RBO = [data.RBTX data.RBTY data.RBTZ];
                LBO = [data.LBTX data.LBTY data.LBTZ];
                FON = [data.FTNX data.FTNY data.FTNZ];
            end
            N=size(t,1);
            Fs = 1/(t(2)-t(1));
            vtm_t = cell(1,N);
            parfor i = 1:N
                vtm = ViconThreeMarkers(RBO(i,1:3),...
                    LBO(i,1:3),FON(i,1:3),t(i),varargin);
                vtm_t{i} = vtm;
            end
        end
        
    end
    
    
    methods
        function vtm = ViconThreeMarkers(rightback,leftback,...
                front,timestamp,varargin)
            %VICONTHREEMARKERS(rightback,leftback,front,timestamp)
            %Calculates and creates the quaternion of the plane represented
            %by the three markers. rightback,leftback and front are Nx3
            %matrices containg the x,y and z measurements of the
            %point/marker in 3D space. Front is on the positive Y axis,
            %rightback and leftback are on the X axis.
            midpoint = (rightback+leftback)/2;
            front = ThreeMarkers.normWithOffset(front,midpoint);
            rightback = ThreeMarkers.normWithOffset(rightback,midpoint);
            leftback = ThreeMarkers.normWithOffset(leftback,midpoint);
            crosspointTmp = cross(front-midpoint,...
                leftback-midpoint)+midpoint;
            crosspoint = ThreeMarkers.normWithOffset(crosspointTmp,midpoint);
            %not a perfect 60 degree Triangle... so rotate on the
            %XY plane to get the
            %actual front marker in the zero frame.
            %TODO test this transform.
            %              rotAngle = ThreeMarkers.getAngle(front,...
            %                  leftback,...
            %                  0);
            %              yValue = our_point_0(3,1:2)*[cos(rotAngle-pi/2) -...
            %                  sin(rotAngle-pi/2); sin(rotAngle-pi/2) cos(rotAngle-pi/2)];
            %              our_point_0(3,1:2) = yValue;
            %Create the normalized matrix of the points.
            points_T = [ rightback;
                leftback;
                front;
                crosspoint]';
            setQuaternion = false;
            if ((~isempty(varargin))&&(~isempty(varargin{1})))
                if (strcmp(varargin{1},'screw')==1)
                    %Create the screw theory compliant points for Vikon
                    points_T(4,:) = 1;
                    %Get the homogenous matrix for these points
                    H_T_0 = ThreeMarkers.points_0/points_T;
                    %Remove translation
                    H_T_0(1:3,4)  = [0 0 0]';
                    %and error
                    H_T_0(4,1:3)  = [0 0 0];
                    %H_0_T = H_T_0';
                    H_0_T = invht(H_T_0);
                elseif (strcmp(varargin{1},'kabsch')==1)
                    %display(['HORN:' varargin{1}])
                    [H_0_T] = Kabsch(ThreeMarkers.points_0(1:3,:),...
                        points_T);
                    H_0_T(4,1:3)=[0 0 0];
                    H_0_T(:,4)=[0 0 0 1]';
                end
            else
%                 points_T
                %display(['KABSCH:' varargin{1}])
%                 ThreeMarkers.points_0(1:3,:)
%                 points_T
                [rotInfo] = absor(ThreeMarkers.points_0(1:3,:),...
                    points_T);
                H_0_T = rotInfo.M;
                H_0_T(4,1:3)=[0 0 0];
                H_0_T(:,4)=[0 0 0 1]';
                setQuaternion = true;
            end
            vtm@ThreeMarkers(H_0_T);
            vtm.timestamp = timestamp;
%             if setQuaternion
%                 %display('SETTING Q');
%                 vtm.quaternion = rotInfo.q';
%             end
        end
    end
end

