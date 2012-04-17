classdef ViconThreeMarkers < ThreeMarkers
    %VICONTHREEMARKERS Handles an equilateral triangle configuration
    % of Vicon markers.
    methods
        function vtm = ViconThreeMarkers(rightback,leftback,...
                front,timestamp)
            timestamp = timestamp;
            midpoint = (rightback+leftback)/2;
            front = ThreeMarkers.normWithOffset(front,midpoint);
            rightback = ThreeMarkers.normWithOffset(rightback,midpoint);
            leftback = ThreeMarkers.normWithOffset(leftback,midpoint);
            crosspointTmp = cross(front-midpoint,...
                leftback-midpoint)+midpoint;
            crosspoint = ThreeMarkers.normWithOffset(crosspointTmp,midpoint);
            %Create the normalized matrix of the points.
            points_T = [ rightback;
                leftback;
                front;
                crosspoint]';
            
            %not a perfect 60 degree Triangle... so rotate on the
            %XY plane to get the
            %actual front marker in the zero frame.
            %TODO test this transform.
            %rotAngle = vtm.getAngle(vtm.front,vtm.leftback,vtm.midpoint);
            %yValue = vtm.points_psi_0(1,1:2)*[cos(rotAngle) -...
            %     sin(rotAngle); sin(rotAngle) cos(rotAngle)];
            %vtm.points_psi_0(3,1:2) = yValue;
            
            %Create the screw theory compliant points for Vikon
            points_T(4,:) = 1;
            %Get the homogenous matrix for these points
            H_T_0 = ThreeMarkers.points_0/points_T;
            %Remove translation
            H_T_0(1:3,4)  = [0 0 0]';
            %and error
            H_T_0(4,1:3)  = [0 0 0];
            H_0_T = H_T_0';
            quaternion = matrix2quaternion(H_0_T)';
            vtm@ThreeMarkers(quaternion)
            vtm.timestamp = timestamp;
        end
    end
end

