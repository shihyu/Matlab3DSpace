classdef ViconThreeMarkers < ThreeMarkers
    %ViconThreeMarkers Creates an object contains all the marker in the
    %local frame, and the marker transfered to the global frame
    %with medthods to get the H matrix between them ie H_T_0.
    methods
        function vtm = ViconThreeMarkers(rightback,leftback,...
            front,timestamp)
            vtm.timestamp = timestamp;
            midpoint = (rightback+leftback)/2;
            front = vtm.normWithOffset(front,midpoint);
            rightback = vtm.normWithOffset(rightback,midpoint);
            leftback = vtm.normWithOffset(leftback,midpoint);
            crosspointTmp = cross(front-midpoint,...
                    leftback-midpoint)+midpoint;
            crosspoint = vtm.normWithOffset(crosspointTmp,midpoint);
            %Create the normalized matrix of the points.
            vtm.points_T = [ rightback;
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
            vtm.points_T(4,:) = 1;
            %Get the homogenous matrix for these points
            H_T_0 = vtm.points_0/vtm.points_T;
            %Remove translation
            H_T_0(1:3,4)  = [0 0 0]';
            %and error
            H_T_0(4,1:3)  = [0 0 0];
            vtm.H_0_T = H_T_0';
            vtm.quaternion = matrix2quaternion(vtm.H_0_T)';
            %Recalculate position with no translation added to screw.
            vtm.points_T = H_T_0\vtm.points_0;

        end
    end 
end

