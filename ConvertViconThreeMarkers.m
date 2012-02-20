classdef ConvertViconThreeMarkers < handle
    %CONVERTVICONThreeMarkers Creates an object contains all the marker in the
    %local frame, and the marker transfered to the global frame
    %with medthods to get the H matrix between them ie H_psi_v_psi_0.
    properties
        timestamp = 0;
        points_psi_v = zeros(4);
        H_psi_v_psi_0 = zeros(4);
    end
    properties (Constant)
        points_psi_0 = [ 
                1 -1 0 0; 
                0 0 1 0;
                0 0 0 1; 
                1 1 1 1];
    end
    methods (Access = private,Static)
      function plot(point,style)
          plotSensor(point,style);
      end
      
      function [normedPoint] = normWithOffset(point,reference)
           normedPoint = (point-reference)/norm(point-reference)+reference;
      end
    end
    
    methods (Static)
         function angle = getAngle(marker1,marker2,zeropoint)
            angle = acos(dot(marker1-zeropoint,...
                marker2-zeropoint)/...
                (norm(marker1-zeropoint)*norm(marker2-zeropoint)));           
        end
    end
    
    methods
        
        function rotmat = getRotationMatrix(cvt)
            rotmat = cvt.H_psi_v_psi_0(1:3,1:3);
        end
        
        function plot_v(cvt)
            cvt.plot(cvt.points_psi_v,'--k');
        end
        function plot_0(cvt)
            cvt.plot(cvt.points_psi_0,'--m');
        end
              
        function cvt = ConvertViconThreeMarkers(rightback,leftback,...
            front,timestamp)
            
            cvt.timestamp = timestamp;
            midpoint = (rightback+leftback)/2;
            front = cvt.normWithOffset(front,midpoint);
            rightback = cvt.normWithOffset(rightback,midpoint);
            leftback = cvt.normWithOffset(leftback,midpoint);
            crosspointTmp = cross(front-midpoint,...
                    leftback-midpoint)+midpoint;
            crosspoint = cvt.normWithOffset(crosspointTmp,midpoint);
            %Create the normalized matrix of the points.
            cvt.points_psi_v = [ rightback;
                                 leftback;
                                 front;
                                 crosspoint]';

            %not a perfect 60 degree Triangle... so rotate on the 
            %XY plane to get the
            %actual front marker in the zero frame.
            %TODO test this transform.
            %rotAngle = cvt.getAngle(cvt.front,cvt.leftback,cvt.midpoint);
            %yValue = cvt.points_psi_0(1,1:2)*[cos(rotAngle) -...
            %     sin(rotAngle); sin(rotAngle) cos(rotAngle)];
            %cvt.points_psi_0(3,1:2) = yValue;

            %Create the screw theory compliant points for Vikon
            cvt.points_psi_v(4,:) = 1;
            %Get the homogenous matrix for these points
            cvt.H_psi_v_psi_0 = cvt.points_psi_0/cvt.points_psi_v;
            %Remove translation
            cvt.H_psi_v_psi_0(1:3,4)  = [0 0 0]';
            %and error
            cvt.H_psi_v_psi_0(4,1:3)  = [0 0 0];
            %Recalculate position with no translation added to screw.
            cvt.points_psi_v = cvt.H_psi_v_psi_0\cvt.points_psi_0;

        end
    end 
end

