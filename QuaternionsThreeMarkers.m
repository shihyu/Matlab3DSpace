classdef QuaternionsThreeMarkers < handle
    %QuaternionThreeMarkers 
    properties
        timestamp = 0
        points_psi_v = 0
        H_psi_v_psi_0 = 0
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
              
        function cvt = QuaternionsThreeMarkers(quaternion,timestamp)
            
            cvt.timestamp = timestamp;
           

        end
    end 
end

