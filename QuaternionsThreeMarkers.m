classdef QuaternionsThreeMarkers < handle
    %QuaternionThreeMarkers 
    properties
        timestamp = 0;
        points_psi_i = zeros(4);
        H_psi_1_psi_i = zeros(4);
    end
    properties (Constant)
        points_psi_1 = [ 
                1 -1 0 0; 
                0 0 1 0;
                0 0 0 1; 
                1 1 1 1];
    end
    methods (Access = private,Static)
      function plot(point,style)
          plotSensor(point,style);
      end
    end
      
    methods
        function plot_i(cvt)
            cvt.plot(cvt.points_psi_i,'--m');
        end
        function plot_1(cvt)
            cvt.plot(cvt.points_psi_1,'--m');
        end
              
        function cvt = QuaternionsThreeMarkers(quaternion,timestamp)
            cvt.H_psi_1_psi_i = quaternion2matrix(quaternion);
            cvt.points_psi_i = cvt.H_psi_1_psi_i*cvt.points_psi_1;
            cvt.timestamp = timestamp;
        end
    end 
end

