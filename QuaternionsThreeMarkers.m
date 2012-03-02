classdef QuaternionsThreeMarkers < handle
    %QuaternionThreeMarkers 
    properties
        timestamp = 0;
        points_psi_i = zeros(4);
        H_psi_1_psi_i = zeros(4);
        quaternion = [0 0 0 0];
    end
    properties (Constant)
        points_psi_1 = [ 
                1 -1 0 0; 
                0 0 1 0;
                0 0 0 1; 
                1 1 1 1];
    end
        
    methods
        function plot_i(qtm)
            qtm.plot(qtm.points_psi_i,'--m');
        end
        function plot_1(qtm)
            qtm.plot(qtm.points_psi_1,'--m');
        end
              
        function qtm = QuaternionsThreeMarkers(quaternion,timestamp)
            qtm.H_psi_1_psi_i = quaternion2matrix(quaternion);
            qtm.quaternion = quaternion;
            qtm.points_psi_i = qtm.H_psi_1_psi_i*qtm.points_psi_1;
            qtm.timestamp = timestamp;
        end
    end 
end

