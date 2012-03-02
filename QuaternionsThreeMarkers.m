classdef QuaternionsThreeMarkers < ThreeMarkers
    methods            
        function qtm = QuaternionsThreeMarkers(quaternion,timestamp)
            qtm.H_0_T = quaternion2matrix(quaternion);
            qtm.quaternion = quaternion;
            qtm.points_T = qtm.H_0_T*qtm.points_0;
            qtm.timestamp = timestamp;
        end
    end 
end

