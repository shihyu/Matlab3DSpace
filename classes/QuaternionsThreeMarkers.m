classdef QuaternionsThreeMarkers < ThreeMarkers
    methods            
        function qtm = QuaternionsThreeMarkers(quaternionTimestamp)
            quaternion = quaternionTimestamp(1:4);
            timestamp = quaternionTimestamp(5);
            qtm.H_0_T = quaternion2matrix(quaternion);
            qtm.quaternion = quaternion;
            qtm.points_T = qtm.H_0_T*qtm.points_0;
            qtm.timestamp = timestamp;
        end
    end 
end

