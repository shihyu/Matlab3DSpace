classdef QuaternionsThreeMarkers < ThreeMarkers
    methods (Static)
        
        function qtm_t = readData(filename,runName,nodeId)
            %READDATA Reads the VICON three markers in
            % and creates the ViconThreeMarker object.
            reader = promoveReader(filename,runName);
            q = reader.readNodeData(nodeId);
            qtm_t = [];
            for i = 1:size(q,1)
                qtm_t =[qtm_t QuaternionsThreeMarkers(q)];
            end
        end
    end
    
    methods
        function qtm = QuaternionsThreeMarkers(quaternionTimestamp)
            quaternion = quaternionTimestamp(1:4);
            timestamp = quaternionTimestamp(5);
            qtm@ThreeMarkers(quaternion)
            qtm.timestamp = timestamp;
        end
    end
end

