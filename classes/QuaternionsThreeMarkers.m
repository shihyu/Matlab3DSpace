classdef QuaternionsThreeMarkers < ThreeMarkers
    methods (Static)
        
        function [qtm_t metrics] = readData(filename,...
                runName,nodeId,Fs_wanted,Fs_recorded)
            %READDATA Reads the VICON three markers in
            % and creates the ViconThreeMarker object.
            reader = promoveReader(filename,runName);
            q = reader.readNodeData(nodeId);
            if Fs_wanted ~= Fs_recorded
                q = resample(q(:,1:5),Fs_wanted,Fs_recorded);
            end
            qtm_t = [];
            %display(size(q,1))
            N=size(q,1);
            metrics = zeros(1,N);
            hdiff = zeros(4,4);

            parfor i = 1:size(q,1)
                qtm = QuaternionsThreeMarkers(q(i,:))
                qtm_t =[qtm_t qtm];
                %metrics(i) = qtm.calculateRotDiff(qtm.getH(),hdiff);
                %hdiff = qtm.getH();
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

