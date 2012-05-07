classdef QuaternionsThreeMarkers < ThreeMarkers
    methods (Static)
        
        function [qtm_t] = readData(filename,...
                runName,nodeId,Fs_wanted,Fs_recorded)
            %READDATA Reads the VICON three markers in
            % and creates the ViconThreeMarker object.
            reader = promoveReader(filename,runName);
            q = reader.readNodeData(nodeId);
            if isempty(q)
                error('QuaternionsThreeMarkers:readData',...
                    'The Node does not have any values to read.');
            end
            if Fs_wanted ~= Fs_recorded
                q = resample(q(:,1:5),Fs_wanted,Fs_recorded);
            end
            
            %display(size(q,1))
            N=size(q,1);
            qtm_t = cell(1,N);
            parfor i = 1:N
                qtm = QuaternionsThreeMarkers(q(i,:))
                qtm_t{i} = qtm;
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

