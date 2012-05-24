classdef QuaternionsThreeMarkers < ThreeMarkers
    methods (Static)
        
        function [qtm_t,sync] = readDataPromove(filename,...
                runName,nodeId,Fs_wanted,Fs_recorded)
            %READDATA Read the quaternion data. This is static function is custom
            %for CSV files that Intertia Technology outputs from their
            %ProMoveGUI.
            reader = promoveReader(filename,runName);
            [q,sync] = reader.readNodeData(nodeId);
            if isempty(q)
                error('QuaternionsThreeMarkers:readData',...
                    'The Node does not have any values to read.');
            end
            if Fs_wanted ~= Fs_recorded
                sync = resample(sync,Fs_wanted,Fs_recorded);
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

