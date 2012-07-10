classdef QuaternionsThreeMarkers < ThreeMarkers
    %QuaternionsThreeMarkers Contains static data to help with the reading
    %of data from InertiaTechnology sensors using the SOFIE-HDF-FORMAT
    %library.
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
                if length(sync)==length(q)
                sync = resample(sync,Fs_wanted,Fs_recorded);
                end
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
        
        function [qtm_t,sync] = readDataRaw(filename,...
                runName,Fs_wanted,Fs_recorded)
            %READDATA Read the quaternion data. This reads from a raw
            %recording file. Ie with quat1 quat2 quat3 quat4 and
            %timestamp.
            [qtm_t,sync] = QuaternionsThreeMarkers.readDataPromove(...
                filename,...
                runName,-1,Fs_wanted,Fs_recorded);
        end
        
    end
    
    methods
        function qtm = QuaternionsThreeMarkers(quaternionTimestamp)
            %QuaternionsThreeMarkers(quaternionTimestamp) - The same as for
            %ThreeMarkers,except that the time stamp is also recored as the
            %5 element in the quaternion vector.
            quaternion = quaternionTimestamp(1:4);
            timestamp = quaternionTimestamp(5);
            qtm@ThreeMarkers(quaternion)
            qtm.timestamp = timestamp;
        end
    end
end

