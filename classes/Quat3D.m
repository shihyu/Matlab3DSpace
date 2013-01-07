classdef Quat3D < ThreeD
    %Quat3D Contains static data to help with the reading
    %of data from Quaternion data stored in the SFIE-HDF-FORMAT.
    methods (Static)
        
        function [qtm_t,sync] = readDataPromove(filename,...
                runName,nodeId,Fs_wanted,Fs_recorded)
            %READDATA Read the quaternion data. This is static function is
            %custom for CSV files that Intertia Technology outputs from
            %their ProMoveGUI. Set nodeId to -1 if you must read all the
            %data in the file as if it was from one node.
            reader = promoveReader(filename,runName);
            [q,sync] = reader.readNodeData(nodeId);
            if isempty(q)
                error('Quat3D:readData',...
                    'The Node does not have any values to read.');
            end
            t = q(:,5);
            [t,q] = ThreeD.sortAccordingToTimestamp(t,q);
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
                qtm = Quat3D(q(i,:))
                qtm_t{i} = qtm;
            end
        end
        
        function [qtm_t,sync] = readDataRaw(filename,...
                runName,Fs_wanted,Fs_recorded)
            %READDATA Read the quaternion data. This reads from a raw
            %recording file. Ie with quat1 quat2 quat3 quat4 and
            %timestamp.
            [qtm_t,sync] = Quat3D.readDataPromove(...
                filename,...
                runName,-1,Fs_wanted,Fs_recorded);
        end
        
    end
    
    methods
        function qtm = Quat3D(quaternionTimestamp)
            %Quat3D(quaternionTimestamp) - The same as for
            %ThreeD,except that the time stamp is also recored as the
            %5 element in the quaternion vector.
            quaternion = quaternionTimestamp(1:4);
            timestamp = quaternionTimestamp(5);
            qtm@ThreeD(quaternion)
            qtm.timestamp = timestamp;
        end
    end
end

