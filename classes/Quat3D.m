classdef Quat3D < ThreeD
    %Quat3D Works with Quaternions to create ThreeD objects.
    methods (Static)
        
        function [qtm_t,sync] = readDataSHDF(filename,...
                runName,varargin)
            %READDATA Read the quaternion data. This is static function is
            %custom for reading data from the SOFIHDFFORMAT file format.
            %The table must contain quat1,quat2,quat3 and quat4 columns
            %
            %OPTIONAL PARAMTERS:
            %   nodeId -set this to read a specific node ID. default reads
            %            all.
            %   Fs_wanted,Fs_recorded - Set both to resample the data.
            %data in the file as if it was from one node.
            p = inputParser;
            p.addOptional('nodeId',-1);
            p.addOptional('Fs_wanted',-1);
            p.addOptional('Fs_recorded',-1);
            p.parse(varargin{:});
            
            nodeId = ...
                p.Results.nodeId ;
            Fs_wanted= ...
                p.Results.Fs_wanted ;
            Fs_recorded = ...
                p.Results.Fs_recorded ;
            
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

