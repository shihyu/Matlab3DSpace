function [ quaternions, rbt,lbt,fot] = getdata( filename,run, node)
%GETDATA Summary of this function goes here
%   Detailed explanation goes here
reader = h5Reader(filename,[ run '/csvimport']);
promoveData = reader.readLeafData('CsvImportData',false);
quaternions =  [promoveData.quat1 promoveData.quat2 promoveData.quat3 ...
    promoveData.quat4 promoveData.timestamp];
%display(['Reader currentPos',reader.getRunName()])
reader = reader.setRunName(['/' run '/c3d/'])
%display(['Reader currentPos',reader.getRunName()])
if node == 2
    rbt = reader.readLeafData('C3DMarkerCol_RBT');
    lbt = reader.readLeafData('C3DMarkerCol_LBT');
    fot = reader.readLeafData('C3DMarkerCol_FTN');
else
    rbt = reader.readLeafData('C3DMarkerCol_RBO');
    lbt = reader.readLeafData('C3DMarkerCol_LBO');
    fot = reader.readLeafData('C3DMarkerCol_FON');
end

