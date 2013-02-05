%DEMO on how to use the matlab3dspace software.
%Specify the runName and filename
runName = '/demo';
filename = 'matlab3dspace-presentation.h5';
%The most low leve way to read data is like this:
reader = c3dReader(filename,runName)
marker1 = reader.readMarker('RBT')
%Higher Level Interface:
rawData = RawMarkers.readFromFile(filename,runName,'RBT','LBT','FTN')
%Is it good data?
[yesOrNo,distances]=RawMarkers.areTheMarkersWellSpaced(rawData,...
    'plotBoxPlot',true,...
        'plotTheDistances',true);
%Lets see what the final results looks like
data_2 = Markers3D.create3DMarkersFromRawData(rawData);
rawData = RawMarkers.readFromFile(filename,runName,'RBO','LBO','FON')
data_1 = Markers3D.create3DMarkersFromRawData(rawData);
%And plot the run.
ThreeD.plotRun([data_1;data_2])