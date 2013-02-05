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
