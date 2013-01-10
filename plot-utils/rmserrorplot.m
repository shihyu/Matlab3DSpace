function [RMSVector,PCoeffVector,...
    VerschilVector,theFigure]=rmserrorplot(A,B,theTitle,plotDifference)
%RMSERRORPLOT Plots a standard RMS error box plot used to see how big the
%error between the orginal and the normal is.
%Arguments: A,B the cell of vectors that must have each value compared to
%it.
%theTitle: The title for the plot.
%plotDifference: Plot the differences.
RMSVector =[];
PCoeffVector = [];
RMSVerschilLengte = [];
IdentificationVector = [];
RMSVerschilVector = [];
VerschilVector = cell(1,length(A));
theXLabel = [];
for j = 1:length(A)
    Aj = A{j};
    Bj = B{j};
    minSize = min(length(Aj),length(Bj));
    Verschil =  Aj(1:minSize)-Bj(1:minSize);
    RMS = sqrt(mean(Verschil.^2));
    RMSVector = [RMSVector, RMS];
    RMSVerschil = abs(Verschil-RMS)';
    VerschilVector{j} = Verschil;
    RMSVerschilLengte = [RMSVerschilLengte, numel(RMSVerschil)];
    RMSVerschilVector = [RMSVerschilVector; RMSVerschil];
    Identification = [repmat(j,RMSVerschilLengte(j),1)];
    %Calculate the pearsons coefficient.
    C = cov( Aj(1:minSize),Bj(1:minSize));
    PCoeff = C(1,2) / sqrt(C(1,1) * C(2,2));
    PCoeffVector = [PCoeffVector, PCoeff];
    IdentificationVector = [IdentificationVector; Identification];
    theXLabel = [theXLabel ' ' num2str(j) '.RMS:' num2str(RMS,4) ...
        ' PC: ' num2str(PCoeff,4)]; 
    if plotDifference==true
       figure('visible','on','WindowStyle','docked',...
                'Name',[theTitle ' - DATA PLOT' num2str(j)]);
       subplot(2,1,1);
       plot(Aj,'b--.');
       hold on;
       title([theTitle ' A and B PLOT(red)']);
       subplot(2,1,1);
       plot(Bj,'r--.');
       hold off;
       subplot(2,1,2);
       plot(Verschil);
       title([theTitle ' Error PLOT']);
       
    end
end
theFigure = figure('visible','on','WindowStyle','docked',...
                'Name',theTitle);
boxplot(RMSVerschilVector, IdentificationVector, 'notch', 'on');
grid on;
xlabel ('Roll,Pitch and Yaw RMS error (deg)'),
ylabel('Absolute RMS difference: '),
title (theTitle);
