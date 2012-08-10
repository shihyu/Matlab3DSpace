function rmserrorplot(A,B,theTitle,plotDifference)
%RMSERRORPLOT Plots a standard RMS error box plot used to see how big the
%error between the orginal and the normal is.
%Arguments: A,B the cell of vectors that must have each value compared to
%it.
%theTitle: The title for the plot.
%plotDifference: Plot the differences.
RMSVector =[];
RMSVerschilLengte = [];
IdentificationVector = [];
RMSVerschilVector = [];
theXLabel = ['Measurement number: '];
for j = 1:length(A)
    Aj = A{j};
    Bj = B{j};
    minSize = min(length(Aj),length(Bj));
    Verschil =  Aj(1:minSize)-Bj(1:minSize);
    RMS = sqrt(mean(Verschil.^2));
    RMSVector = [RMSVector, RMS];
    RMSVerschil = abs(Verschil-RMS)';
    RMSVerschilLengte = [RMSVerschilLengte, numel(RMSVerschil)];
    RMSVerschilVector = [RMSVerschilVector; RMSVerschil];
    Identification = [repmat(j,RMSVerschilLengte(j),1)];
    IdentificationVector = [IdentificationVector; Identification];
    theXLabel = [theXLabel ' No:' num2str(j) ': RMS mean:' num2str(RMS) ]; 
    if plotDifference==true
        figure('visible','on','WindowStyle','docked',...
                'Name',[theTitle ' - DATA PLOT' num2str(j)]);
       subplot(2,1,1);
       plot(Aj);
       hold on;
       title([theTitle ' A and B PLOT(red)']);
       subplot(2,1,1);
       plot(Bj,'r--');
       hold off;
       subplot(2,1,2);
       plot(Verschil);
       title([theTitle ' Error PLOT']);
       
    end
end
figure('visible','on','WindowStyle','docked',...
                'Name',theTitle);
boxplot(RMSVerschilVector, IdentificationVector, 'notch', 'on')
xlabel (theXLabel),
ylabel('absolute RMS difference: '),
title (theTitle);
