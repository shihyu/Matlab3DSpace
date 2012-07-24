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
theXLabel = ['Nr. van de meting: '];
for j = 1:length(A)
    Aj = A{j};
    Bj = B{j};
    Verschil =  Aj-Bj;
    RMS = sqrt(mean(Verschil.^2));
    RMSVector = [RMSVector, RMS];
    RMSVerschil = abs(Verschil-RMS)';
    RMSVerschilLengte = [RMSVerschilLengte, numel(RMSVerschil)];
    RMSVerschilVector = [RMSVerschilVector; RMSVerschil];
    Identification = [repmat(j,RMSVerschilLengte(j),1)];
    IdentificationVector = [IdentificationVector; Identification];
    theXLabel = [theXLabel ' RMS ' num2str(j) ':' num2str(RMS) ]; 
    if plotDifference
        figure('visible','on','WindowStyle','docked',...
                'Name',[theTitle ' - DATA PLOT' num2str(j)]);
       subplot(3,1,1);
       plot(Aj);
       title([theTitle ' A PLOT']);
       subplot(3,1,2);
       plot(Bj);
       title([theTitle ' B PLOT']);
       subplot(3,1,3);
       plot(Verschil);
       title([theTitle ' Error PLOT']);
       
    end
end
figure('visible','on','WindowStyle','docked',...
                'Name',theTitle);
boxplot(RMSVerschilVector, IdentificationVector, 'notch', 'on')
xlabel (theXLabel),
ylabel('absoluut RMS verschil: '),
title (theTitle);
