function initialiseprocessing()
%initiliseprocessing Turn on MATLAB pool clear workspace and set format.
clear all;
close all;
clc;
format long;
poolSize = matlabpool('size');
if poolSize == 0
    matlabpool local 4;
end

end

