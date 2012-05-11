function test_suite = test_processingutils
% MUST BE IN THE DIRECTORY WHERE THE TEST RUNS.

function test_synchronise
clear all
close all
Fs = 120;
data1 = [0 1 0 0 0];
data2 = [0 0 0 1 0 0];
metric1 = [0 1 0 0 0];
metric2 = [0 0 0 1 0 0];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,1,4)
assertEqual(data1,result1);
assertEqual(result2,[0 1 0 0]);
assertEqual(data1(1:4),result2);
clear all
close all
Fs = 120;
data1 = [0 0 0 1 0 0];
data2 = [0 1 0 0 0];
metric1 = [0 0 0 1 0 0];
metric2 = [0 1 0 0 0];

[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,1,4)
assertEqual(data2,result2);
assertEqual(result1,[0 1 0 0]);
assertEqual(data2(1:4),result1);

Fs = 120;
data1 = [0 1 0 0 2 0 0 1];
data2 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
metric1 = [0 1 0 0 2 0 0 1];
metric2 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,1,4)
assertEqual(data1,result1);
assertEqual(result2,[0 1 0 0 4 0 0 2 0 1 1 3]);

Fs = 120;
data1 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
data2 = [0 1 0 0 2 0 0 1];
metric1 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
metric2 = [0 1 0 0 2 0 0 1];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,1,4)
assertEqual(data2,result2);
assertEqual(result1,[0 1 0 0 4 0 0 2 0 1 1 3]);

Fs = 1;
data1 = [0 1 0 0 2 0 0 1];
data2 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
metric1 = [0 1 0 0 2 0 0 1];
metric2 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,2,4)
assertEqual(data1,result1);
assertEqual(result2,[0 1 0 0 4 0 0 2 0 1 1 3]);

Fs = 1;
data1 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
data2 = [0 1 0 0 2 0 0 1];
metric1 = [0 1 2 0 0 1 0 0 1 0 0 4 0 0 2 0 1 1 3];
metric2 = [0 1 0 0 2 0 0 1];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,2,4)
assertEqual(data2,...
    result2);
assertEqual(result1,[0  1 0 0 4 0 0 2 0 1 1 3]);

Fs = 1;
data1 = [0 0 1 0 0 8 0 0 1 0 0 2 0 0 1 0 1 1 3];
data2 = [0 1 0 0 2 0 0 1];
metric1 = [0 0 1 0 0 8 0 0 1 0 0 2 0 0 1 0 1 1 3];
metric2 = [0 1 0 0 2 0 0 1];
[result1,result2] = synchronise(metric1,metric2,data1,data2,Fs,2,4)
assertEqual(data2,...
    result2);
assertEqual(result1,[0 1 0 0 8 0 0 1 0 0 2 0 0 1 0 1 1 3]);
%close all;
