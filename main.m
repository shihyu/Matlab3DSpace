tic
clear;
close all;
clc;
format long;
filename = '~/aa_sofie_data/bicycle-instrumentation/imu-round-2/imu-round2.h5'
node=1;
%run = '/zaxisnode1_2'
%run = '/zaxisnode1'
run = '/xaxisnode1_2'
%run = '/xaxisnode2'
%run = '/yaxisnode1'         %Does not synchronise.
%run = '/yaxisnode1_2'
%run = '/yaxisnode2'
%run = '/moveaxisnode1'
plotGraphs = 1;
makeMovie = 0;

%SampleRates
Fs = 120
Fs_movie = 120/15;
Fs_quat = 200
%Get the data..
[quaternions, rbt,lbt,fot] = getdata(filename,run,node);
quaternions = resample(quaternions(:,1:5),Fs,Fs_quat);

%Create Objects and sync metrics.
[metric_v,metric_i,H_0_V_t,H_1_I_t] = createObjects(rbt,lbt,fot,quaternions);
%Synchronise
[H_0_V_t,H_1_I_t] = ...
    synchronise(metric_v,metric_i,H_0_V_t,H_1_I_t,Fs,600,100);
clear metric_i;
clear metric_v;

H_I_V_est = calculateChangeofGlobalFrames(H_0_V_t,H_1_I_t,200,99)

%Find new end.
minSize = min(size(H_0_V_t,2),size(H_1_I_t,2));
v_eulers1 = repmat(zeros(3,1),1,size(H_0_V_t,2)-200);
v_eulers2 = repmat(zeros(3,1),1,size(H_0_V_t,2)-200);
i_eulers1 = repmat(zeros(3,1),1,size(H_0_V_t,2)-200);
i_eulers2 = repmat(zeros(3,1),1,size(H_0_V_t,2)-200);
error = repmat(zeros(3,1),1,size(H_0_V_t,2)-200);
figure;
if makeMovie == 1
    hf= figure('visible','off','OuterPosition',[0 0 2048 2048]); %turns visibility of figure off
    aviobj=avifile('test.avi'); %creates AVI file, test.avi
end

display('Calculating positions.')

for i = 200:minSize
    qvt = H_1_I_t(i);
    H_1_V = H_I_V_est*qvt.getH;
    cvt = H_0_V_t(i);
    H_0_V = cvt.getH;
    %Convert to Inertia frame.
    points_I =H_I_V_est*qvt.getT;
    
    %Calculate the difference between the rotation from one to the other.
    [v_eulers1(:,i-200+1),v_eulers2(:,i-200+1)] = invrpy(H_0_V);
    [i_eulers1(:,i-200+1),i_eulers2(:,i-200+1)] = invrpy(H_1_V);
    %Error
    [errorQuat,error(:,i-200+1)] =  cvt.quaternionerror(cvt.getQ,...
            matrix2quaternion(H_1_V));
    
    if plotGraphs == 1
        subplot(1,2,1);
        cvt.plot_T();
        title('Vicon Points in Vicon Frame:');
        grid on
        axis([-2 2 -2 2 -2 2]);
        
        subplot(1,2,2);
        %plotSensor(cvt.points_psi_0,'r');
        plotSensor(points_I,'r');
        title('Promove Points in Vikon frame');
        grid on
        axis([-2 2 -2 2 -2 2]);
        
        if makeMovie &&  mod(i,Fs_movie) == 0
            aviobj=addframe(aviobj,hf); %adds frames to the AVI file
        else
            drawnow;
        end
    end
end
axes('Position',[0 0 1 1])
display('Creating Video');
if makeMovie == 1
    aviobj=close(aviobj); %closes the AVx = 2*pi;
y = 2*pi+0.1;
assertElementsAlmostEqual(angleDifference(x,y),-0.1);
assertElementsAlmostEqual(angleDifference(y,x),0.1);I file
    close(hf); %closes the handle to invisible figure
end;


display('Plotting error.');
t=linspace(0,1/Fs*size(v_eulers1,2),size(v_eulers1,2));
%Plots of angles
figure
subplot(3,2,1);
plotangles(t,v_eulers1(1,:),i_eulers1(1,:),'Z-Axis (radians):')
subplot(3,2,3);
plotangles(t,v_eulers1(2,:),i_eulers1(2,:),'Y-axis (radians):')
subplot(3,2,5)
plotangles(t,v_eulers1(3,:),i_eulers1(3,:),'X-axis (radians):')

%Plots of Error
subplot(3,2,2);
x=v_eulers1(1,:);
y=i_eulers1(1,:);
diffAng = ThreeMarkers.angleDifference(x,y);
plotangles(t,diffAng,0,...
    ['Z-axis Error (radians): max:' num2str(max(abs(diffAng))) ]);
subplot(3,2,4);
x=v_eulers1(2,:);
y=i_eulers1(2,:);
diffAng = ThreeMarkers.angleDifference(x,y);
plotangles(t,diffAng,0,...
    ['Y-axis Error (radians): max:' num2str(max(abs(diffAng))) ]);
subplot(3,2,6);
x=v_eulers1(3,:);
y=i_eulers1(3,:);
diffAng = ThreeMarkers.angleDifference(x,y);
plotangles(t,diffAng,0,...
    ['X-axis Error (radians): max:' num2str(max(abs(diffAng))) ]);

figure
subplot(3,2,1);
plotangles(t,v_eulers1(1,:),i_eulers1(1,:),'Z-Axis (radians):')
subplot(3,2,3);
plotangles(t,v_eulers1(2,:),i_eulers1(2,:),'Y-axis (radians):')
subplot(3,2,5)
plotangles(t,v_eulers1(3,:),i_eulers1(3,:),'X-axis (radians):')

%Plots of Error
subplot(3,2,2);
plotangles(t,error(1,:),0,...
    ['Z-axis Error (radians):']);
subplot(3,2,4);
plotangles(t,error(2,:),0,...
    ['Y-axis Error (radians):']);
subplot(3,2,6);
plotangles(t,error(3,:),0,...
    ['X-axis Error (radians):']);
toc
