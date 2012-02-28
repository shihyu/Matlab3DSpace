tic
clear;
close all;
clc;
format long;
filename = '~/aa_sofie_data/bicycle-instrumentation/imu-round-2/imu-round2.h5'
node=1;
%run = '/zaxisnode1_2'
run = '/zaxisnode1'
%run = '/xaxisnode1_2'
%run = '/xaxisnode2'
%run = '/yaxisnode1'         %Does not synchronise.
%run = '/yaxisnode1_2'
%run = '/yaxisnode2'
%run = '/moveaxisnode1'
plotGraphs = 0;
makeMovie = 0;

%Get the data..
[quaternions, rbt,lbt,fot] = getdata(filename,run,node);

%quat = quaternions(2,:);
Fs = 120
%Double speed movie.
Fs_movie = 120/15;
Fs_quat = 200
quaternions = resample(quaternions(:,1:4),Fs,Fs_quat);

hdiffOld = zeros(4,8);

minSize = min(size(rbt,1),size(quaternions,1));
%minSize =2500;

y_v = zeros(1,minSize);
y_i = zeros(1,minSize);

H_psi_0_psi_v_t = [];
H_psi_1_psi_i_t = [];

points_v = [];
points_i = [];
%Loop to create H matrixs and calculate diff's in rotation matrix's
for i = 1:minSize
    %Get the vicon points.
    cvt = ConvertViconThreeMarkers(rbt(i,1:3),...
        lbt(i,1:3),fot(i,1:3),rbt(i,4));
    qvt = QuaternionsThreeMarkers(quaternions(i,1:4),rbt(i,4));
    %Save the cvt
    H_psi_0_psi_v_t = [H_psi_0_psi_v_t cvt];
    %Save the quaternions.
    H_psi_1_psi_i_t = [H_psi_1_psi_i_t qvt];
    
    %Calculate the difference between H matrix's from last point
    y_v(i) = calculateRotDiff(cvt.H_psi_v_psi_0,hdiffOld(1:4,1:4));
    y_i(i) = calculateRotDiff(qvt.H_psi_1_psi_i,hdiffOld(1:4,5:8));
    %Save current H matrix for next iteration.
    hdiffOld = [cvt.H_psi_v_psi_0 qvt.H_psi_1_psi_i];
end
clear fot;
clear rbt;
clear lbt;

[H_psi_0_psi_v_t,H_psi_1_psi_i_t] = ...
    synchronise(y_v,y_i,H_psi_0_psi_v_t,H_psi_1_psi_i_t,Fs,200);

grid on;
clear Riv;
clear RvI;
%Find new end.
minSize = min(size(H_psi_0_psi_v_t,2),size(H_psi_1_psi_i_t,2));

%Get the estimated H Matrix: Least Means square solution
%from within the Still part of the experiment.
points_v = [];
points_i = [];
for i = 200:200+99
    cvt = H_psi_0_psi_v_t(i);
    qvt = H_psi_1_psi_i_t(i);
    points_v = [points_v cvt.points_psi_v];
    points_i = [points_i qvt.points_psi_i];
end
%Estimate the transform from 1 to 0.
H_psi_1_psi_0_est = points_v*pinv(points_i);
%Zero translation and error.
H_psi_1_psi_0_est(1:3,4) = 0;
H_psi_1_psi_0_est(4,1:3) = 0;

%
v_eulers1 = repmat(zeros(3,1),1,size(H_psi_0_psi_v_t,2)-200);
v_eulers2 = repmat(zeros(3,1),1,size(H_psi_0_psi_v_t,2)-200);
i_eulers1 = repmat(zeros(3,1),1,size(H_psi_0_psi_v_t,2)-200);
i_eulers2 = repmat(zeros(3,1),1,size(H_psi_0_psi_v_t,2)-200);
error = repmat(zeros(3,1),1,size(H_psi_0_psi_v_t,2)-200);
figure;
if makeMovie == 1
    hf= figure('visible','off','OuterPosition',[0 0 2048 2048]); %turns visibility of figure off
    aviobj=avifile('test.avi'); %creates AVI file, test.avi
end

display(['Calculating positions.'])

for i = 200:minSize
    H_psi_1_psi_i = H_psi_1_psi_i_t(i).H_psi_1_psi_i;
    %Convert to Inertia frame.
    H_psi_0_psi_i = H_psi_1_psi_0_est*H_psi_1_psi_i;
    cvt = H_psi_0_psi_v_t(i);
    H_psi_0_psi_v = cvt.H_psi_v_psi_0';
    points_psi_i = H_psi_0_psi_i*qvt.points_psi_1;
    
    %Calculate the difference between the rotation from one to the other.
    [v_eulers1(:,i-200+1),v_eulers2(:,i-200+1)] = invrpy(H_psi_0_psi_v);
    [i_eulers1(:,i-200+1),i_eulers2(:,i-200+1)] = invrpy(H_psi_0_psi_i);
    %Error
    [errorQuat,error(:,i-200+1)] =  quaternionerror(cvt.quaternion,...
            matrix2quaternion(H_psi_0_psi_i)');
    
    if plotGraphs == 1
        subplot(1,2,1);
        cvt.plot_v();
        title('Vicon Points in Vicon Frame:');
        grid on
        axis([-2 2 -2 2 -2 2]);
        
        subplot(1,2,2);
        %plotSensor(cvt.points_psi_0,'r');
        plotSensor(points_psi_i,'r');
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
diffAng = angleDifference(x,y);
plotangles(t,diffAng,0,...
    ['Z-axis Error (radians): max:' num2str(max(abs(diffAng))) ]);
subplot(3,2,4);
x=v_eulers1(2,:);
y=i_eulers1(2,:);
diffAng = angleDifference(x,y);
plotangles(t,diffAng,0,...
    ['Y-axis Error (radians): max:' num2str(max(abs(diffAng))) ]);
subplot(3,2,6);
x=v_eulers1(3,:);
y=i_eulers1(3,:);
diffAng = angleDifference(x,y);
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
