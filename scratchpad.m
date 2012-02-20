
% x = [5 0 0]
% 
% normX = norm(x)
% 
% x = x/norm(x)
% 
% 
% 
% x = [5 0 4]
% y = [0 5 4]
% mid = [0 0 4]
% 
% crossPoint = cross(x-mid,y-mid) + mid
% 
% crossPoint = (crossPoint-mid)/norm(crossPoint-mid) + mid
% 
% normXMid = norm(x-mid)
% x = (x-mid)/norm(x-mid)+mid
% normXmid = norm(x-mid)


%Cross correlation.
% x = [0 0 1 0 1 0 0 ];
% y = [0 0 0 1 0 1 0 0 0 0];
% Ny = size(y,2);
% Nx = size(x,2);
% R_xy = xcorr(x,y);
% R_yx = xcorr(y,x);
% 
% figure;
% subplot(2,2,1);
% R=R_xy(1:Ny);
% R_xy=fliplr(R);
% stem([1:size(R_xy,2)],R_xy,'k');
% 
% subplot(2,2,2);
% [max_xcf,max_lags]=max(R_xy)
% hold on;
% stem([1:Nx],x,'k')
% stem([1:Ny-max_lags+1],y(max_lags:Ny).*2,'g')
% title(['Rxy max corr:' num2str(max_xcf) ' LAG ' num2str(max_lags)])
% 
% subplot(2,2,3);
% R=R_yx(1:Ny);
% R_yx=fliplr(R);
% stem([1:size(R_yx,2)],R_yx,'g');
% 
% subplot(2,2,4);
% [max_xcf,max_lags]=max(R_yx)
% hold on;
% stem([1:Ny],y,'k')
% stem([1:Nx-max_lags+1],x(max_lags:Nx).*2,'g')
% title(['Ryx max corr:' num2str(max_xcf) ' LAG ' num2str(max_lags)])
% 
% y = [9;5;7]
% A = [3]
% ,[1:size(v_eulers1,1)]
% x = A\y
% 
% A = [2.9;3.2;2.8;3;3.15]
% A_est = pinv(A)
% x = A_est*y
size(v_eulers1,2)
t=linspace(0,1/Fs*size(v_eulers1,2),size(v_eulers1,2));
plot(t,v_eulers1(1,:),'k-')


