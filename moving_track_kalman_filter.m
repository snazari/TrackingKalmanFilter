%% Sam Nazari
%  Kalman Filtering Demo of tracking a moving object using a 2-D kalman 
%  filter.

clear all;
clc;
base_dir = '.';
cd(base_dir);

%% Load Frames
img_file = dir('*png'); 
load('MT_node_moving.mat'); % we load the target node

%% Professing definitions
t       = 1;        % loop processing interval
frame   = 10;       % starting frame
u       = .005;     % control input
ksi     = [CM_idx(frame,1); CM_idx(frame,2); 0; 0]; % ICs
ksi_eta = ksi;      % state estimate
noise   = .1;       % process noise intensity
noise_x = 1;
noise_y = 1;

%% Kalman Filter params
R = [noise_x 0; 0 noise_y]; %coviarance of the noise
Q = [t^4/4 0 t^3/2 0; 0 t^4/4 0 t^3/2; t^3/2 0 t^2 0; 0 t^3/2 0 t^2].*noise^2; % covariance of the observation noise
P = Q; % estimate of initial state
F = [1 0 t 0; 0 1 0 t; 0 0 1 0; 0 0 0 1];  %state transition model
B = [1 0 0 0; 0 1 0 0];  %observation model
C = [(t^2/2); (t^2/2); t; t];  %control-input model
ksi_sta = []; % green node state
v = []; % green node velocity
y = []; % the measurements of the node state
ksi_sta_eta = []; %  initial state estimate
v_eta = []; % initial velocity estimate
P_eta = P;
pre_state = [];
pre_var = [];

%do the kalman filter and plot the origianl node and estimation node
for s = frame:length(img_file)
    img_tmp = double(imread(img_file(s).name));
    img = img_tmp(:,:,1);  % load the image
    y(:,s) = [ CM_idx(s,1); CM_idx(s,2)]; % load the given moving node
    
    ksi_eta = F * ksi_eta + C * u;
    pre_state = [pre_state; ksi_eta(1)] ;
    P = F * P * F' + Q; %Time Update
    pre_var = [pre_var; P] ;
    K = P*B'*inv(B*P*B'+R); %the Kalman Gain
    % Measurement Update
    if ~isnan(y(:,s))
        ksi_eta = ksi_eta + K * (y(:,s) - B * ksi_eta); %using the innovations signal
    end
   
    P =  (eye(4)-K*B)*P; % updated estimate covariance
    
    ksi_sta_eta = [ksi_sta_eta; ksi_eta(1:2)];
    v_eta = [v_eta; ksi_eta(3:4)];
    
    x_estimation(s)=ksi_eta(2); %estimation in horizontal position
    y_estimation(s)=ksi_eta(1); %estimation in vertical position
   
    r = 10;
    j=0:.01:2*pi; %parameters of nodes
    imagesc(img);
    axis off
    hold on;
    plot(r*sin(j)+y(2,s),r*cos(j)+y(1,s),'.b'); % the actual moving mode
    plot(r*sin(j)+ksi_eta(2),r*cos(j)+ksi_eta(1),'.r'); % the kalman filtered tracking node
    hold off
    pause(0.05) %speed of loading frame
end
%show the position difference in horizontal direction between actual and estimation positions
l1=length(y(2,:)); n=1:l1;
figure;
plot(n,y(2,:),'b',n,x_estimation,'r');
xlabel('time'); ylabel('horizontal position');
title('position difference in horizontal direction: estimation(red) and exact(blue)');

%show the position difference in vertical direction between actual and estimation positions
figure;
plot(n,y(1,:),'b',n,y_estimation,'r');
xlabel('time'); ylabel('vertical position');
title('position difference in vertical direction: estimation(red) and exact(blue)');

%show the distance between actual and estimation positions or error
x_d=y(2,:)-x_estimation;
y_d=y(1,:)-y_estimation;
xy_d=x_d.^(2)+y_d.^(2);
xy_d=xy_d.^(1/2);
figure;
plot(n,xy_d);
xlabel('time'); ylabel('position distance');
title('distance between actual and estimation positions');

%show tracking with low noise
n_very_noisy=150;
img_tmp = double(imread(img_file(n_very_noisy).name));
img = img_tmp(:,:,1);  % load the image
figure;
imagesc(img);
axis off
hold on;
plot(r*sin(j)+y(2,n_very_noisy),r*cos(j)+y(1,n_very_noisy),'.b'); % the actual moving mode
plot(r*sin(j)+x_estimation(n_very_noisy),r*cos(j)+y_estimation(n_very_noisy),'.r'); % the kalman filtered tracking node
title('tracking with low noise: estimation(red) and actual(blue)');
hold off

%show the mild noise tracking
%n_very_noisy=84;
%img_tmp = double(imread(img_file(n_very_noisy).name));
%img = img_tmp(:,:,1);  % load the image
%figure;
%imagesc(img);
%axis off
%hold on;
%plot(r*sin(j)+y(2,n_very_noisy),r*cos(j)+y(1,n_very_noisy),'.b'); % the actual moving mode
%plot(r*sin(j)+x_estimation(n_very_noisy),r*cos(j)+y_estimation(n_very_noisy),'.r'); % the kalman filtered tracking node
%title('tracking with mild noise: estimation(red) and actual(blue)');
%hold off

%show the very noise tracking
n_very_noisy=123;
img_tmp = double(imread(img_file(n_very_noisy).name));
img = img_tmp(:,:,1);  % load the image
figure;
imagesc(img);
axis off
hold on;
plot(r*sin(j)+y(2,n_very_noisy),r*cos(j)+y(1,n_very_noisy),'.b'); % the actual moving mode
plot(r*sin(j)+x_estimation(n_very_noisy),r*cos(j)+y_estimation(n_very_noisy),'.r'); % the kalman filtered tracking node
title('tracking with large noise: estimation(red) and actual(blue)');
hold off


