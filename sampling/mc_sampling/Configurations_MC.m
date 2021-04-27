clear all
close all
clc 



%% Monte Carlo: compute probability with different sampling numbers
%  Working Environment: 1 robot, 1/2 static osbtacles with/without
%  uncertainty

% robot informations
model.robot_pos = [-1; 1];                      % robot position mean
model.robot_sigma_x = 0.4;                      % robot variance of x
model.robot_sigma_y = 0.6;                      % robot variance of y
model.robot_sigma_theta = 0.01;                 % robot variance of theta
model.robot_rho_xtheta= 0.0;                    % robot covariance of x and theta
model.robot_rho_ytheta= 0.0;                    % robot covariance of y and theta
model.robot_rho_xy= 0.0;                        % robot covariance of x and y

% covariance matrix/Sigma
model.robot_pos_cov = [model.robot_sigma_x^2, model.robot_rho_xy*model.robot_sigma_x*model.robot_sigma_y; 
                 model.robot_rho_xy*model.robot_sigma_x*model.robot_sigma_y, model.robot_sigma_y^2;];

% robot size
model.robot_box_size = [1; 1];
model.robot_ell_size = model.robot_box_size/2;%0.5*sqrt(2)*model.robot_box_size;

% obstacle number
% -- 1 -- one single obstacle
% -- 2 -- two osbatcles
model.obs_num = 1;

if model.obs_num == 1
    model.obs_pos = [0; 0];  
    model.obs_sigma_x = [0.4];
    model.obs_sigma_y = [0.4];
    model.obs_sigma_theta = [0.01];
    model.obs_rho_xtheta= [0.0];
    model.obs_rho_ytheta= [0.0];
    model.obs_rho_xy= [0.0];
else
    model.obs_pos = [[0; 0],[-2; 0]];                   % obstacle position mean
    model.obs_sigma_x = [0.4, 0.3];                     % obstacle variance of x                     
    model.obs_sigma_y = [0.4, 0.3];                     % obstacle variance of y
    model.obs_sigma_theta = [0.01, 0.01];               % obstacle variance of theta
    model.obs_rho_xtheta= [0.0, 0.0];                   % obstacle covariance of x and theta
    model.obs_rho_ytheta= [0.0, 0.0];                   % obstacle covariance of y and theta
    model.obs_rho_xy= [0.0, 0.0];                       % obstacle covariance of x and y
end

% covariance matrix/Sigma
model.obs_pos_cov = zeros(2, 2, model.obs_num);
for i = 1:model.obs_num
    model.obs_pos_cov(:, :, i) = [model.obs_sigma_x(i)^2, model.obs_rho_xy(i)*model.obs_sigma_x(i)*model.obs_sigma_y(i); 
                            model.obs_rho_xy(i)*model.obs_sigma_x(i)*model.obs_sigma_y(i), model.obs_sigma_y(i)^2];
end

% osbtacle size
model.obs_box_size = [1; 1];
model.obs_ell_size = model.obs_box_size/2;%0.5*sqrt(2)*model.obs_box_size;

% whether the uncertainties of obstacles are considered
model.if_obsEllip = 1;

%% plot figure of distribution of robot and obstacles

% whether the uncertainties of obstalces are considered 
plotoptions.if_obsEllip = 1;
% whether the uncertainty of robot considered 
plotoptions.if_robotEllip = 1;

fig = plot_figure(model, plotoptions);


%% Monte-Carlo sampling result
sampleN = [[1:9]*1e3, [1:9]*1e4, [1:9]*1e5, [1:9]*1e6];
ColliProbability = zeros(size(sampleN, 2),1);
time = zeros(size(sampleN, 2),1);

for i = 1:size(sampleN, 2)
    % Monte-Carlo Sampling
    [ColliProbability(i,1), time(i,1)] = mcCollsionProbability(model, sampleN(i));
end

% mean of sampling result


% the figure of sampling result
figure(2);
plot(log10(sampleN), ColliProbability);
ylim([0 1])
xlim([log10(sampleN(1)) log10(sampleN(end))])
title('Collision Probability with Monte-Carlo Sampling')
xlabel('iterations')
ylabel('Probability')

% the figure of sampling time
figure(3);
fig_time = plot(log10(sampleN), time);
xlim([log10(sampleN(1)) log10(sampleN(end))])
title('Time with Monte-Carlo Sampling')
xlabel('iterations')
ylabel('Time(s)')
        


        
        
