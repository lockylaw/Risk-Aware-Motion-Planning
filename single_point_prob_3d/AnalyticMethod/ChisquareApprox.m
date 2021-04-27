clear all
close all
clc 



%% Monte Carlo: compute probability with different sampling numbers
%  Working Environment: 1 robot, 1/2 static osbtacles with/without
%  uncertainty

% robot informations
model.robot_pos = [-1; 1];                      % robot position mean
model.robot_sigma_x = 0.2;                      % robot variance of x
model.robot_sigma_y = 0.3;                      % robot variance of y
model.robot_sigma_theta = 0.01;                 % robot variance of theta
model.robot_rho_xtheta= 0.0;                    % robot covariance of x and theta
model.robot_rho_ytheta= 0.0;                    % robot covariance of y and theta
model.robot_rho_xy= 0.0;                        % robot covariance of x and y

% covariance matrix/Sigma
model.robot_pos_cov = [model.robot_sigma_x^2, model.robot_rho_xy*model.robot_sigma_x*model.robot_sigma_y; 
                 model.robot_rho_xy*model.robot_sigma_x*model.robot_sigma_y, model.robot_sigma_y^2;];

% robot size
model.robot_size = [1; 1];
model.robot_ell_size = 1;     

% obstacle number
model.obs_pos = [0; 0];       % obstacle position mean
model.obs_num = size(model.obs_pos, 2);

model.obs_sigma_x = [0.2];             % obstacle variance of x                     
model.obs_sigma_y = [0.3];             % obstacle variance of y
model.obs_rho_xy= [0.0];               % obstacle covariance of x and y

% covariance matrix/Sigma
model.obs_pos_cov = zeros(2, 2, model.obs_num);
for i = 1:model.obs_num
    model.obs_pos_cov(:, :, i) = [model.obs_sigma_x(i)^2, model.obs_rho_xy(i)*model.obs_sigma_x(i)*model.obs_sigma_y(i); 
                            model.obs_rho_xy(i)*model.obs_sigma_x(i)*model.obs_sigma_y(i), model.obs_sigma_y(i)^2];
end

% osbtacle size
model.obs_box_size = [[2; 2],[3; 1.2], [3; 1.2]];    


% Probability Type
% -- 1 -- Gaussian 
% -- 2 -- Gaussian Mixture Model
% -- 3 -- Truncated Gaussian
ProbType = 1;

%% plot figure of distribution of robot and obstacles

fig.fig_main = figure;              % main figure
hold on;
axis([-4 1 -1 4]);
ax_main = fig.fig_main.CurrentAxes;
fig.ax_main = ax_main;
box on;
grid on;
axis equal;
xlabel('x (m)');
ylabel('y (m)');
set(ax_main, 'FontSize', 24);

for i = 1:model.obs_num
    fig.fig_obs_pos = plot(ax_main, model.obs_pos(1,i), model.obs_pos(2,i), ...
                'Color', 'g', ...
                'Marker', '*', ... 
                'LineWidth', 2.0, 'LineStyle', '-');
end


% plot robot position mean
fig.fig_robot_pos = plot(ax_main, model.robot_pos(1), model.robot_pos(2), ...
                'Color', 'g', ...
                'Marker', '*', ... 
                'LineWidth', 2.0, 'LineStyle', '-');

% plot robot shape
fig.fig_ell_robot = plot_ellipse_2D(ax_main, model.robot_pos, model.robot_size, 0, ...
                'FaceColor', 'r', 'FaceAlpha', 0.4, ...
                'EdgeColor', 'r', 'EdgeAlpha', 0.6);

%%
N = 1e6;
pd = mvnrnd([model.robot_pos(1), model.robot_pos(2)], model.robot_pos_cov, N);
pdf = mvnpdf(pd, [model.robot_pos(1), model.robot_pos(2)], model.robot_pos_cov);

% pd_1 = mvnrnd([model.obs_pos(1, 1), model.obs_pos(2, 1)], ...
%         model.obs_pos_cov(:, :, 1), N);
% pd_obs = pd_1;

% generate random samples of robots
rand_x = pd(:, 1);
rand_y = pd(:, 2);

hold on
scatter(fig.ax_main, rand_x, rand_y, '.');

dis_size = 0.5;

if_Collision = zeros(N, 1);
for i = 1:N
    
    dis = sqrt((pd(i, 1) - model.robot_pos(1))^2 + (pd(i, 2) - model.robot_pos(2))^2);
    if dis < dis_size
        if_Collision(i) = 1;
    end
   
end

ColliProb_MC = sum(if_Collision) / N;
ColliVar = sum( (if_Collision - pdf ) .^ 2 ) / N ^ 2;


%%









