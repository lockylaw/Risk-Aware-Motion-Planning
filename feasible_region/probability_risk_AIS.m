% An example to show the risk of the robot
clear all
close all
clc 

rng('default')  % For reproducibility

%% configurations
% setup

model.robot_pos = [-2; 1];                      % robot position mean
model.robot_sigma_x = 0.4;                      % robot variance of x
model.robot_sigma_y = 0.4;                      % robot variance of y
model.robot_sigma_theta = 0.01;                 % robot variance of theta
model.robot_rho_xtheta= 0.0;                    % robot covariance of x and theta
model.robot_rho_ytheta= 0.0;                    % robot covariance of y and theta
model.robot_rho_xy= -0.0;    

% covariance matrix/Sigma
model.robot_pos_cov = [model.robot_sigma_x^2, model.robot_rho_xy*model.robot_sigma_x*model.robot_sigma_y; 
                 model.robot_rho_xy*model.robot_sigma_x*model.robot_sigma_y, model.robot_sigma_y^2;];

% robot size
model.robot_size = [0.5; 0.5];
model.robot_ell_size = model.robot_size * 0.5 * sqrt(2);
model.obs_num = 2;
model.obs_shape = "ellip";
if model.obs_num == 1
    model.obs_size = [2; 1];
    model.obs_pos = [0; 0];
    model.max_size = [-4 4 -3 3];
elseif model.obs_num == 2
    model.obs_size = [[2; 1], [1; 2]];
    model.obs_pos = [-2 1; -1 1];
    model.max_size = [-5.5 3.5 -3.5 4.5];
end

model.obs_ell_size = model.obs_size * 0.5 * sqrt(2);

% whether the uncertainties of obstacles are considered
model.if_obsEllip = 1;
model.if_Gaussian = 1;


%% plot figure
fig_main = figure;              % main figure
hold on;
axis(model.max_size);
ax_main = fig_main.CurrentAxes;
box on;
grid on;
axis equal;
xlabel('x (m)');
ylabel('y (m)');
set(ax_main, 'FontSize', 24);
% plot obstacle
% fig_box_obs = plot_box_2D(ax_main, obs_pos, obs_box_size, 0, ...
%             'FaceColor', [0.4 0.4 0.4], 'FaceAlpha', 0.6, ...
%             'EdgeColor', 'k', 'EdgeAlpha', 0.8);
for i = 1:model.obs_num
    fig_ell_obs = plot_ellipse_2D(ax_main, model.obs_pos(:, i), model.obs_ell_size(:, i), 0, ...
                'FaceColor', 'r', 'FaceAlpha', 0.4, ...
                'EdgeColor', 'r', 'EdgeAlpha', 0.6);
end
fig_ell_robot = plot_ellipse_2D(ax_main, model.robot_pos, model.robot_ell_size, 0, ...
            'FaceColor', 'r', 'FaceAlpha', 0.4, ...
            'EdgeColor', 'r', 'EdgeAlpha', 0.6);
% plot robot position mean
fig_robot_pos = plot(ax_main, model.robot_pos(1), model.robot_pos(2), ...
                'Color', 'g', ...
                'Marker', '*', ... 
                'LineWidth', 2.0, 'LineStyle', '-');
% plot robot position covariance
[pos_cov_xData, pos_cov_yData] = ...
    getErrorEllipsePoint2D(model.robot_pos_cov, model.robot_pos, 1, 0);
fig_robot_pos_cov_1 = plot(ax_main, pos_cov_xData, pos_cov_yData, ...
                'Color', 'k', ...
                'LineWidth', 1.5, 'LineStyle', '--');
[pos_cov_xData, pos_cov_yData] = ...
    getErrorEllipsePoint2D(model.robot_pos_cov, model.robot_pos, 2, 0);
fig_robot_pos_cov_2 = plot(ax_main, pos_cov_xData, pos_cov_yData, ...
                'Color', 'b', ...
                'LineWidth', 1.5, 'LineStyle', '--');
[pos_cov_xData, pos_cov_yData] = ...
    getErrorEllipsePoint2D(model.robot_pos_cov, model.robot_pos, 3, 0);
fig_robot_pos_cov_3 = plot(ax_main, pos_cov_xData, pos_cov_yData, ...
                'Color', 'm', ...
                'LineWidth', 1.5, 'LineStyle', '--');
print(fig_main, 'scenario.pdf', '-dpdf', ...
            '-r300', '-bestfit');

%% risk contour map
pos_x_vec = model.max_size(1):0.1:model.max_size(2);
pos_y_vec = model.max_size(3):0.1:model.max_size(4);
OptionsAIS.sampleN = 1e3;
OptionsAIS.iteration_l = 10;
[X, Y] = meshgrid(pos_x_vec, pos_y_vec);
collision_probability = zeros(length(pos_x_vec), length(pos_y_vec));
VaR = zeros(length(pos_x_vec), length(pos_y_vec),3);
% MC sampling
for i = 1 : length(pos_x_vec)
    for j = 1 : length(pos_y_vec)
        pos = [pos_x_vec(i); pos_y_vec(j)];
        model.robot_pos = pos;
        % collision probability
        collision_probability(i,j) = AIS(model, OptionsAIS);
        fprintf('\n------------ sample (%d,%d) ------------', i, j);
        fprintf('\n collision probability = %.2f\n', collision_probability(i,j));
    end
end

%%
% plotting collision probability contour
figure;
hold on;
grid on;
axis(model.max_size);
daspect([1 1 0.2])
xlabel('x (m)');
ylabel('y (m)');
set(gca, 'FontSize', 24);
for i = 1:model.obs_num
    fig_ell_obs = plot_ellipse_2D(gca, model.obs_pos(:, i), model.obs_ell_size(:, i), 0, ...
                'FaceColor', 'r', 'FaceAlpha', 0.4, ...
                'EdgeColor', 'r', 'EdgeAlpha', 0.6);
end
% surf(X, Y, collision_probability', 'EdgeColor', 'none', ...
%     'FaceAlpha', 0.6);
contour(X, Y, collision_probability', [0.01, 0.01], 'LineColor', 'r', ...
    'LineWidth', 2.0);
contour(X, Y, collision_probability', [0.03, 0.03], 'LineColor', 'm', ...
    'LineWidth', 2.0);
contour(X, Y, collision_probability', [0.10, 0.10], 'LineColor', 'k', ...
    'LineWidth', 2.0);
