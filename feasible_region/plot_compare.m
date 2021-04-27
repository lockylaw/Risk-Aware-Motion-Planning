% An example to show the risk of the robot
clear all
close all
clc 

rng('default')  % For reproducibility

% model_type define the distribution type
% ---  1  Gaussian 
% ---  2  GMM
model_type = 2;

if model_type == 1
    load('colliProb_MC_1obs.mat');
    collision_probability_MC = collision_probability;
    load('model_1obs.mat');
    ProbThre = 0.03;
    [feasible_region_linear, X_linear, Y_linear] = ...
        probability_risk_Linear(ProbThre, model_type);
    [feasible_region_fastAss, X_fastAss, Y_fastAss] = ...
        probability_risk_FastAss(ProbThre, model_type);

elseif model_type == 2
    load('colliProb_MC_1obs_nonG.mat');
    collision_probability_MC = collision_probability;
    load('model_1obs_nonG.mat');
    ProbThre = 0.03;
    [feasible_region_fastAss, X_fastAss, Y_fastAss] = ...
        probability_risk_FastAss(ProbThre, model_type);
end

%%
% plotting collision probability contour
pos_x_vec = model.max_size(1):0.1:model.max_size(2);  
pos_y_vec = model.max_size(3):0.1:model.max_size(4);
[X, Y] = meshgrid(pos_x_vec, pos_y_vec);

figure;
hold on;
grid on;
view(2);

daspect([1 1 0.2])
xlabel('x (m)');
ylabel('y (m)');
% zlabel('Pr');
set(gca, 'FontSize', 24);

for i = 1:model.obs_num
    fig_ell_obs = plot_ellipse_2D(gca, model.obs_pos(:, i), ...
                model.obs_size(:, i), model.orient, ...
                'FaceColor', 'r', 'FaceAlpha', 0.4, ...
                'EdgeColor', 'r', 'EdgeAlpha', 0.6, 'HandleVisibility', 'off');
end

if model_type == 1
    [~, region_MC] = contour(X, Y, collision_probability_MC', [0.03, 0.03], 'LineColor', 'r', ...
    'LineWidth', 2.0);
    [~, linear_shape] = contour(X_linear, Y_linear, feasible_region_linear', [0.1, 0.1], 'LineColor', 'g', ...
        'LineWidth', 2.0);
    [~, FastAss_shape] = contour(X_fastAss, Y_fastAss, feasible_region_fastAss', [0.1, 0.1], 'LineColor', 'k', ...
        'LineWidth', 2.0);
    sigma3_shape = plot_ellipse_curve(gca, model.obs_pos(:, 1), model.obs_size(:, 1) + ...
                    model.robot_size(:, 1) + 3 * [model.robot_sigma_x; model.robot_sigma_y], ...
                    model.orient, 'm', 'LineWidth', 2.0);
    legend([region_MC, linear_shape, FastAss_shape, sigma3_shape], ...
        'MC', 'linear', 'Fast', '3\sigma')
    title('Feasible Region, Gaussian')
    axis([-3 3 -3 3]);
elseif model_type == 2
    [~, region_MC] = contour(X, Y, collision_probability_MC', [0.03, 0.03], 'LineColor', 'r', ...
    'LineWidth', 2.0);
    [~, FastAss_shape] = contour(X_fastAss, Y_fastAss, feasible_region_fastAss', [0.1, 0.1], 'LineColor', 'k', ...
        'LineWidth', 2.0);
    sigma3_shape_MC = plot_ellipse_curve(gca, model.obs_pos(:, 1), model.obs_size(:, 1) + ...
                    model.robot_size(:, 1) + 3 * [model.robot_sigma_x; model.robot_sigma_y], ...
                    model.orient, 'm', 'LineWidth', 2.0);
    max_sigma_x = max(model.sigma_x_1, model.sigma_x_2);
    max_sigma_y = max(model.sigma_y_1, model.sigma_y_2);
    sigma3_shape_max = plot_ellipse_curve(gca, model.obs_pos(:, 1), model.obs_size(:, 1) + ...
                    model.robot_size(:, 1) + 3 * [max_sigma_x; max_sigma_y], ...
                    model.orient, 'b', 'LineWidth', 2.0);
    legend([region_MC, FastAss_shape, sigma3_shape_MC, sigma3_shape_max], ...
        'MC', 'Fast', '3\sigma_{MC}', '3\sigma_{max}')
    title('Feasible Region, GMM')
    axis([-3 3 -3 3]);
end
    



function h = plot_ellipse_curve(ax, pos, ell, orient, varargin)
    a = ell(1);
    b = ell(2);
    r = 0: 0.1: 2*pi+0.1;
    alpha = [ cos(orient) -sin(orient)
              sin(orient)  cos(orient)];
    p = [(a*cos(r))' (b*sin(r))'] * alpha;
    X = pos(1) + p(:,1);
    Y = pos(2) + p(:,2);

    % plot the ellipse
    h = plot(ax, X, Y, varargin{:});
    
end