% An example to show the risk of the robot
clear all
close all
clc 

rng('default')  % For reproducibility

%% configurations
% setup

% model_type define the distribution type
% ---  1  Gaussian 
% ---  2  GMM
model_type = 2;

model = modelGenerator(model_type);


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
    fig_ell_obs = plot_ellipse_2D(ax_main, model.obs_pos(:, i), ...
                model.obs_size(:, i), model.orient(i), ...
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

%% MOnte Carlo
pos_x_vec_MC = model.max_size(1):0.1:model.max_size(2);
pos_y_vec_MC  = model.max_size(3):0.1:model.max_size(4);
[X_MC , Y_MC ] = meshgrid(pos_x_vec_MC, pos_y_vec_MC);
num_MC_sample = 1E4;
collision_probability = zeros(length(pos_x_vec_MC), length(pos_y_vec_MC));
% MC sampling
for i = 1 : length(pos_x_vec_MC)
    for j = 1 : length(pos_y_vec_MC)
        pos = [pos_x_vec_MC(i); pos_y_vec_MC(j)];
        model.robot_pos = pos;
%         collision probability
        collision_probability(i,j) = MonteCarlo(num_MC_sample, model);
        fprintf('\n------------ sample (%d,%d) ------------', i, j);
        fprintf('\n collision probability = %.2f\n', collision_probability(i,j));
    end
end

%%
% plotting collision probability contour
figure;
hold on;
grid on;
view(2);
axis(model.max_size);
daspect([1 1 0.2])
xlabel('x (m)');
ylabel('y (m)');
% zlabel('Pr');
set(gca, 'FontSize', 24);
for i = 1:model.obs_num
    fig_ell_obs = plot_ellipse_2D(gca, model.obs_pos(:, i), ...
                model.obs_ell_size(:, i), model.orient(i), ...
                'FaceColor', 'r', 'FaceAlpha', 0.4, ...
                'EdgeColor', 'r', 'EdgeAlpha', 0.6);
end
contour(X_MC, Y_MC, collision_probability', [0.03, 0.03], 'LineColor', 'r', ...
    'LineWidth', 2.0);