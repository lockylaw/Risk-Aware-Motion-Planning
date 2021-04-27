clear all
close all
clc 
load("planning_path.mat")

%% probelm setup for the collision avoidance scenario
% the global pr struct should not be changed during simulation
global pr
% workspace environment
pr.ws_x         = [-10; 10];
pr.ws_y         = [-6; 6];
% robot physics
pr.robot_dynamics_continuous  = @double_integrator_dynamics_continuous;	% dynamics
pr.robot_maxVx  = 2.0;
pr.robot_maxVy  = 2.0;
pr.robot_maxAx  = 1.0;
pr.robot_maxAy  = 1.0;
% obstacle physic
model.obs_num         = 1;
model.obs_size        = [1.6; 1.0];         	% [a, b]
model.obs_scale       = 1.6;                  % collision potential scale
% robot start and goal postion
model.robot_size      = [0.4; 0.4];
model.robot_pos_start = [-8; 0];              % 2x1
model.robot_pos_goal  = [ 8; 0];              % 2x1

model.robot_sigma_x = 0.02;                      % robot variance of x
model.robot_sigma_y = 0.02;                      % robot variance of y
model.robot_sigma_vel_x = 0.01;                 % robot variance of v_x
model.robot_sigma_vel_y = 0.01;                 % robot variance of v_y
model.robot_sigma_a_x = 0.01;                 % robot variance of theta
model.robot_sigma_a_y = 0.01;                 % robot variance of theta

model.robot_pos_cov = [model.robot_sigma_x^2, 0, 0, 0; 
                       0, model.robot_sigma_y^2, 0, 0;
                       0, 0, model.robot_sigma_vel_x^2, 0
                       0, 0, 0, model.robot_sigma_vel_y^2;];
model.robot_acc_cov = [model.robot_sigma_a_x^2, 0;
                       0, model.robot_sigma_a_x^2];

% obstacle pos
model.obs_pos         = [ 0; -0.1];
% MPC parameters
dt              = 0.1;
N               = 20;
w_pos           = 8.0;                  % mpc weights
w_inputs        = 0.1;
w_coll          = 0.0;
w_slack         = 1E4;
% state matrices
model.A_matrix = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
model.B_matrix = [0 0; 0 0; dt 0; 0 dt];
model.state_robot_init = [model.robot_pos_start; 0 ;0];
% weights
model.weights = [1/4; 1/4; 1/8; 1/8; 1/8; 1/8];
model.weights_Horizon = [2/15 * ones(5,1); 2/45 * ones(5,1); 1/90 * ones(10,1)];

%% ploting figure
fig_main_origin = figure(1);              % main figure
hold on;
box on; 
grid on;
axis equal;
axis([pr.ws_x', pr.ws_y']);
xlabel('x [m]');
ylabel('y [m]')
ax_main_origin = fig_main_origin.CurrentAxes;
fig_ell_obs_origin = plot_ellipse_2D(ax_main_origin, model.obs_pos, model.obs_size, 0, ...
            'FaceColor', 'r', 'FaceAlpha', 0.4, ...
            'EdgeColor', 'r', 'EdgeAlpha', 0.2);
fig_robot_pos_origin = plot_ellipse_2D(ax_main_origin, model.robot_pos_start, model.robot_size, 0, ...
            'FaceColor', 'b', 'FaceAlpha', 0.8, ...
            'EdgeColor', 'b', 'EdgeAlpha', 0.8);
fig_robot_goal_origin = plot(ax_main_origin, model.robot_pos_goal(1), model.robot_pos_goal(2), ...
            'd', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r', ...
            'MarkerSize', 10);
fig_robot_mpc_path_origin = plot(ax_main_origin, model.robot_pos_start(1), model.robot_pos_start(2), ...
            'Color', 'c', 'LineStyle', '-', 'LineWidth', 2.0);
 
%% 

Options.N = 20;
Options.sampleN = 2e4;

length_path = size(planning_result.path_x, 1);
ColliProb  = zeros(length_path, 1);
ColliProb_AIS = zeros(length_path, 1);
ColliVar = zeros(length_path, 1);
ColliVar_AIS = zeros(length_path, 1);
time = zeros(length_path, 1);
fig_robot_path = scatter(planning_result.path_x(:,1), planning_result.path_y(:,1), '.', 'y');

% for i = 1:length_path
for i = 45
    
    model.state_robot(:,1) = model.state_robot_init;
    planning_result_sin.path_x = planning_result.path_x(i, :);
    planning_result_sin.path_y = planning_result.path_y(i, :);
    planning_result_sin.vel_x = planning_result.vel_x(i, :);
    planning_result_sin.vel_y = planning_result.vel_y(i, :);
    planning_result_sin.input_x = planning_result.input_x(i, :);
    planning_result_sin.input_y = planning_result.input_y(i, :);
    
%     [ColliProb_Group, ColliProb_IS_Group, ColliVar_Group, ColliVar_IS_Group, time_Group] = mcCollisionProbability_Tra_IdenPoint(model, ...
%                                         planning_result_sin, Options, ax_main_origin);
                                    
    [ColliProb(i), ColliProb_AIS(i), ColliVar(i), ColliVar_AIS(i), time(i)] = mcCollisionProbability_Tra_IdenPoint(model, ...
                                        planning_result_sin, Options, ax_main_origin);
%     plot_ellipse_2D(ax_main_origin, [planning_result_sin.path_x; planning_result_sin.path_y], ...
%             [ColliProb(i) * 5, ColliProb(i) * 5], 0, ...
%             'FaceColor', 'g', 'FaceAlpha', 0.4, ...
%             'EdgeColor', 'g', 'EdgeAlpha', 0.2);
%     plot_ellipse_2D(ax_main_AIS, [planning_result_sin.path_x; planning_result_sin.path_y], ...
%             [ColliVar_AIS, ColliVar_AIS], 0, ...
%             'FaceColor', 'g', 'FaceAlpha', 0.4, ...
%             'EdgeColor', 'g', 'EdgeAlpha', 0.2);
%     set(fig_robot_path_AIS, 'XData', planning_result_sin.path_x, ...
%         'YData', planning_result_sin.path_y);
%     scatter(planning_result.path_x(i,:), planning_result.path_y(i,:), '.', 'b');
   

%     fprintf('waypoint = %d, CorProb = %f.3, CorVar = %f.3\n', i, ColliProb(i), ColliVar(i));
end



