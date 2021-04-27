clear all
close all
clc 

%% probelm setup for the collision avoidance scenario
% the global pr struct should not be changed during simulation
global pr
% workspace environment
pr.ws_x         = [-10; 10];
pr.ws_y         = [-2; 4];
% robot physics
pr.robot_dynamics_continuous  = @double_integrator_dynamics_continuous;	% dynamics
pr.robot_maxVx  = 2.0;
pr.robot_maxVy  = 2.0;
pr.robot_maxAx  = 1.0;
pr.robot_maxAy  = 1.0;
% obstacle physic
model.obs_num         = 1;
model.obs_pos         = [0; -1];
model.obs_size        = [10; 2];         	% [a, b]
model.obs_scale       = 1.6;                  % collision potential scale
% robot start and goal postion
model.robot_size      = [0.5; 0.5];
model.robot_pos_start = [-4; 0.8];              % 2x1
model.robot_vel_start = [0.5; 0];              % 2x1
model.robot_pos_goal  = [ 4; 1.2];              % 2x1

model.robot_sigma_x = 0.1;                      % robot variance of x
model.robot_sigma_y = 0.2;                      % robot variance of y
model.robot_sigma_vel_x = 0.1;                 % robot variance of v_x
model.robot_sigma_vel_y = 0.1;                 % robot variance of v_y
model.robot_sigma_a_x = 0.1;                 % robot variance of theta
model.robot_sigma_a_y = 0.05;                 % robot variance of theta

model.robot_pos_cov = [model.robot_sigma_x^2, 0, 0, 0; 
                       0, model.robot_sigma_y^2, 0, 0;
                       0, 0, model.robot_sigma_vel_x^2, 0
                       0, 0, 0, model.robot_sigma_vel_y^2;];
model.robot_acc_cov = [model.robot_sigma_a_x^2, 0;
                       0, model.robot_sigma_a_x^2];

% obstacle pos

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

%% ploting figure
fig_main_origin = figure(1);              % main figure
hold on;
box on; 
grid on;
axis equal;
% axis([pr.ws_x', pr.ws_y']);
xlabel('x [m]');
ylabel('y [m]')
ax_main_origin = fig_main_origin.CurrentAxes;
for i = 1:model.obs_num
    fig.fig_ell_obs = plot_square_2D(ax_main_origin, model.obs_pos(:, i), model.obs_size(:, i), ...
                        'FaceColor', [1, 0, 0, 0.4], 'EdgeColor', [1, 0, 0, 0.6]);
end
fig_robot_pos_origin = plot_ellipse_2D(ax_main_origin, model.robot_pos_start, model.robot_size, 0, ...
            'FaceColor', 'b', 'FaceAlpha', 0.8, ...
            'EdgeColor', 'b', 'EdgeAlpha', 0.8);
 
%% 

sampleN = 1e4;
waypointNum = 100;
state_robot_origin = zeros(2, waypointNum);
state_robot_origin(1, :) = linspace(model.robot_pos_start(1), ...
    model.robot_pos_goal(1), waypointNum);
state_robot_origin(2, :) = linspace(model.robot_pos_start(2), ...
    model.robot_pos_goal(2), waypointNum);

if_Collision = zeros(waypointNum, sampleN);
if_Collision_path = zeros(1, sampleN);

scatter(ax_main_origin, state_robot_origin(1, :), state_robot_origin(2, :), '.')

model.robot_sigma_pos = diag([model.robot_sigma_x^2, model.robot_sigma_y^2]);

    
% for j = 1: sampleN
%     for i = 1:waypointNum
%         rand = mvnrnd([0,0], model.robot_sigma_pos);
%         if_Collision(i, j) = ColliCheck([state_robot_origin(1, i) + rand(1); ...
%             state_robot_origin(2, i) + rand(2)], model);
%     end
% end
% 
% for j = 1: sampleN
%     if sum(if_Collision(:, j)) > 0
%         if_Collision_path(j) = 1;
%     end
% end


ColliProb_Path = sum(if_Collision_path) / sampleN;
ColliProb = sum(if_Collision, 2) / sampleN;
ColliProb_sum = sum(ColliProb);
ColliProb_prod = 1 - prod(1 - ColliProb);

%%

a = [0, -1];
a = a';
b = -0.5;

if_Collision_trun = zeros(waypointNum, sampleN);

for j = 1: sampleN
    delta_pos = [0;0];
    state_robot = state_robot_origin;
    sigma_pos = diag([model.robot_sigma_x^2, model.robot_sigma_y^2]);
    pd_x = makedist('Normal', 'mu', delta_pos(1), 'sigma', model.robot_sigma_x);
    pd_y = makedist('Normal', 'mu', delta_pos(2), 'sigma', model.robot_sigma_y);
    rand = zeros(waypointNum, 2);
    for i = 1:waypointNum
        rand_x = random(pd_x);
        rand_y = random(pd_y);
        rand(i, :) = [rand_x + state_robot(1, i), rand_y + state_robot(2, i)];
        if_Collision_trun(i, j) = ColliCheck([rand(i, 1); rand(i, 2)], model);
%         if if_Collision_trun(i, j) == 1
%             const_sqrt = sqrt(a' * sigma_pos * a);
%             const_sqrt_inv = const_sqrt^(-1);
%             beta = (b - a' *[rand(i, 1); rand(i, 2)]) * const_sqrt_inv;
%             lambda = mvnpdf(beta)/ mvncdf(beta);
%             delta_x = lambda * const_sqrt_inv * sigma_pos * a;
%             delta_sigma = lambda*(lambda - beta) / const_sqrt_inv^2 * (sigma_pos*a) * (sigma_pos*a)';
%             sigma_pos = sigma_pos - delta_sigma;
%             delta_pos = delta_pos - delta_x;
%             pd_y = truncate(pd_y, -delta_pos(2), delta_pos(2));
%         end
    end
    scatter(ax_main_origin, rand(:, 1), rand(: ,2), '.')
end


ColliProb_trun = sum(if_Collision_trun, 2) / sampleN;
ColliProb_trun_prod = 1 - prod(1 - ColliProb_trun);


