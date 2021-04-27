clear all
close all
clc 

%% probelm setup for the collision avoidance scenario
% the global pr struct should not be changed during simulation
global pr
% workspace environment
pr.ws_x         = [-5; 5];
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
model.robot_vel_start = [4; 0];              % 2x1
model.robot_pos_goal  = [ 4; 1.2];              % 2x1

model.robot_sigma_x = 0.01;                      % robot variance of x
model.robot_sigma_y = 0.02;                      % robot variance of y
model.robot_sigma_vel_x = 0.01;                 % robot variance of v_x
model.robot_sigma_vel_y = 0.01;                 % robot variance of v_y
model.robot_sigma_a_x = 0.1;                 % robot variance of theta
model.robot_sigma_a_y = 0.2;                 % robot variance of theta

model.robot_pos_cov = [model.robot_sigma_x^2, 0, 0, 0; 
                       0, model.robot_sigma_y^2, 0, 0;
                       0, 0, model.robot_sigma_vel_x^2, 0
                       0, 0, 0, model.robot_sigma_vel_y^2;];
model.robot_acc_cov = [model.robot_sigma_a_x^2, 0;
                       0, model.robot_sigma_a_x^2];

% obstacle pos

% MPC parameters
T               = 2;
N               = 20;
dt              = T/N;
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
axis([pr.ws_x', pr.ws_y']);
xlabel('x [m]');
ylabel('y [m]')
ax_main_origin = fig_main_origin.CurrentAxes;
set(ax_main_origin, 'FontSize', 24);
for i = 1:model.obs_num
    fig.fig_ell_obs = plot_square_2D(ax_main_origin, model.obs_pos(:, i), model.obs_size(:, i), ...
                        'FaceColor', [1, 0, 0, 0.4], 'EdgeColor', [1, 0, 0, 0.6]);
end
fig_robot_pos_origin = plot_ellipse_2D(ax_main_origin, model.robot_pos_start, model.robot_size, 0, ...
            'FaceColor', 'b', 'FaceAlpha', 0.8, ...
            'EdgeColor', 'b', 'EdgeAlpha', 0.8);
 
%% 

sampleN         = 1e4;
waypointNum     = 20;
T               = 2;
dt              = T/waypointNum;
% state matrices
model.A_matrix = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
model.B_matrix = [0 0; 0 0; dt 0; 0 dt];
model.state_robot_init = [model.robot_pos_start; 0 ;0];


state_robot_origin = zeros(4, waypointNum);

if_Collision = zeros(waypointNum, sampleN);
if_Collision_path = zeros(1, sampleN);

% scaPlot = scatter(ax_main_origin, state_robot_origin(1, :), state_robot_origin(2, :), '.');

model.robot_sigma_pos = diag([model.robot_sigma_x^2, model.robot_sigma_y^2]);

    
for j = 1: sampleN
    
    [state_robot,  time] = PathGenerate(model, waypointNum, ax_main_origin);
    
    for i = 1:waypointNum
        rand = mvnrnd([0,0], model.robot_sigma_pos);
        if_Collision(i, j) = ColliCheck([state_robot(1, i); state_robot(2, i)], model);
    end
%     set(scaPlot, 'xData', state_robot(1, :), 'yData', state_robot(2, :));
%     pause(0.2)
end

for j = 1: sampleN
    if sum(if_Collision(:, j)) > 0
        if_Collision_path(j) = 1;
    end
end


ColliProb_Path = sum(if_Collision_path) / sampleN;
ColliProb = sum(if_Collision, 2) / sampleN;
ColliProb_sum = sum(ColliProb);
ColliProb_prod = 1 - prod(1 - ColliProb);

%%

sampleN         = 50;
a = [0, -1];
a = a';
b = -0.5;

if_Collision_trun = zeros(waypointNum, sampleN);


for j = 1: sampleN
    delta_pos = [0;0];
    state_robot = zeros(4, waypointNum);
    state_robot(:, 1) = [model.robot_pos_start; model.robot_vel_start];
    sigma_pos = diag([model.robot_sigma_x^2, model.robot_sigma_y^2]);
    pd_x = makedist('Normal', 'mu', 0, 'sigma', model.robot_sigma_x);
    pd_y = makedist('Normal', 'mu', 0, 'sigma', model.robot_sigma_y);
    pd_vel_x = makedist('Normal', 'mu', 0, 'sigma', model.robot_sigma_vel_x);
    pd_vel_y = makedist('Normal', 'mu', 0, 'sigma', model.robot_sigma_vel_y);
    pd_a_x = makedist('Normal', 'mu', 0, 'sigma', model.robot_sigma_a_x);
    pd_a_y = makedist('Normal', 'mu', 0, 'sigma', model.robot_sigma_a_y);
    for i = 1:waypointNum
        rand_x = random(pd_x);
        rand_y = random(pd_y);
        rand_vel_x = random(pd_vel_x);
        rand_vel_y = random(pd_vel_y);
        rand_a_x = random(pd_a_x);
        rand_a_y = random(pd_a_y);
        
        % generate random samples of robots positions
        randpos_robot = [rand_x; rand_y; 
                         rand_vel_x; rand_vel_y];

        % generate random samples of robots accelerations
        randacc_robot = [rand_a_x; rand_a_y];
        acc_sin = randacc_robot + [0.1; 0];

        state_robot(:, i+1) = model.A_matrix * state_robot(:, i) + ...
            model.B_matrix * acc_sin + randpos_robot;
        
        if_Collision_trun(i, j) = ColliCheck([state_robot(1, i); state_robot(2, i)], model);
        
        if if_Collision_trun(i, j) == 1
            const_sqrt = sqrt(a' * sigma_pos * a);
            const_sqrt_inv = const_sqrt^(-1);
            beta = (b - a' *[rand(1); rand(2)]) * const_sqrt_inv;
            lambda = mvnpdf(beta)/ mvncdf(beta);
            delta_x = lambda * const_sqrt_inv * sigma_pos * a;
            delta_sigma = lambda*(lambda - beta) / const_sqrt_inv^2 * (sigma_pos*a) * (sigma_pos*a)';
            sigma_pos = sigma_pos - delta_sigma;
            delta_pos = delta_pos - delta_x;
            pd_y = truncate(pd_y, -delta_pos(2), delta_pos(2));
        end
    end
end


ColliProb_trun = sum(if_Collision_trun, 2) / sampleN;
ColliProb_trun_prod = 1 - prod(1 - ColliProb_trun);


