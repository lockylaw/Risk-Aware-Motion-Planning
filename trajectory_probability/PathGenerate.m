function [state_robot,  time] = ...
    PathGenerate(model, N, ax_main_origin)
%PATHGENERATE Summary of this function goes here
%   Detailed explanation goes here
    tic
    
    state_robot = zeros(4, N);
    state_robot(:, 1) = [model.robot_pos_start; model.robot_vel_start];
    
    for i = 1:N-1
        
        % generate random samples of robots positions
        rand_x = normrnd(0, model.robot_sigma_x);
        rand_y = normrnd(0, model.robot_sigma_y);
        rand_vel_x = normrnd(0, model.robot_sigma_vel_x);
        rand_vel_y = normrnd(0, model.robot_sigma_vel_y);

        % generate random samples of robots accelerations
        rand_a_x = normrnd(0, model.robot_sigma_a_x);
        rand_a_y = normrnd(0, model.robot_sigma_a_y);
    
        % generate random samples of robots positions
        randpos_robot = [rand_x; rand_y; 
                         rand_vel_x; rand_vel_y];

        % generate random samples of robots accelerations
        randacc_robot = [rand_a_x; rand_a_y];
        acc_sin = randacc_robot + [0.1; 0];

        state_robot(:, i+1) = model.A_matrix * state_robot(:, i) + ...
            model.B_matrix * acc_sin + randpos_robot;

    end
%     scatter(ax_main_origin, state_robot(1, :), state_robot(2, :), '.')
    time = toc;
end

