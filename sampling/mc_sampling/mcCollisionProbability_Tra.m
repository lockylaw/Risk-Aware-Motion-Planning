%% get collision probability
function [ColliProbability, ColliProbability_AIS, ColliVar, ColliVar_AIS, time] = ...
        mcCollisionProbability_Tra(model, planning_result_sin, sampleN, N)
%     Input:  model     -- the simulation model of robot and obstacles
%             sampleN   -- the number of samples
%             N         -- the length of horizons
%     Output: ColliProbability -- collision probability
%             time             -- simulation time
    
    tic
    
    state_robot = zeros(4, N);
    state_robot(:, 1) = [planning_result_sin.path_x(1); planning_result_sin.path_y(1);
                         planning_result_sin.vel_x(1); planning_result_sin.vel_y(1)];
    
    % generate random samples of robots
    R_robot_pos = chol(model.robot_pos_cov);
    R_robot_acc = chol(model.robot_acc_cov);
    
    ColliNum = zeros(sampleN, 1);
    AIS = zeros(sampleN, 1);
%     fig_robot_path_2 = scatter(state_robot(1, :), state_robot(2, :), '.', 'r');

    % generate random samples of robots positions
    rand_x = normrnd(0, model.robot_sigma_x, [N, sampleN]);
    rand_y = normrnd(0, model.robot_sigma_y, [N, sampleN]);
    rand_vel_x = normrnd(0, model.robot_sigma_vel_x, [N, sampleN]);
    rand_vel_y = normrnd(0, model.robot_sigma_vel_y, [N, sampleN]);
    rand_pdf_x = normpdf(rand_x, 0, model.robot_sigma_x) / normpdf(0, 0, model.robot_sigma_x);
    rand_pdf_y = normpdf(rand_y, 0, model.robot_sigma_y) / normpdf(0, 0, model.robot_sigma_y);
    rand_pdf_vel_x = normpdf(rand_vel_x, 0, model.robot_sigma_vel_x) / normpdf(0, 0, model.robot_sigma_vel_x);
    rand_pdf_vel_y = normpdf(rand_vel_y, 0, model.robot_sigma_vel_y) / normpdf(0, 0, model.robot_sigma_vel_y);
    
    % generate random samples of robots accelerations
    rand_a_x = normrnd(0, model.robot_sigma_a_x, [N, sampleN]);
    rand_a_y = normrnd(0, model.robot_sigma_a_y, [N, sampleN]);
    rand_pdf_a_x = normpdf(rand_a_x, 0, model.robot_sigma_a_x) / normpdf(0, 0, model.robot_sigma_a_x);
    rand_pdf_a_y = normpdf(rand_a_y, 0, model.robot_sigma_a_y) / normpdf(0, 0, model.robot_sigma_a_y);
    
    for i_sample = 1:sampleN
        
        for j_horizon = 1:N
            
            % generate random samples of robots positions
            randpos_robot = [rand_x(j_horizon, i_sample); rand_y(j_horizon, i_sample); 
                             rand_vel_x(j_horizon, i_sample); rand_vel_y(j_horizon, i_sample)];
            
            % generate random samples of robots accelerations
            randacc_robot = [rand_a_x(j_horizon, i_sample); rand_a_y(j_horizon, i_sample)];
            acc_sin = randacc_robot + [planning_result_sin.input_x(j_horizon); planning_result_sin.input_y(j_horizon)];
            
            state_robot(:, j_horizon+1) = model.A_matrix * state_robot(:, j_horizon) + ...
                model.B_matrix * acc_sin + randpos_robot;
            
            if_Collision = CollisionCheck(state_robot(1:2, j_horizon+1), model.robot_size, ... 
                                          model.obs_pos, model.obs_size);
            
            pdf_IS = [rand_pdf_x(j_horizon, i_sample), rand_pdf_y(j_horizon, i_sample), ...
                      rand_pdf_vel_x(j_horizon, i_sample), rand_pdf_vel_y(j_horizon, i_sample), ...
                      rand_pdf_a_x(j_horizon, i_sample), rand_pdf_a_y(j_horizon, i_sample)] * model.weights;
            
            pdf_Multi = mvnpdf([randpos_robot; randacc_robot], zeros(6,1), diag([model.robot_sigma_x; 
                                model.robot_sigma_y; model.robot_sigma_vel_x; model.robot_sigma_vel_y; 
                                model.robot_sigma_a_x; model.robot_sigma_a_y])) / ...
                        mvnpdf(zeros(6,1), zeros(6,1), diag([model.robot_sigma_x; 
                                model.robot_sigma_y; model.robot_sigma_vel_x; model.robot_sigma_vel_y; 
                                model.robot_sigma_a_x; model.robot_sigma_a_y])); 
                             
            likelihood = if_Collision * pdf_Multi / pdf_IS ;
                  
                                      
            if if_Collision == 1
                ColliNum(i_sample) = 1;
                AIS(i_sample) = likelihood;
                fprintf('\n sample = %d , (%.3f, %.3f)', j_horizon, rand_pdf_x(j_horizon, i_sample), rand_pdf_y(j_horizon, i_sample));
                break
            end
        end
%         if mod(i_sample, 100) == 0
%             fprintf('sample = %d \n', i_sample);
%         end
    end
    
    ColliProbability = sum(ColliNum) / sampleN;
    ColliProbability_AIS = sum(AIS) / sampleN;
    
    ColliVar = sum((ColliProbability*ones(sampleN, 1) - ColliNum).^2) / sampleN;
    ColliVar_AIS = sum((ColliProbability_AIS*ones(sampleN, 1) - AIS).^2) / sampleN;
    
    % the time of simulations
    time = toc;
   
end