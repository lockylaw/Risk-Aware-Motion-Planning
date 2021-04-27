%% get collision probability
function [ColliProbability, ColliProbability_AIS, ColliVar, ColliVar_AIS, time] = ...
        mcCollisionProbability_Tra_IdenPoint(model, planning_result_sin, Options, ax_main_origin)
%     Input:  model        -- the simulation model of robot and obstacles
%             planning_result_sin  -- trajectory data 
%             Options      -- self-defined options of AIS, including the
%                             number of iterations, samples, etc
%             ax_main_origin       -- the name figure 
%     Output: ColliProbability -- collision probability of the whole path
%             ColliVar         -- covariance of collision probability
%             time             -- simulation time
    
    tic
    
    sampleN = Options.sampleN;
    N = Options.N;
    
    state_robot = zeros(4, sampleN, N);
    state_robot(:, :, 1) = [planning_result_sin.path_x(1); planning_result_sin.path_y(1);
                         planning_result_sin.vel_x(1); planning_result_sin.vel_y(1)]*ones(1, sampleN);
        
    if_Collision = zeros(sampleN, N);
    AIS = zeros(sampleN, N);
    pdf_IS = zeros(sampleN, N);
    pdf_Multi = zeros(sampleN, N);
    likelihood = zeros(sampleN, N);

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
    
    ColliProbability_Group = zeros(1, N);
    ColliProbability_AIS_Group = zeros(1, N);
    ColliVar_Group = zeros(1, N);
    ColliVar_AIS_Group = zeros(1, N);
    
    for j_horizon = 1:N
        
        for i_sample = 1:sampleN
            
            % generate random samples of robots positions
            randpos_robot = [rand_x(j_horizon, i_sample); rand_y(j_horizon, i_sample); 
                             rand_vel_x(j_horizon, i_sample); rand_vel_y(j_horizon, i_sample)];
            
            % generate random samples of robots accelerations
            randacc_robot = [rand_a_x(j_horizon, i_sample); rand_a_y(j_horizon, i_sample)];
            acc_sin = randacc_robot + [planning_result_sin.input_x(j_horizon); planning_result_sin.input_y(j_horizon)];
            
            state_robot(:, i_sample, j_horizon+1) = model.A_matrix * state_robot(:, i_sample, j_horizon) + ...
                model.B_matrix * acc_sin + randpos_robot;
            
            if (j_horizon > 1) && if_Collision(i_sample, j_horizon-1) == 1
                if_Collision(i_sample, j_horizon) = 1;
            else
                if_Collision(i_sample, j_horizon) = CollisionCheck(state_robot(1:2, i_sample, j_horizon+1), ...
                    model.robot_size, model.obs_pos, model.obs_size);
            end
            
            pdf_IS(i_sample, j_horizon) = [rand_pdf_x(j_horizon, i_sample), rand_pdf_y(j_horizon, i_sample), ...
                      rand_pdf_vel_x(j_horizon, i_sample), rand_pdf_vel_y(j_horizon, i_sample), ...
                      rand_pdf_a_x(j_horizon, i_sample), rand_pdf_a_y(j_horizon, i_sample)] * model.weights;
            
            pdf_Multi(i_sample, j_horizon) = mvnpdf([randpos_robot; randacc_robot], zeros(6,1), diag([model.robot_sigma_x; 
                                model.robot_sigma_y; model.robot_sigma_vel_x; model.robot_sigma_vel_y; 
                                model.robot_sigma_a_x; model.robot_sigma_a_y])) / ...
                                mvnpdf(zeros(6,1), zeros(6,1), diag([model.robot_sigma_x; 
                                model.robot_sigma_y; model.robot_sigma_vel_x; model.robot_sigma_vel_y; 
                                model.robot_sigma_a_x; model.robot_sigma_a_y])); 
                            
            likelihood(i_sample, j_horizon) = pdf_Multi(i_sample, j_horizon)/pdf_IS(i_sample, j_horizon);
            
        end
        AIS(:, j_horizon) = if_Collision(:, j_horizon) .* pdf_Multi(:, j_horizon) ./ pdf_IS(:, j_horizon) ;
        ColliProbability_Group(j_horizon) = sum(if_Collision(:, j_horizon)) / sampleN;
        ColliProbability_AIS_Group(j_horizon) = sum(AIS(:, j_horizon)) / sampleN;  
        ColliVar_Group(j_horizon) = sum((if_Collision(:, j_horizon) - ColliProbability_Group(j_horizon)).^2) / sampleN^2;
        ColliVar_AIS_Group(j_horizon) = sum((if_Collision(:, j_horizon) .* likelihood(:, j_horizon) - ColliProbability_AIS_Group(j_horizon)).^2) / sampleN^2;
        
    end
    
    Constant = 0.1 / max(ColliProbability_Group);
    
%     for k = 1:N
%         plot_ellipse_2D(ax_main_origin, [planning_result_sin.path_x(k); planning_result_sin.path_y(k)], ...
%             [ColliProbability_Group(k)*Constant, ColliProbability_Group(k)*Constant], ...
%             0, 'FaceColor', 'g', 'FaceAlpha', 0.4, 'EdgeColor', 'g', 'EdgeAlpha', 0.2);
%     end
    
    
    ColliProbability = ColliProbability_Group * model.weights_Horizon;
    ColliProbability_AIS = ColliProbability_AIS_Group * model.weights_Horizon;
    
    ColliVar = ColliVar_Group * model.weights_Horizon;
    ColliVar_AIS = ColliVar_AIS_Group * model.weights_Horizon;
    
%     ColliProbability = ColliProbability_Group;
%     ColliProbability_AIS = ColliProbability_AIS_Group;
%     
%     ColliVar = ColliVar_Group;
%     ColliVar_AIS = ColliVar_AIS_Group;
    
    % the time of simulations
    time = toc;
   
end