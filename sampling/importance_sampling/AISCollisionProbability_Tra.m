%% get collision probability
function [ColliProbability, ColliVar, time] = ...
        AISCollisionProbability_Tra(model, planning_result_sin, OptionsAIS, ax_main_origin)
%     Input:  model        -- the simulation model of robot and obstacles
%             planning_result_sin  -- trajectory data 
%             OptionsAIS   -- self-defined options of AIS, including the
%                             number of iterations, samples, etc
%             ax_main_origin       -- the name figure 
%     Output: ColliProbability -- collision probability of the whole path
%             ColliVar         -- covariance of collision probability
%             time             -- simulation time
    
    tic
    
    % planning options
    sampleN = OptionsAIS.sampleN;
    N = OptionsAIS.N;
    iteration_l = OptionsAIS.iteration_l;
    % step size of adaptive IS
    stepSize = 0.05;
    
    state_robot = zeros(4, sampleN, iteration_l, N);
    for k_ite = 1:iteration_l
        for i_sample = 1:sampleN
            state_robot(:, i_sample, k_ite, 1) = [planning_result_sin.path_x(1); 
                planning_result_sin.path_y(1); planning_result_sin.vel_x(1); planning_result_sin.vel_y(1)];
        end
    end
    
    if_Collision = zeros(sampleN, iteration_l, N);
    AIS = zeros(sampleN, iteration_l, N);
    pdf_IS = zeros(sampleN, iteration_l, N);
    pdf_Multi = zeros(sampleN, iteration_l, N);
%     fig_robot_path_2 = scatter(planning_result_sin.path_x, planning_result_sin.path_y, '.', 'r');

    % generate random samples of robot states
    rand_x = normrnd(0, model.robot_sigma_x, [sampleN, iteration_l, N]);
    rand_y = normrnd(0, model.robot_sigma_y, [sampleN, iteration_l, N]);
    rand_vel_x = normrnd(0, model.robot_sigma_vel_x, [sampleN, iteration_l, N]);
    rand_vel_y = normrnd(0, model.robot_sigma_vel_y, [sampleN, iteration_l, N]);
    rand_pdf_x = normpdf(rand_x, 0, model.robot_sigma_x) / normpdf(0, 0, model.robot_sigma_x);
    rand_pdf_y = normpdf(rand_y, 0, model.robot_sigma_y) / normpdf(0, 0, model.robot_sigma_y);
    rand_pdf_vel_x = normpdf(rand_vel_x, 0, model.robot_sigma_vel_x) / normpdf(0, 0, model.robot_sigma_vel_x);
    rand_pdf_vel_y = normpdf(rand_vel_y, 0, model.robot_sigma_vel_y) / normpdf(0, 0, model.robot_sigma_vel_y);
    
    % generate random samples of robot accelerations
    rand_a_x = normrnd(0, model.robot_sigma_a_x, [sampleN, iteration_l, N]);
    rand_a_y = normrnd(0, model.robot_sigma_a_y, [sampleN, iteration_l, N]);
    rand_pdf_a_x = normpdf(rand_a_x, 0, model.robot_sigma_a_x) / normpdf(0, 0, model.robot_sigma_a_x);
    rand_pdf_a_y = normpdf(rand_a_y, 0, model.robot_sigma_a_y) / normpdf(0, 0, model.robot_sigma_a_y);
    
    ColliProbability_Group = zeros(1, N);
    ColliVar_Group = zeros(1, N);
    gradient_Group = zeros(6, sampleN, iteration_l, N);
    weights_AIS = zeros(6, iteration_l + 1, N);
    
    % loop for different horizons
    for j_horizon = 1:N
        
        % inherit previous weight values
        if j_horizon == 1
            weights_AIS(:, 1, j_horizon) = model.weights;
        else
            weights_AIS(:, 1, j_horizon) = weights_AIS(:, end, j_horizon-1);
        end
            
        % loop for different iterations
        for k_ite = 1:iteration_l
            
            gradient_iteration = zeros(6,1);
            
            % loop for different samples
            for i_sample = 1:sampleN
                
                % generate next stage value with uncertainty
                randpos_robot = [rand_x(i_sample, k_ite, j_horizon); rand_y(i_sample, k_ite, j_horizon); 
                                 rand_vel_x(i_sample, k_ite, j_horizon); rand_vel_y(i_sample, k_ite, j_horizon)];
                randacc_robot = [rand_a_x(i_sample, k_ite, j_horizon); rand_a_y(i_sample, k_ite, j_horizon)];
                acc_sin = randacc_robot + [planning_result_sin.input_x(j_horizon); planning_result_sin.input_y(j_horizon)];
                state_robot(:, i_sample, k_ite, j_horizon+1) = model.A_matrix * ...
                    state_robot(:, i_sample, k_ite, j_horizon) + model.B_matrix * acc_sin + randpos_robot;
                
                % indicator functions of collision
                if (j_horizon > 1) && if_Collision(i_sample, k_ite, j_horizon - 1) == 1
                    if_Collision(i_sample, k_ite, j_horizon) = 1;
                else
                    if_Collision(i_sample, k_ite, j_horizon) = CollisionCheck(state_robot(1:2, i_sample, k_ite, j_horizon+1), ...
                        model.robot_size, model.obs_pos, model.obs_size);
%                     if if_Collision(i_sample, k_ite, j_horizon) == 1
%                         fprintf('\n %d, %d, %d', j_horizon, k_ite, i_sample);
%                     end
                end
                
                
                gradient_pdf = [rand_pdf_x(i_sample, k_ite, j_horizon), rand_pdf_y(i_sample, k_ite, j_horizon), ...
                        rand_pdf_vel_x(i_sample, k_ite, j_horizon), rand_pdf_vel_y(i_sample, k_ite, j_horizon), ...
                        rand_pdf_a_x(i_sample, k_ite, j_horizon), rand_pdf_a_y(i_sample, k_ite, j_horizon)];
                
                % compute the pdf of multivariate Gaussian and importance
                % weights
                pdf_IS(i_sample, k_ite, j_horizon) = gradient_pdf * weights_AIS(:, k_ite, j_horizon);

                pdf_Multi(i_sample, k_ite, j_horizon) = mvnpdf([randpos_robot; randacc_robot], zeros(6,1), diag([model.robot_sigma_x; 
                        model.robot_sigma_y; model.robot_sigma_vel_x; model.robot_sigma_vel_y; 
                        model.robot_sigma_a_x; model.robot_sigma_a_y])) / ...
                        mvnpdf(zeros(6,1), zeros(6,1), diag([model.robot_sigma_x; 
                        model.robot_sigma_y; model.robot_sigma_vel_x; model.robot_sigma_vel_y; 
                        model.robot_sigma_a_x; model.robot_sigma_a_y])); 
                
                % compute the optimizing gradient of weights
                Constant_Gra = (if_Collision(i_sample, k_ite, j_horizon) * pdf_Multi(i_sample, k_ite, j_horizon) / ...
                    pdf_IS(i_sample, k_ite, j_horizon))^2 / pdf_IS(i_sample, k_ite, j_horizon);
                gradient_Group(:, i_sample, k_ite, j_horizon) = Constant_Gra * gradient_pdf;
                gradient_iteration = gradient_iteration + gradient_Group(:, i_sample, k_ite, j_horizon);
            end
           
            % generate new weights of next iteration
            gradient_iteration = -1 / sampleN * gradient_iteration;
            weights_AIS(:, k_ite + 1, j_horizon) = weights_AIS(:, k_ite, j_horizon) - ...
                stepSize / sqrt(iteration_l) * gradient_iteration;
            
        end
        
        % compute the collision probability and variance for one specific
        % stage point
        ColliProbability_Group(j_horizon) = sum(sum(if_Collision(:, :, j_horizon) .* pdf_Multi(:, :, j_horizon) ./ pdf_IS(:, :, j_horizon))) / ...
            sum(sum(pdf_Multi(:, :, j_horizon) ./ pdf_IS(:, :, j_horizon)));
        ColliVar_Group(j_horizon) = ...
            sum(sum(((if_Collision(:, :, j_horizon) - ColliProbability_Group(j_horizon)).* ...
            pdf_Multi(:, :, j_horizon) ./ pdf_IS(:, :, j_horizon)).^2))  / ...
            sum(sum(pdf_Multi(:, :, j_horizon) ./ pdf_IS(:, :, j_horizon)))^2 /iteration_l /sampleN;
    end
    
%     % visualize the probability as circles in main figure
%     for k = 1:N
%         plot_ellipse_2D(ax_main_origin, [planning_result_sin.path_x(k); planning_result_sin.path_y(k)], ...
%             [ColliProbability_Group(k)*Constant, ColliProbability_Group(k)*Constant], ...
%             0, 'FaceColor', 'g', 'FaceAlpha', 0.4, 'EdgeColor', 'g', 'EdgeAlpha', 0.2);
%     end
    
    
    ColliProbability = ColliProbability_Group * model.weights_Horizon;
    ColliVar = ColliVar_Group * model.weights_Horizon;
    
%     ColliProbability = ColliProbability_Group;
%     ColliVar = ColliVar_Group;
    
    % the time of simulations
    time = toc;
   
end