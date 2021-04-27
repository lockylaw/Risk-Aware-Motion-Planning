%% get collision probability
function [ColliProb, ColliVar] = ...
        AdaptiveImportanceSampling(model, OptionsAIS)
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
    iteration_l = OptionsAIS.iteration_l;
    % step size of adaptive IS
    stepSize = 0.05;
    
    rand_x = normrnd(model.robot_pos(1), model.robot_sigma_x, [sampleN, iteration_l]);
    rand_y = normrnd(model.robot_pos(2), model.robot_sigma_y, [sampleN, iteration_l]);
    rand_pdf_x = normpdf(rand_x, model.robot_pos(1), model.robot_sigma_x) ;
    rand_pdf_y = normpdf(rand_y, model.robot_pos(2), model.robot_sigma_y) ;
    
    
    if_Collision = zeros(sampleN, iteration_l);
    AIS = zeros(sampleN, iteration_l);
    pdf_IS = zeros(sampleN, iteration_l);
    pdf_Multi = zeros(sampleN, iteration_l);
%     fig_robot_path_2 = scatter(planning_result_sin.path_x, planning_result_sin.path_y, '.', 'r');

    gradient_Group = zeros(2, sampleN, iteration_l);
    weights_AIS = zeros(2, iteration_l + 1);
    
        
    % inherit previous weight values
    weights_AIS(:, 1) = [0.5; 0.5];
        
    % loop for different iterations
    for k_ite = 1:iteration_l

        gradient_iteration = zeros(2,1);

        % loop for different samples
        for i_sample = 1:sampleN

            % generate next stage value with uncertainty
            randpos_robot = [rand_x(i_sample, k_ite); rand_y(i_sample, k_ite)];

            % indicator functions of collision
            if_Collision(i_sample, k_ite) = ColliCheck(randpos_robot, model);
                
            gradient_pdf = [rand_pdf_x(i_sample, k_ite), rand_pdf_y(i_sample, k_ite)];

            % compute the pdf of multivariate Gaussian and importance
            % weights
            pdf_IS(i_sample, k_ite) = gradient_pdf * weights_AIS(:, k_ite);

            pdf_Multi(i_sample, k_ite) = mvnpdf(randpos_robot, ...
                    [model.robot_pos(1); model.robot_pos(2)], ...
                    diag([model.robot_sigma_x; model.robot_sigma_y])) ; 

            % compute the optimizing gradient of weights
            Constant_Gra = (if_Collision(i_sample, k_ite) * pdf_Multi(i_sample, k_ite) / ...
                pdf_IS(i_sample, k_ite))^2 / pdf_IS(i_sample, k_ite);
            gradient_Group(:, i_sample, k_ite) = Constant_Gra * gradient_pdf;
            gradient_iteration = gradient_iteration + gradient_Group(:, i_sample, k_ite);
        end

        % generate new weights of next iteration
        gradient_iteration = -1 / sampleN * gradient_iteration;
        weights_AIS(:, k_ite + 1) = weights_AIS(:, k_ite) - ...
            stepSize / sqrt(iteration_l) * gradient_iteration;

    end

    % compute the collision probability and variance for one specific
    % stage point
    ColliProb = sum(sum(if_Collision .* pdf_Multi ./ pdf_IS)) / ...
        sum(sum(pdf_Multi ./ pdf_IS));
    ColliVar = sum(sum(((if_Collision - ColliProb).* pdf_Multi ./ pdf_IS).^2)) / ...
        sum(sum(pdf_Multi ./ pdf_IS))^2 /iteration_l /sampleN;
    
    
    time = toc;
   
end