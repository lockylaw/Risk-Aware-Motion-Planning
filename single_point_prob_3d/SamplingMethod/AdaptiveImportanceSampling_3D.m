%% get collision probability
function ColliProb = AdaptiveImportanceSampling_3D(model, OptionsAIS)
%     Input:  model        -- the simulation model of robot and obstacles
%             planning_result_sin  -- trajectory data 
%             OptionsAIS   -- self-defined options of AIS, including the
%                             number of iterations, samples, etc
%             ax_main_origin       -- the name figure 
%     Output: ColliProbability -- collision probability of the whole path
%             ColliVar         -- covariance of collision probability
%             time             -- simulation time
    
    % planning options
    sampleN = OptionsAIS.sampleN;
    iteration_l = OptionsAIS.iteration_l;
    % step size of adaptive IS
    stepSize = 1e3;
    
    rand_x = normrnd(model.robot_pos(1), model.robot_sigma_x, [sampleN, iteration_l]);
    rand_y = normrnd(model.robot_pos(2), model.robot_sigma_y, [sampleN, iteration_l]);
    rand_z = normrnd(model.robot_pos(3), model.robot_sigma_z, [sampleN, iteration_l]);
    rand_pdf_x = normpdf(rand_x, model.robot_pos(1), model.robot_sigma_x) / ...
        normpdf(model.robot_pos(1), model.robot_pos(1), model.robot_sigma_x);
    rand_pdf_y = normpdf(rand_y, model.robot_pos(2), model.robot_sigma_y) / ...
        normpdf(model.robot_pos(2), model.robot_pos(2), model.robot_sigma_y);
    rand_pdf_z = normpdf(rand_z, model.robot_pos(3), model.robot_sigma_z) / ...
        normpdf(model.robot_pos(3), model.robot_pos(3), model.robot_sigma_z);
    
    
    if_Collision = zeros(sampleN, iteration_l);
    pdf_IS = zeros(sampleN, iteration_l);
    pdf_Multi = zeros(sampleN, iteration_l);
%     fig_robot_path_2 = scatter(planning_result_sin.path_x, planning_result_sin.path_y, '.', 'r');

    gradient_Group = zeros(3, sampleN, iteration_l);
    weights_AIS = zeros(3, iteration_l + 1);
    weights_AIS_log = zeros(3, iteration_l + 1);
        
    % inherit previous weight values
    weights_AIS(:, 1) = [1; 1; 1];
    weights_AIS_log(:, 1) = log(weights_AIS(:, 1));
    
    
    % loop for different iterations
    for k_ite = 1:iteration_l

        gradient_iteration = zeros(3,1);

        % loop for different samples
        for i_sample = 1:sampleN

            % generate next stage value with uncertainty
            randpos_robot = [rand_x(i_sample, k_ite); rand_y(i_sample, k_ite);
                             rand_z(i_sample, k_ite)];

            % indicator functions of collision
            if model.obs_shape == 'square'
                if_Collision(i_sample, k_ite) = ColliCheck_Square(randpos_robot, model);
            elseif model.obs_shape == 'ellip'
                if_Collision(i_sample, k_ite) = ColliCheck_Ellip(randpos_robot, model);
            end
                
            gradient_pdf = [rand_pdf_x(i_sample, k_ite), rand_pdf_y(i_sample, k_ite), ...
                            rand_pdf_z(i_sample, k_ite)];

            % compute the pdf of multivariate Gaussian and importance
            % weights
            pdf_IS(i_sample, k_ite) = gradient_pdf * weights_AIS(:, k_ite);

            pdf_Multi(i_sample, k_ite) = mvnpdf(randpos_robot, ...
                    [model.robot_pos(1); model.robot_pos(2); model.robot_pos(3)], ...
                    model.robot_pos_cov) / ...
                    mvnpdf([model.robot_pos(1); model.robot_pos(2); model.robot_pos(3)], ...
                    [model.robot_pos(1); model.robot_pos(2); model.robot_pos(3)], ...
                    model.robot_pos_cov); 

            % compute the optimizing gradient of weights
            Constant_Gra = (if_Collision(i_sample, k_ite) * pdf_Multi(i_sample, k_ite) / ...
                pdf_IS(i_sample, k_ite))^2 / pdf_IS(i_sample, k_ite);
            gradient_Group(:, i_sample, k_ite) = Constant_Gra * gradient_pdf;
            gradient_iteration = gradient_iteration + gradient_Group(:, i_sample, k_ite);
        
        end

        % generate new weights of next iteration
        gradient_iteration = -1 / sampleN * gradient_iteration;
        weights_AIS_log(:, k_ite + 1) = weights_AIS_log(:, k_ite) - ...
            stepSize / sqrt(iteration_l) * gradient_iteration;
        weights_AIS(:, k_ite + 1) = exp(weights_AIS_log(:, k_ite + 1));
    end

    % compute the collision probability and variance for one specific
    % stage point
    ColliProb = sum(sum(if_Collision .* pdf_Multi ./ pdf_IS)) / ...
        sum(sum(pdf_Multi ./ pdf_IS));
   
end