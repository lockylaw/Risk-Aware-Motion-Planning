clear all
close all
clc 

sample_random = 1e3;
i_sample_random = 1;
colliProb_MC = zeros(sample_random, 1);
colliProb_QBS = zeros(sample_random, 1);
colliProb_AIS = zeros(sample_random, 1);
colliProb_IS = zeros(sample_random, 1);
model_cell = cell(sample_random, 1);

while(i_sample_random  <= sample_random)
    %% Monte Carlo: compute probability with different sampling numbers
    %  Working Environment: 1 robot, 1/2 static osbtacles with/without
    %  uncertainty

    FlagGene = 1;

    while(FlagGene)
        
        % robot informations
        model.robot_pos = -3 + (3+3)*rand(2,1);         % robot position mean
        model.robot_sigma_x = 0.3 + 1 * rand(1);                      % robot variance of x
        model.robot_sigma_y = 0.3 + 1 * rand(1);                      % robot variance of y
        model.robot_sigma_theta = 0.01;                 % robot variance of theta
        model.robot_rho_xtheta= 0.0;                    % robot covariance of x and theta
        model.robot_rho_ytheta= 0.0;                    % robot covariance of y and theta
        model.robot_rho_xy= 0.0;                        % robot covariance of x and y

        % covariance matrix/Sigma
        model.robot_pos_cov = [model.robot_sigma_x^2, model.robot_rho_xy*model.robot_sigma_x*model.robot_sigma_y; 
                         model.robot_rho_xy*model.robot_sigma_x*model.robot_sigma_y, model.robot_sigma_y^2;];

        % robot size
        model.robot_size = 1 + 1 * rand(1) * [1; 1];

        % obstacle number

        model.obs_num = 1 + ceil(3 * rand());
        model.obs_shape = "ellip";
        model.obs_size = zeros(2, model.obs_num);
        model.obs_pos = zeros(2, model.obs_num);
        for i = 1:model.obs_num
            if model.obs_num == 2 && round(rand(1)) == 1
                model.obs_size(:, i) = 1 + 1 * rand(2,1);  
                max_size = max(model.robot_size) + 2;
                max_obs_size = max(model.obs_size(:, i));
                if i == 1
                    model.obs_pos(:, i) = [-max_obs_size + 2 * max_obs_size * rand(1,1) + max_size;
                        -max_obs_size + 2 * max_obs_size * rand(1,1)]; 
                elseif i == 2
                    model.obs_pos(:, i) = [-max_obs_size + 2 * max_obs_size * rand(1,1) - max_size;
                        -max_obs_size + 2 * max_obs_size * rand(1,1)]; 
                end
            elseif model.obs_num == 3 && round(rand(1)) == 1
                model.obs_size(:, i) = 1 + 1 * rand(2,1);  
                max_size = max(model.robot_size) + 2;
                max_obs_size = max(model.obs_size(:, i));
                if i == 1
                    model.obs_pos(:, i) = [-max_obs_size + 2 * max_obs_size * rand(1,1) + max_size;
                        -max_obs_size + 2 * max_obs_size * rand(1,1) + max_size]; 
                elseif i == 2
                    model.obs_pos(:, i) = [-max_obs_size + 2 * max_obs_size * rand(1,1) - max_size;
                        -max_obs_size + 2 * max_obs_size * rand(1,1)]; 
                elseif i == 3
                    model.obs_pos(:, i) = [-max_obs_size + 2 * max_obs_size * rand(1,1) + max_size;
                        -max_obs_size + 2 * max_obs_size * rand(1,1) - max_size]; 
                end
            elseif model.obs_num == 4 && round(rand(1)) == 1
                model.obs_size(:, i) = 1 + 1 * rand(2,1);  
                max_size = max(model.robot_size) + 2;
                max_obs_size = max(model.obs_size(:, i));
                if i == 1
                    model.obs_pos(:, i) = [-max_obs_size + 2 * max_obs_size * rand(1,1) + max_size;
                        -max_obs_size + 2 * max_obs_size * rand(1,1) + max_size]; 
                elseif i == 2
                    model.obs_pos(:, i) = [-max_obs_size + 2 * max_obs_size * rand(1,1) - max_size;
                        -max_obs_size + 2 * max_obs_size * rand(1,1) + max_size]; 
                elseif i == 3
                    model.obs_pos(:, i) = [-max_obs_size + 2 * max_obs_size * rand(1,1) + max_size;
                        -max_obs_size + 2 * max_obs_size * rand(1,1) - max_size]; 
                elseif i == 4
                    model.obs_pos(:, i) = [-max_obs_size + 2 * max_obs_size * rand(1,1) - max_size;
                        -max_obs_size + 2 * max_obs_size * rand(1,1) - max_size]; 
                end
            else
                model.obs_size(:, i) = 1 + 1 * rand(2,1);  
                max_obs_size = max(model.obs_size(:, i));
                model.obs_pos(:, i) = -max_obs_size + 2 * max_obs_size * rand(2,1);  
            end
        end


        % whether the uncertainties of obstacles are considered
        model.if_obsEllip = 1;
        
        if ColliCheck_Ellip(model.robot_pos, model, "obs_check") == 0
            FlagGene = 0;
        end

    end

    % Probability Type
    % -- 1 -- Gaussian 
    % -- 2 -- Gaussian Mixture Model
    % -- 3 -- Truncated Gaussian


    %% plot figure of distribution of robot and obstacles
    
%     fprintf('\n------- sample %d -------', i_sample_random);
%     fprintf('\nobs_num = %.3f', model.obs_num);
%     fprintf('\n-------------------------\n');

%     fig = plot_figure(model);
    
%     pause(0.2);

    %%
    sampleN = 1e5;
    tag = 1;
    colliProb_MC(i_sample_random) = MonteCarlo(sampleN, model);
    
    if colliProb_MC(i_sample_random) < 0.03 || colliProb_MC(i_sample_random) > 0.95
        i_sample_random = i_sample_random - 1;
        tag = 0;
    end
    
    if(tag)
        %%
        sampleN_QBS = 40;
        colliProb_QBS(i_sample_random) = QuadratureBasedSampling(model, sampleN_QBS);

        %%
        OptionsAIS.sampleN = 1e3;
        OptionsAIS.iteration_l = 10;
        colliProb_AIS(i_sample_random) = AdaptiveImportanceSampling(model, OptionsAIS);
        
        OptionsIS.sampleN = 1e3;
        OptionsIS.iteration_l = 1;
        colliProb_IS(i_sample_random) = AdaptiveImportanceSampling(model, OptionsIS);
        
        fprintf('\n------- sample %d -------', i_sample_random);
        fprintf('\nobs_num = %.3f', model.obs_num);
        fprintf('\nMonte Carlo = %.3f, QBS = %.3f, AIS = %.3f, IS = %.3f', ...
            colliProb_MC(i_sample_random), colliProb_QBS(i_sample_random), ...
            colliProb_AIS(i_sample_random), colliProb_IS(i_sample_random));
        fprintf('\n-------------------------\n');
        
        model_cell(i_sample_random) = {model};
    end
    
    %%
    i_sample_random = i_sample_random + 1;
    tag = 1;
    
end





