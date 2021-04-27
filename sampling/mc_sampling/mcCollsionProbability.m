%% get collision probability
function [ColliProbability, time] = mcCollsionProbability(model, N)
%     Input:  model -- the simulation model of robot and obstacles
%             n     -- the number of samples
%     Output: ColliProbability -- collision probability
%             time             -- simulation time
    
    tic
    
    % generate random samples of robots
    R_robot = chol(model.robot_pos_cov);
    randpos_robot = repmat(model.robot_pos.',N, 1) + randn(N,2)*R_robot;
    op_ColliProbability_single = zeros(model.obs_num, 1);
    
    % single obstacle
    if model.obs_num == 1
        ColliNum = 0;
        
        % generate random samples of obstacles
        R_obs = chol(model.obs_pos_cov);
        randpos_obs = repmat(model.obs_pos.',N, 1) + randn(N,2)*R_obs;
        
        % distance between robot and obstacles center
        dis = randpos_robot - randpos_obs;
        for i = 1:N
            disRandi =  sqrt(dis(i,1)^2 + dis(i,2)^2);
            % check whether the collison happens
            if disRandi > model.robot_box_size(1)/2 + model.obs_box_size(1)/2
                ColliNum = ColliNum + 1;
            end
        end
        ColliProbability = 1 - ColliNum / N;
        
    % multiple obstacles   
    else
        for j = 1:model.obs_num
            ColliNum = 0;
            
            % generate random samples of obstacles
            R_obs = chol(model.obs_pos_cov(:, :, j));
            randpos_obs = repmat(model.obs_pos(:,j).',N, 1) + randn(N,2)*R_obs;
            
            % distance between robot and obstacles center
            dis = randpos_robot - randpos_obs;
            for i = 1:N
                disRandi =  sqrt(dis(i,1)^2 + dis(i,2)^2);
                % check whether the collison happens
                if disRandi > model.robot_box_size(1)/2 + model.obs_box_size(1)/2
                    ColliNum = ColliNum + 1;
                end
            end
            op_ColliProbability_single(j) = ColliNum / N;
        end 
        ColliProbability = 1 - prod(op_ColliProbability_single);
    end
    
    % the time of simulations
    time = toc;
   
end