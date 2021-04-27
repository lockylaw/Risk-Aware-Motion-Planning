function [rand_x, rand_y] = TruncatedNormal(model, N)
%     Input:  model -- the simulation model of robot and obstacles
%             n     -- the number of samples
%     Output: ColliProbability -- collision probability
%             time             -- simulation time

    
    pd_x = makedist('Normal', 'mu', model.robot_pos(1), 'sigma', model.robot_sigma_x);
    pd_y = makedist('Normal', 'mu', model.robot_pos(2), 'sigma', model.robot_sigma_y);
    trunNorm_x = truncate(pd_x, model.robot_pos(1) - 5 * model.robot_sigma_x, inf);
    trunNorm_y = truncate(pd_y, -inf, model.robot_pos(2) + 5 * model.robot_sigma_x);
    
    % generate random samples of robots
    rand_x = random(trunNorm_x, N, 1);
    rand_y = random(trunNorm_y, N, 1);
    
   
end

