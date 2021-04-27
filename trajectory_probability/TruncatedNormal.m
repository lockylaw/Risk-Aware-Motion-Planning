function [rand_x, rand_y] = TruncatedNormal(model, N, fig)
%     Input:  model -- the simulation model of robot and obstacles
%             n     -- the number of samples
%     Output: ColliProbability -- collision probability
%             time             -- simulation time

    
    pd_x = makedist('Normal', 'mu', model.robot_pos(1), 'sigma', 0.2);
    pd_y = makedist('Normal', 'mu', model.robot_pos(2), 'sigma', 0.3);
    trunNorm_x = truncate(pd_x, model.robot_pos(1)-1, inf);
    trunNorm_y = truncate(pd_y, -inf, model.robot_pos(2)+1);
    
    % generate random samples of robots
    rand_x = random(trunNorm_x, N, 1);
    rand_y = random(trunNorm_y, N, 1);
    
    hold on
    scatter(fig.ax_main, rand_x, rand_y, '.');
    
   
end

