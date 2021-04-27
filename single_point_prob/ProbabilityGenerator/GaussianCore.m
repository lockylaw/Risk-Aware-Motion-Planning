%% get collision probability
function [rand_x, rand_y] = GaussianCore(model, N, fig)
%     Input:  model -- the simulation model of robot and obstacles
%             n     -- the number of samples
%     Output: ColliProbability -- collision probability
%             time             -- simulation time

    ratio = 0.6;
    robot_mu = [-0.02 + model.robot_pos(1) -0.01 + model.robot_pos(2); 
                0.01 + model.robot_pos(1) 0.015 + model.robot_pos(2)]; 
    robot_sigma = cat(3, [0.05, 0.03], [0.04, 0.06]);                      
    p = [ratio, 1-ratio];
    gm = gmdistribution(robot_mu, robot_sigma, p);
    
    % generate random samples of robots
    rand = random(gm, N);
    
    hold on
    scatter(fig.ax_main, rand(:, 1), rand(:, 2), '.');
    
    rand_x = rand(:,1);
    rand_y = rand(:,2);
   
end