function [rand_x, rand_y, pdf] = GaussianDist(model, N, fig)
%     Input:  model -- the simulation model of robot and obstacles
%             n     -- the number of samples
%     Output: ColliProbability -- collision probability
%             time             -- simulation time
  
    
    pd = mvnrnd([model.robot_pos(1), model.robot_pos(2)], ...
        diag([model.robot_sigma_x^2, model.robot_sigma_y^2]), N);
    pdf = mvnpdf(pd, [model.robot_pos(1), model.robot_pos(2)], ...
        diag([model.robot_sigma_x^2, model.robot_sigma_y^2]));
    
    % generate random samples of robots
    rand_x = pd(:, 1);
    rand_y = pd(:, 2);
    
%     hold on
%     scatter(fig.ax_main, rand_x, rand_y, '.');
   
end

