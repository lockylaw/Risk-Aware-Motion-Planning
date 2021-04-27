function pdf = TruncatedNormalPDF(robot_pos, model)
%     Input:  model -- the simulation model of robot and obstacles
%             n     -- the number of samples
%     Output: ColliProbability -- collision probability
%             time             -- simulation time

    if robot_pos(1) > model.robot_pos(1) - 2 * model.robot_sigma_x && ...
        robot_pos(2) < model.robot_pos(2) + 2 * model.robot_sigma_y
        pdf = mvnpdf(robot_pos, model.robot_pos, model.robot_pos_cov) / 0.978^2 ; 
    else 
        pdf = 0;
    end
    
end

