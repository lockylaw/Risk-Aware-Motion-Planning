function [feasible_region_fastAss, X_fastAss, Y_fastAss] = probability_risk_FastAss(ProbThre, model_type)

%% configurations
% setup

% model_type define the distribution type
% ---  1  Gaussian 
% ---  2  GMM

model = modelGenerator(model_type);

coll_size = model.robot_size + model.obs_size;
omega = diag([1/coll_size(1)^2, 1/coll_size(2)^2]);
omega_sqrt = sqrt(omega);


%% fast assess

pos_x_vec_fastAss = model.max_size(1):0.01:model.max_size(2);
pos_y_vec_fastAss = model.max_size(3):0.01:model.max_size(4);
[X_fastAss, Y_fastAss] = meshgrid(pos_x_vec_fastAss, pos_y_vec_fastAss);

feasible_region_fastAss = zeros(length(pos_x_vec_fastAss), length(pos_y_vec_fastAss));
% MC sampling
for i = 1 : length(pos_x_vec_fastAss)
    for j = 1 : length(pos_y_vec_fastAss)
        pos = [pos_x_vec_fastAss(i); pos_y_vec_fastAss(j)];
        % feasible region
        p_i = model.alpha * pos;
        p_j = model.obs_pos;
        
        mu_x = p_i(1) - p_j(1);
        mu_y = p_i(2) - p_j(2);
        sigma_x = model.robot_sigma_x;
        sigma_y = model.robot_sigma_y;
        cov_xy = model.robot_pos_cov(1,2);
        
        R_a = 1 / coll_size(1)^2;
        R_b = 1 / coll_size(2)^2;
        
        mu_Q_1 = R_a * (sigma_x^2 + mu_x^2) + R_b * (sigma_y^2 + mu_y^2) - 1;
        sigma_Q_1_sqrt = R_a^2 * (2*sigma_x^4 + 4*sigma_x^2*mu_x^2) + ...
                         R_b^2 * (2*sigma_y^4 + 4*sigma_y^2*mu_y^2) + ...
                         2 * R_a * R_b * cov_xy^2;
                
        Conc_Q_1 = 2/9 * sigma_Q_1_sqrt / mu_Q_1^2;
        Conc_Q_1_star = - mu_Q_1;
        
        if Conc_Q_1_star > 0 || Conc_Q_1 > ProbThre
            feasible_region_fastAss(i, j) = 1;
        end
    end
end
