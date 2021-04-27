function [feasible_region_linear, X_linear, Y_linear] = probability_risk_Linear(ProbThre, model_type)

model = modelGenerator(model_type);

coll_size = model.robot_size + model.obs_size;
omega = diag([1/coll_size(1)^2, 1/coll_size(2)^2]);
omega_sqrt = sqrt(omega);


%% linearize

pos_x_vec_linear = model.max_size(1):0.01:model.max_size(2);
pos_y_vec_linear = model.max_size(3):0.01:model.max_size(4);
[X_linear, Y_linear] = meshgrid(pos_x_vec_linear, pos_y_vec_linear);

feasible_region_linear = zeros(length(pos_x_vec_linear), length(pos_y_vec_linear));

% MC sampling
for i = 1 : length(pos_x_vec_linear)
    for j = 1 : length(pos_y_vec_linear)
        pos = [pos_x_vec_linear(i); pos_y_vec_linear(j)];
        % feasible region
        p_i = model.alpha * pos;
        p_j = model.obs_pos;
        p_i_bar = omega_sqrt*p_i;
        p_j_bar = omega_sqrt*p_j;
        a_io = (p_i_bar - p_j_bar) / norm(p_i_bar - p_j_bar);
        b_io = 1;
        indicator_ij = a_io'*omega_sqrt*(p_i-p_j) - b_io ...
            - erfinv(1-2*ProbThre)*sqrt(2*a_io'*omega_sqrt*model.robot_pos_cov*omega_sqrt'*a_io);
        if indicator_ij >= 0
            feasible_region_linear(i, j) = 1;
        end
    end
end

