function [X_map, x, y] = probability_risk_SP(model)

%% configurations
% setup

% model_type define the distribution type
% ---  1  Gaussian 
% ---  2  GMM
coll_size = model.robot_size + model.obs_size;
omega = diag([1/coll_size(1)^2, 1/coll_size(2)^2]);
omega_sqrt = sqrt(omega);


%% senario programming
beta = 1e-6;
s_hut = 20;
sigma = 0.03;

S_k = SenarioProgSampNum(beta, s_hut, sigma);

if model.if_Gaussian == 1

    pd = mvnrnd([model.robot_pos(1), model.robot_pos(2)], ...
        diag([model.robot_sigma_x^2, model.robot_sigma_y^2]), S_k);
    pd = pd';
    pd_trans = omega_sqrt * model.alpha * pd;
    obs_pos_trans = omega_sqrt * model.alpha * model.obs_pos;
    norm_trans = zeros(1, S_k);
    A_k = zeros(2, S_k);
    b_k = zeros(S_k, 1);
    for i = 1:S_k
        norm_trans(i) = norm(pd_trans(:, i) - obs_pos_trans);
        A_k(:, i) = (pd_trans(:, i) - obs_pos_trans)/norm_trans(i);
        b_k(i) = A_k(:, i)'* (pd_trans(:, i) - A_k(:, i));
    end
    
elseif model.if_Gaussian == 2
    
%     pd = random(model.gm, S_k) + repmat(model.robot_pos', S_k, 1);
    pd = repmat(model.robot_pos', S_k, 1);
    pd_trans = omega_sqrt * model.alpha * pd';
    obs_pos_trans = omega_sqrt * model.alpha * model.obs_pos;
    norm_trans = zeros(1, S_k);
    A_k = zeros(2, S_k);
    b_k = zeros(S_k, 1);
    for i = 1:S_k
        norm_trans(i) = norm(pd_trans(:, i) - obs_pos_trans);
        A_k(:, i) = (pd_trans(:, i) - obs_pos_trans)/norm_trans(i);
        b_k(i) = A_k(:, i)'* (pd_trans(:, i) - A_k(:, i));
    end
    
end

x = -4:0.02:4;
y = -4:0.02:4;
X_map = zeros(length(x), length(y));
for i = 1:length(x)
    for j = 1:length(y)
        X_map(i, j) = any(A_k' * omega_sqrt * model.alpha * [x(i); y(j)] <= b_k);
        if X_map(i,j) == 1
            X_map(i, j) = NaN;
        end
    end
end
        

end
