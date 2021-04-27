function model = modelGenerator(model_type)
%MODELGENERATOR Summary of this function goes here
%   Detailed explanation goes here

if model_type == 1

    model.robot_pos = [-2; 1];                      % robot position mean
    model.robot_sigma_x = 0.2;                      % robot variance of x
    model.robot_sigma_y = 0.2;                      % robot variance of y
    model.robot_sigma_theta = 0.01;                 % robot variance of theta
    model.robot_rho_xtheta= 0.0;                    % robot covariance of x and theta
    model.robot_rho_ytheta= 0.0;                    % robot covariance of y and theta
    model.robot_rho_xy= -0.0;    

    % covariance matrix/Sigma
    model.robot_pos_cov = ...
        [model.robot_sigma_x^2, model.robot_rho_xy*model.robot_sigma_x*model.robot_sigma_y; 
        model.robot_rho_xy*model.robot_sigma_x*model.robot_sigma_y, model.robot_sigma_y^2;];

elseif model_type == 2

    model.robot_pos = [-2; 1];                      % robot position mean
    model.pos_x = [-0.25; 0.5] / 2;                 % x values of GMM 
    model.pos_y = [0.2; -0.4] / 2;                  % y values of GMM 
    model.Mu = [model.pos_x, model.pos_y];
    model.sigma_x_1 = 0.3 / 2;                      % robot variance of x
    model.sigma_y_1 = 0.2 / 2;                      % robot variance of y
    model.rho_xy_1= 0.0 / 2; 
    model.sigma_x_2 = 0.4 / 2;                      % robot variance of x
    model.sigma_y_2 = 0.2 / 2;                      % robot variance of y
    model.rho_xy_2= 0.0 / 2; 

    % covariance matrix/Sigma
    model.pos_cov_1 = [model.sigma_x_1^2, model.rho_xy_1*model.sigma_x_1*model.sigma_y_1; 
                 model.rho_xy_1*model.sigma_x_1*model.sigma_y_1, model.sigma_y_1^2;];
    model.pos_cov_2 = [model.sigma_x_2^2, model.rho_xy_2*model.sigma_x_2*model.sigma_y_2; 
                 model.rho_xy_2*model.sigma_x_2*model.sigma_y_2, model.sigma_y_2^2;];
   
    model.Sigma = cat(3,model.pos_cov_1,model.pos_cov_2);
    model.P = [2/3; 1/3];
    
    model.gm = gmdistribution(model.Mu,model.Sigma,model.P);
%     model.pos_mean = [pos_x' * P; pos_y' * P];
%     model.pos_cov_mean = zeros(size(pos_cov_1));
%     for i = 1:length(P)
%         model.pos_cov_mean = model.pos_cov_mean + P(i) * model.gm.Sigma(:, :, i) + ...
%             P(i)*(model.gm.mu(:, i) - model.pos_mean) * (model.gm.mu(:, i) - model.pos_mean)';
%     end
%     model.pos_sigma = sqrt(diag(model.pos_cov_mean));
    pd = random(model.gm,1e7);
    rand_x = pd(:, 1);
    rand_y = pd(:, 2);
    model.robot_pos_mean = [mean(rand_x); mean(rand_y)];
    model.robot_pos_cov = cov(rand_x, rand_y);
    model.robot_sigma_x = sqrt(model.robot_pos_cov(1,1));
    model.robot_sigma_y = sqrt(model.robot_pos_cov(2,2));
end
             
             
% robot size
model.robot_size = [0.5; 0.5];
model.robot_ell_size = model.robot_size * 0.5 * sqrt(2);
model.obs_num = 1;
model.obs_shape = "ellip";
if model.obs_num == 1
    model.obs_size = [2; 1];
    model.obs_pos = [0; 0];
    model.max_size = [-3.5 3.5 -3.5 3.5];
    model.orient = -pi/4;
elseif model.obs_num == 2
    model.obs_size = [[2; 1], [1; 2]];
    model.obs_pos = [-2 1; -1 1];
    model.max_size = [-5.5 3.5 -3.5 4.5];
    model.orient = [-pi/4, pi/6];
end

for i = 1:model.obs_num
    model.alpha(:, :, i) = [ cos(model.orient(i)) -sin(model.orient(i))
                             sin(model.orient(i))  cos(model.orient(i))];
end

model.obs_ell_size = model.obs_size * 0.5 * sqrt(2);

% whether the uncertainties of obstacles are considered
model.if_obsEllip = 1;
model.if_Gaussian = model_type;

model.max_size = [-4, 4, -4, 4];
end

