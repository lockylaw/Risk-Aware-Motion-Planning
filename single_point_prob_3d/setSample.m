clear all
close all
clc 



%% Monte Carlo: compute probability with different sampling numbers
%  Working Environment: 1 robot, 1/2 static osbtacles with/without
%  uncertainty

% robot informations
model.robot_pos = [-2; 3];                      % robot position mean
model.robot_sigma_x = 0.6;                      % robot variance of x
model.robot_sigma_y = 0.8;                      % robot variance of y
model.robot_sigma_theta = 0.01;                 % robot variance of theta
model.robot_rho_xtheta= 0.0;                    % robot covariance of x and theta
model.robot_rho_ytheta= 0.0;                    % robot covariance of y and theta
model.robot_rho_xy= 0.0;                        % robot covariance of x and y

% covariance matrix/Sigma
model.robot_pos_cov = [model.robot_sigma_x^2, model.robot_rho_xy*model.robot_sigma_x*model.robot_sigma_y; 
                 model.robot_rho_xy*model.robot_sigma_x*model.robot_sigma_y, model.robot_sigma_y^2;];

% robot size
model.robot_size = [1; 1];
model.robot_ell_size = model.robot_size/2;%0.5*sqrt(2)*model.robot_box_size;

% obstacle number

model.obs_pos = [[0; 0],[-5; 1], [-4; 5]];                   % obstacle position mean
model.obs_num = size(model.obs_pos, 2);
model.obs_shape = "square";

% osbtacle size
model.obs_size = [[2; 2],[3; 1.2], [3; 1.2]];    

% whether the uncertainties of obstacles are considered
model.if_obsEllip = 1;



% Probability Type
% -- 1 -- Gaussian 
% -- 2 -- Gaussian Mixture Model
% -- 3 -- Truncated Gaussian


%% plot figure of distribution of robot and obstacles

fig = plot_figure(model);

%%
ProbType = 1;
tic
sampleN = 1e6;
if ProbType == 1
    [rand_x, rand_y, pdf] = GaussianDist(model, sampleN, fig);
elseif ProbType == 2
    [rand_x, rand_y, pdf] = GaussianCore(model, sampleN, fig);
elseif ProbType == 3
    [rand_x, rand_y, pdf] = TruncatedNormal(model, sampleN, fig);
end

sampleN_MC = sampleN;
colliProb_MC = MonteCarlo(sampleN_MC, model);

time_MC = toc;

%%
tic
sampleN_QBS = 80;
colliProb_QBS = QuadratureBasedSampling(model, sampleN_QBS);
time_QBS = toc;

%%
tic
OptionsAIS.sampleN = 1e3;
OptionsAIS.iteration_l = 10;
colliProb_AIS = AdaptiveImportanceSampling(model, OptionsAIS);
time_AIS = toc;






