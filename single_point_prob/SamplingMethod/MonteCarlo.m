function colliProb = MonteCarlo(sampleN, model, varargin)
%MONTECARLO Summary of this function goes here
%   Detailed explanation goes here

if model.if_Gaussian == 1

    pd = mvnrnd([model.robot_pos(1), model.robot_pos(2)], ...
        diag([model.robot_sigma_x^2, model.robot_sigma_y^2]), sampleN);

    % generate random samples of robots
    rand_x = pd(:, 1);
    rand_y = pd(:, 2);
    
elseif model.if_Gaussian == 2
    
    pd = random(model.gm,sampleN);
    rand_x = pd(:, 1) + model.robot_pos(1);
    rand_y = pd(:, 2) + model.robot_pos(2);
    
else
    A = [-1 0; 0 1];
    B = [- model.robot_pos(1) + 3 * model.robot_sigma_x;
           model.robot_pos(2) + 3 * model.robot_sigma_y];
    pd = rmvnrnd(model.robot_pos, model.robot_pos_cov, sampleN, A, B);
    rand_x = pd(:, 1);
    rand_y = pd(:, 2);
%     [rand_x, rand_y] = TruncatedNormal(model, sampleN);
end


colliProb = 0;
if_Collision = zeros(sampleN, 1);

for i = 1: sampleN
    if model.obs_shape == "square"
        if_Collision(i) = ColliCheck_Square([rand_x(i); rand_y(i)], model);
    elseif model.obs_shape == "ellip"
        if_Collision(i) = ColliCheck_Ellip([rand_x(i); rand_y(i)], model);
    end
    
    if if_Collision(i) == 1
        colliProb = colliProb + 1;
    end
end

colliProb = colliProb / sampleN;

% scatter(rand_x, rand_y, '.');

end

