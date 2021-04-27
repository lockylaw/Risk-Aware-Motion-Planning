function colliProb = MonteCarlo_3D(sampleN, model)
%MONTECARLO Summary of this function goes here
%   Detailed explanation goes here

pd = mvnrnd([model.robot_pos(1), model.robot_pos(2), model.robot_pos(3)], ...
    model.robot_pos_cov, sampleN);

colliProb = 0;
if_Collision = zeros(sampleN, 1);

for i = 1: sampleN
    if model.obs_shape == "square"
        if_Collision(i) = ColliCheck_Square(pd(i, :)', model);
    elseif model.obs_shape == "ellip"
        if_Collision(i) = ColliCheck_Ellip(pd(i, :)', model);
    end
    
    if if_Collision(i) == 1
        colliProb = colliProb + 1;
    end
end

colliProb = colliProb / sampleN;

% scatter(fig.ax_main, rand_x, rand_y, '.');

end

