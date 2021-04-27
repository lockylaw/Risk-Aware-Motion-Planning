function [colliProb, colliVar] = MonteCarlo(rand_x, rand_y, pdf, sampleN, model)
%MONTECARLO Summary of this function goes here
%   Detailed explanation goes here

colliProb = 0;
if_Collision = zeros(sampleN, 1);

for i = 1: sampleN
    if_Collision(i) = ColliCheck([rand_x(i), rand_y(i)], model);
    if if_Collision(i) == 1
        colliProb = colliProb + 1;
    end
end

colliProb = colliProb / sampleN;
colliVar = sum( (if_Collision - pdf ) .^ 2 ) / sampleN ^ 2;

end

