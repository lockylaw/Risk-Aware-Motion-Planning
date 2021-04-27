% get collision probability
function [probabilityEstimateRe, probabilityEstimateReVar] = QuadratureBasedSampling(model, n)
%     Input:  model -- the simulation model of robot and obstacles
%             n     -- the number of iterations
%     Output: probabilityEstimateRe -- collision probability
%             time                  -- simulation time
    
    tic;
    
    syms x;
    
    % Hermite Polynominal with n dimension
    HermiteFun_n = hermiteH(n, x);
    HermiteFun_n = matlabFunction(HermiteFun_n);
    % Hermite Polynominal with n-1 dimension
    HermiteFun_n_1 = hermiteH(n-1, x);
    HermiteFun_n_1 = matlabFunction(HermiteFun_n_1);
    % the roots of Hermite Polynominal 
    y_i = vpa(solve(HermiteFun_n));
    y_i = double(y_i);
    H_n_1 = HermiteFun_n_1(y_i);
    H_n_1 = double(H_n_1);
    
    % the weights by different roots
    weight = 2^(n-1)*prod(1:n)*sqrt(pi)/n^2./H_n_1.^2;
    
    % corresponding robot postions
    robot_x_i = sqrt(2)*y_i*model.robot_sigma_x + model.robot_pos(1);
    robot_y_i = sqrt(2)*y_i*model.robot_sigma_y + model.robot_pos(2);
    
    % collision probability
    probabilityEstimate = 0;
    
    % input of collison check
    robot_states_Hermite = cell(n,n);
    
    % formations of inputs
    for i = 1:n
        for j = 1:n
            robot_state.x = robot_x_i(i);
            robot_state.y = robot_y_i(j);
            robot_states_Hermite{i,j} = robot_state;
        end
    end
   

    for i_loop = 1:n
        for j_loop = 1:n
            weight_loop = weight(i_loop)*weight(j_loop);
            robot_state_tmp = robot_states_Hermite(i_loop,j_loop);
            CollisionCheck_i = ColliCheck([robot_state_tmp{1}.x, ...
                robot_state_tmp{1}.y], model);
            probabilityEstimate = probabilityEstimate + weight_loop*CollisionCheck_i;
        end
    end   

    probabilityEstimateRe = pi^(-2/2)*real(probabilityEstimate);
    probabilityEstimateReVar = NaN;

    
    % the time of simulations
    time = toc;
    
end