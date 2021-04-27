% get collision probability
function [probabilityEstimateRe, time] = quadratureSampling(model, n)
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
    % corresponding obstacles postions
    obs_x_i = sqrt(2)*y_i*model.obs_sigma_x + ones(length(y_i),1)*model.obs_pos(1,:);
    obs_y_i = sqrt(2)*y_i*model.obs_sigma_y + ones(length(y_i),1)*model.obs_pos(2,:);
    
    % collision probability
    probabilityEstimate = 0;
    
    % input of collison check
    robot_states_Hermite = cell(n,n);
    obs_states_Hermite = cell(model.obs_num, n, n);
    
    % formations of inputs
    for i = 1:n
        for j = 1:n
            robot_state.x = robot_x_i(i);
            robot_state.y = robot_y_i(j);
            robot_states_Hermite{i,j} = robot_state;
            for k = 1:model.obs_num
                obs_state.x = obs_x_i(i);
                obs_state.y = obs_y_i(j);
                obs_states_Hermite{k, i,j} = obs_state;
            end
        end
    end
    
    % if the uncertainties of obstacles are considered
    if model.if_obsEllip == 1
        % single obstacle
        if model.obs_num == 1
            for i_loop = 1:n
                fprintf("i_loop = %d \n", i_loop);
                for j_loop = 1:n
                    for k_loop = 1:n
                        for l_loop = 1:n
                                    weight_loop = weight(i_loop)*weight(j_loop)*weight(k_loop)*weight(l_loop);
                                    CollisionCheck_i = CollisionCheck({robot_states_Hermite{i_loop,j_loop}},...
                                        {obs_states_Hermite{1, k_loop, l_loop}}, 1);
                                    probabilityEstimate = probabilityEstimate + weight_loop*CollisionCheck_i;
                        end
                    end
                end
            end   
            
            probabilityEstimateRe = pi^(-4/2)*probabilityEstimate;
        % two obstacles
        else
            for i_loop = 1:n
                fprintf("i_loop = %d \n", i_loop);
                for j_loop = 1:n
                    for k_loop = 1:n
                        for l_loop = 1:n
                            for m_loop = 1:n
                                for n_loop = 1:n
                                    weight_loop = weight(i_loop)*weight(j_loop)*weight(k_loop)*...
                                        weight(l_loop)*weight(m_loop)*weight(n_loop);
                                    CollisionCheck_i = CollisionCheck({robot_states_Hermite{i_loop,j_loop}},...
                                        {obs_states_Hermite{1, k_loop, l_loop}, obs_states_Hermite{2, m_loop, n_loop}}, 1);
                                    probabilityEstimate = probabilityEstimate + weight_loop*CollisionCheck_i;
                                end
                            end
                        end
                    end
                end
            end   

            probabilityEstimateRe = pi^(-6/2)*probabilityEstimate;
        end   
    % if the uncertainties of obstacles are not considered  
    else
        obs_states = cell(model.obs_num, 1);
        for i = 1:model.obs_num
            obs_state.x = model.obs_pos(1 , i);
            obs_state.y = model.obs_pos(2 , i);
            obs_states{i} = obs_state;
        end
            
        for i_loop = 1:n
            for j_loop = 1:n
                weight_loop = weight(i_loop)*weight(j_loop);
                CollisionCheck_i = CollisionCheck({robot_states_Hermite{i_loop,j_loop}},...
                    obs_states, 1);
                probabilityEstimate = probabilityEstimate + weight_loop*CollisionCheck_i;
            end
        end   

        probabilityEstimateRe = pi^(-2/2)*probabilityEstimate;
        
    end
    
    % the time of simulations
    time = toc;
    
end