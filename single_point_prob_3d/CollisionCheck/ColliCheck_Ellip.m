function if_Collision = ColliCheck_Ellip(robot_pos_current, model, varargin)
%     Input:  robot_state      -- the position states of robots (in cell)
%             obs_state        -- the position states of obstacles (in cell)
%             thres            -- the collision threshold (closed center distance)
%     Output: if_Collision     -- if collision happens, return 1; else, return 0

% the number of robots

if_Collision = 0;
for i = 1:model.obs_num
    d_vec_obs = robot_pos_current - model.obs_pos(:, i);
    coll_size = model.robot_size + model.obs_size(:, i);
    if d_vec_obs(1)^2/coll_size(1)^2 + d_vec_obs(2)^2/coll_size(2)^2 + ...
            d_vec_obs(3)^2/coll_size(3)^2 - 1 <= 0
        if_Collision = 1;
    end
end

if model.obs_num > 1 && nargin > 2 && varargin{1} == "obs_check"
    for i = 1:model.obs_num
        for j = 1:model.obs_num
            if i ~= j
                d_vec_obs = model.obs_pos(:, i) - model.obs_pos(:, j);
                coll_size = model.obs_size(:, i) + model.obs_size(:, j);
                if d_vec_obs(1)^2/coll_size(1)^2 + d_vec_obs(2)^2/coll_size(2)^2 - 1 <= 0
                    if_Collision = 1;
                end
            end
        end
    end
end

end

