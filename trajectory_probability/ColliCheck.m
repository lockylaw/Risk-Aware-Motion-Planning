function if_Collision = ColliCheck(robot_pos_current, model)
%     Input:  robot_state      -- the position states of robots (in cell)
%             obs_state        -- the position states of obstacles (in cell)
%             thres            -- the collision threshold (closed center distance)
%     Output: if_Collision     -- if collision happens, return 1; else, return 0

% the number of robots

if_Collision = 0;

robot_size = model.robot_size(1);
obs_pos = model.obs_pos;
obs_size = model.obs_size;
obs_num = model.obs_num;

for i = 1:obs_num
    obs_range = [obs_pos(1, i) - obs_size(1, i)/2 - robot_size, ...
                    obs_pos(1, i) + obs_size(1, i)/2 + robot_size;
                 obs_pos(2, i) - obs_size(2, i)/2 - robot_size, ...
                    obs_pos(2, i) + obs_size(2, i)/2 + robot_size;];
    if  (robot_pos_current(1) - obs_range(1, 1))*(robot_pos_current(1) - obs_range(1, 2)) < 0 && ...
        (robot_pos_current(2) - obs_range(2, 1))*(robot_pos_current(2) - obs_range(2, 2)) < 0
        if_Collision = 1;
    end
                                
end

end

