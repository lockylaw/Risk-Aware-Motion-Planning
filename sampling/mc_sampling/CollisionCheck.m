function if_Collision = CollisionCheck(robot_pos_current, robot_size, obs_pos, obs_size)
%     Input:  robot_state      -- the position states of robots (in cell)
%             obs_state        -- the position states of obstacles (in cell)
%             thres            -- the collision threshold (closed center distance)
%     Output: if_Collision     -- if collision happens, return 1; else, return 0

% the number of robots

if_Collision = 0;
d_vec_obs = robot_pos_current - obs_pos;
coll_size = robot_size + obs_size;
if sqrt(d_vec_obs(1)^2/coll_size(1)^2 + d_vec_obs(2)^2/coll_size(2)^2 - 1) <= 0
    if_Collision = 1;
%     warning('Collision happens!');
%     fprintf('\ndistance is %d', d_vec_obs(1)^2/coll_size(1)^2 + d_vec_obs(2)^2/coll_size(2)^2);
end

end

