function if_Collision = CollisionCheck(robot_state, obs_state, thres)
%     Input:  robot_state      -- the position states of robots (in cell)
%             obs_state        -- the position states of obstacles (in cell)
%             thres            -- the collision threshold (closed center distance)
%     Output: if_Collision     -- if collision happens, return 1; else, return 0

% the number of robots
num_robot = size(robot_state,1);
% the number of obstacles
num_obs = size(obs_state,1);
if_Collision = 0;

for i = 1:num_robot
    for j = 1:num_obs
        % distance between robot and obstacles center
        dis = sqrt((robot_state{i}.x - obs_state{j}.x)^2 + ...
            (robot_state{i}.y - obs_state{j}.y)^2);
        % check if collsion happens
        if dis <= thres
            if_Collision = 1;
        end
    end
end

end

