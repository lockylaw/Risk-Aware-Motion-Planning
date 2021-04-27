%% plot figure
function fig = plot_figure(model)
%     Input:  model          -- the simulation model of robot and obstacles
%             plotoptions    -- the options of figures
%     Output: fig            -- main figure

    fig.fig_main = figure;              % main figure
    hold on;
    axis([-5 3 -3 8]);
    ax_main = fig.fig_main.CurrentAxes;
    fig.ax_main = ax_main;
    box on;
    grid on;
    axis equal;
    xlabel('x (m)');
    ylabel('y (m)');
    set(ax_main, 'FontSize', 24);
    
    for i = 1:model.obs_num
        fig.fig_obs_pos = plot(ax_main, model.obs_pos(1,i), model.obs_pos(2,i), ...
                    'Color', 'g', ...
                    'Marker', '*', ... 
                    'LineWidth', 2.0, 'LineStyle', '-');
        % plot obstacle shape
        fig.fig_ell_obs = plot_square_2D(ax_main, model.obs_pos(:,i), model.obs_box_size(:, i), ...
                    'FaceColor', [1, 0, 0, 0.4], 'EdgeColor', [1, 0, 0, 0.6]);
        fig.fig_ell_obs = plot_square_2D(ax_main, model.obs_pos(:,i), model.obs_box_size(:, i)+2, ...
                    'FaceColor', 'none', 'EdgeColor', [0, 1, 0, 0.6]);
    end
    

    % plot robot position mean
    fig.fig_robot_pos = plot(ax_main, model.robot_pos(1), model.robot_pos(2), ...
                    'Color', 'g', ...
                    'Marker', '*', ... 
                    'LineWidth', 2.0, 'LineStyle', '-');
                
    % plot robot shape
    fig.fig_ell_robot = plot_ellipse_2D(ax_main, model.robot_pos, model.robot_ell_size, 0, ...
                    'FaceColor', 'r', 'FaceAlpha', 0.4, ...
                    'EdgeColor', 'r', 'EdgeAlpha', 0.6);

end