%% plot figure
function fig = plot_figure_3D(model)
%     Input:  model          -- the simulation model of robot and obstacles
%             plotoptions    -- the options of figures
%     Output: fig            -- main figure

    fig.fig_main = figure;              % main figure
    hold on;
    
    ax_main = fig.fig_main.CurrentAxes;
    fig.ax_main = ax_main;
    box on;
    grid on;
    axis equal;
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
    set(ax_main, 'FontSize', 24);
    
    for i = 1:model.obs_num
        fig.fig_obs_pos = plot3(ax_main, ...
                    model.obs_pos(1,i), model.obs_pos(2,i), model.obs_pos(3,i), ...
                    'Color', 'g', ...
                    'Marker', '*', ... 
                    'LineWidth', 2.0, 'LineStyle', '-');
        % plot obstacle shape
        if model.obs_shape == "ellip"
            fig.fig_ell_obs = plot_ellipse_3D(ax_main, model.obs_pos(:,i), ...
                        model.obs_size(:, i), 0, ...
                        'EdgeColor', 'k',...
                        'FaceAlpha', 0.4, 'EdgeAlpha', 0.6);
        elseif model.obs_shape == "square"
            fig.fig_ell_obs = plot_square_3D(ax_main, model.obs_pos(:,i), ...
                        model.obs_size(:, i), ...
                        'FaceColor', [1, 0, 0, 0.4], 'EdgeColor', [1, 0, 0, 0.6]);
            fig.fig_ell_obs = plot_square_3D(ax_main, model.obs_pos(:,i), ...
                        model.obs_size(:, i) + model.robot_size(1), ...
                        'FaceColor', 'none', 'EdgeColor', [1, 0, 0, 0.6]);
        end
    end
    

    % plot robot position mean
    fig.fig_robot_pos = plot3(ax_main, ...
                    model.robot_pos(1), model.robot_pos(2), model.robot_pos(3),...
                    'Color', 'g', ...
                    'Marker', '*', ... 
                    'LineWidth', 2.0, 'LineStyle', '-');
                
    % plot robot shape
    fig.fig_ell_robot = plot_ellipse_3D(ax_main, model.robot_pos, model.robot_size, 0, ...
                    'FaceAlpha', 0.4, ...
                    'EdgeColor', 'k', 'EdgeAlpha', 0.6);
    
    axis([-10 10 -10 10 -10 10]);
end