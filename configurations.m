clear all
close all
clc 

%% configurations
% setup
pos = [-2; 1; 0];                   % robot position mean
sigma_x = 0.6;
sigma_y = 0.4;
sigma_theta = 0.01;
rho_xtheta= 0.0;
rho_ytheta= 0.0;
rho_xy= 0.0;

pos_cov = [sigma_x^2, rho_xy*sigma_x*sigma_y, rho_xtheta*sigma_x*sigma_theta; 
           rho_xy*sigma_x*sigma_y, sigma_y^2, rho_ytheta*sigma_theta*sigma_y;
           rho_xtheta*sigma_x*sigma_theta, rho_ytheta*sigma_theta*sigma_y, sigma_theta^2];
       
obs_pos = [0; 0; 0];                   % robot position mean
obs_sigma_x = 0.4;
obs_sigma_y = 0.4;
obs_sigma_theta = 0.01;
obs_rho_xtheta= 0.0;
obs_rho_ytheta= 0.0;
obs_rho_xy= 0.0;

obs_pos_cov = [obs_sigma_x^2, obs_rho_xy*obs_sigma_x*obs_sigma_y, obs_rho_xtheta*obs_sigma_x*obs_sigma_theta; 
               obs_rho_xy*obs_sigma_x*obs_sigma_y, obs_sigma_y^2, obs_rho_ytheta*obs_sigma_theta*obs_sigma_y;
               obs_rho_xtheta*obs_sigma_x*obs_sigma_theta, obs_rho_ytheta*obs_sigma_theta*obs_sigma_y, obs_sigma_theta^2];


%% plot figure
fig_main = figure;              % main figure
hold on;
axis([-5 3 -3 4]);
ax_main = fig_main.CurrentAxes;
box on;
grid on;
axis equal;
xlabel('x (m)');
ylabel('y (m)');
set(ax_main, 'FontSize', 24);
% plot obstacle
fig_obs_pos = plot(ax_main, obs_pos(1), obs_pos(2), ...
                'Color', 'g', ...
                'Marker', '*', ... 
                'LineWidth', 2.0, 'LineStyle', '-');
% plot robot position covariance
[pos_cov_xData, pos_cov_yData] = getErrorEllipsePoint2D(obs_pos_cov, obs_pos, 1, 0);
fig_obs_pos_cov_1 = plot(ax_main, pos_cov_xData, pos_cov_yData, ...
                'Color', 'k', ...
                'LineWidth', 1.5, 'LineStyle', '--');
[pos_cov_xData, pos_cov_yData] = getErrorEllipsePoint2D(obs_pos_cov, obs_pos, 2, 0);
fig_obs_pos_cov_2 = plot(ax_main, pos_cov_xData, pos_cov_yData, ...
                'Color', 'b', ...
                'LineWidth', 1.5, 'LineStyle', '--');
[pos_cov_xData, pos_cov_yData] = getErrorEllipsePoint2D(obs_pos_cov, obs_pos, 3, 0);
fig_obs_pos_cov_3 = plot(ax_main, pos_cov_xData, pos_cov_yData, ...
                'Color', 'm', ...
                'LineWidth', 1.5, 'LineStyle', '--');
print(fig_main, 'scenario.pdf', '-dpdf', ...
            '-r300', '-bestfit');



% plot robot position mean
fig_robot_pos = plot(ax_main, pos(1), pos(2), ...
                'Color', 'g', ...
                'Marker', '*', ... 
                'LineWidth', 2.0, 'LineStyle', '-');
% plot robot position covariance
[pos_cov_xData, pos_cov_yData] = getErrorEllipsePoint2D(pos_cov, pos, 1, 0);
fig_robot_pos_cov_1 = plot(ax_main, pos_cov_xData, pos_cov_yData, ...
                'Color', 'k', ...
                'LineWidth', 1.5, 'LineStyle', '--');
[pos_cov_xData, pos_cov_yData] = getErrorEllipsePoint2D(pos_cov, pos, 2, 0);
fig_robot_pos_cov_2 = plot(ax_main, pos_cov_xData, pos_cov_yData, ...
                'Color', 'b', ...
                'LineWidth', 1.5, 'LineStyle', '--');
[pos_cov_xData, pos_cov_yData] = getErrorEllipsePoint2D(pos_cov, pos, 3, 0);
fig_robot_pos_cov_3 = plot(ax_main, pos_cov_xData, pos_cov_yData, ...
                'Color', 'm', ...
                'LineWidth', 1.5, 'LineStyle', '--');
print(fig_main, 'scenario.pdf', '-dpdf', ...
            '-r300', '-bestfit');


        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
%% get error ellipse points for plotting
function [xData, yData] = getErrorEllipsePoint2D(covP, mu, r, r_coll)
    % Input:  mean vector (2x1), covariance matrix (2x2) and Mahalanobis distance
    % Output: two-dimensional surface data


    n=100; % Number of points around ellipse
    p=0:pi/n:2*pi; % angles around a circle

    % 1-sigma ellipse
    [eigvec,eigval] = eig(covP(1:2,1:2));                        % Compute eigen-stuff
    xy = [cos(p'),sin(p')] * sqrt(eigval) * eigvec';    % Transformation

    % enlarge with Mahalanobis distance
    xy(:,1) = r*xy(:,1);
    xy(:,2) = r*xy(:,2);

    % adding extra robot radius
    for i = 1 : length(xy(:,1))
        d = norm(xy(i,:));
        xy(i,:) = xy(i,:)*((d+r_coll)/d);
    end

    % moving with mean vector
    xData = xy(:,1) + mu(1);
    yData = xy(:,2) + mu(2);

end