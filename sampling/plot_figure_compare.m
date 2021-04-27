clc
clear
close all
load('ColliPro_MC_IS.mat');
load('ColliPro_AIS.mat');

%%
fig_main_origin = figure(1);              % main figure
hold on;
box on; 
grid on;
xlabel('Horizon');
ylabel('CP')
ax_main_origin = fig_main_origin.CurrentAxes;
set(ax_main_origin, 'FontSize', 24);

ColliVar_Group_Area = [ColliProb_Group - ColliVar_Group * 4e3;
    2 * ColliVar_Group * 4e3];
ColliVar_Group_Area = ColliVar_Group_Area';
area_MC = area(1:20, ColliVar_Group_Area, 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.4);
area_MC(1).FaceAlpha = 0;

ColliVar_IS_Group_Area = [ColliProb_IS_Group - ColliVar_IS_Group * 4e3;
    2 * ColliVar_IS_Group * 4e3];
ColliVar_IS_Group_Area = ColliVar_IS_Group_Area';
area_IS = area(1:20, ColliVar_IS_Group_Area, 'FaceColor', 'r', 'EdgeColor', 'none', 'FaceAlpha', 0.4);
area_IS(1).FaceAlpha = 0;

ColliVar_AIS_Group_Area = [ColliProb_AIS_Group - ColliVar_AIS_Group * 1e5;
    2 * ColliVar_AIS_Group * 1e5];
ColliVar_AIS_Group_Area = ColliVar_AIS_Group_Area';
area_AIS = area(1:20, ColliVar_AIS_Group_Area, 'FaceColor', 'k', 'EdgeColor', 'none', 'FaceAlpha', 0.4);
area_AIS(1).FaceAlpha = 0;

pro_MC = plot(1:20, ColliProb_Group, 'b', 'LineWidth', 1);
pro_IS = plot(1:20, ColliProb_IS_Group, 'r', 'LineWidth', 1);
pro_AIS = plot(1:20, ColliProb_AIS_Group, 'k', 'LineWidth', 1);

h = legend([pro_MC, pro_IS, pro_AIS], 'MC', 'IS', 'AIS');
set(h, 'FontSize', 32)

