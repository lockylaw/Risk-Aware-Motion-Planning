clc
clear
close all

num_data = 1;

if num_data == 1
    load('simple_2d_nonGau.mat');
    label_data = "single obstacle in 2d workspace";
elseif num_data == 2
    load('multi_2d_nonGau.mat');
    label_data = "multiple obstacles in 2d workspace";
end



%%
fig.fig_main = figure;              % main figure
hold on;
ax_main = fig.fig_main.CurrentAxes;
fig.ax_main = ax_main;
box on;
grid on;
% axis equal;
xlabel('x (m)');
ylabel('y (m)');
set(ax_main, 'FontSize', 24);


lengthGroup = length(colliProb_MC);

lengthCount = 8;
resolution = 0.05;

colliProb_Group_MC = zeros(lengthGroup, 1);

colliProb_Count_MC = zeros(lengthCount, 1);

colliError_AIS = zeros(lengthCount, 1);
colliError_IS = zeros(lengthCount, 1);

colliVar_AIS = zeros(lengthCount, 1);
colliVar_IS = zeros(lengthCount, 1);

for i = 1:lengthGroup
    num_MC = ceil(colliProb_MC(i)/resolution);
    if num_MC > lengthCount
        num_MC = lengthCount;
    end

    colliProb_Group_MC(i) = num_MC;
    colliError_AIS(num_MC) = abs(colliProb_MC(i) - colliProb_AIS(i)) + ...
                                colliError_AIS(num_MC);
    colliVar_AIS(num_MC) = abs(colliProb_MC(i) - colliProb_AIS(i)) / ...
                            colliProb_MC(i) + colliVar_AIS(num_MC);
                        
    colliError_IS(num_MC) = abs(colliProb_MC(i) - colliProb_IS(i)) + ...
                            colliError_IS(num_MC);
    colliVar_IS(num_MC) = abs(colliProb_MC(i) - colliProb_IS(i)) / ...
                            colliProb_MC(i) + colliVar_IS(num_MC);
                      
    
    colliProb_Count_MC(num_MC) = colliProb_Count_MC(num_MC) + 1;
    
end

for j = 1:lengthCount
    colliError_AIS(j) = colliError_AIS(j) / colliProb_Count_MC(j);
    colliError_IS(j) = colliError_IS(j) / colliProb_Count_MC(j);
    colliVar_AIS(j) = colliVar_AIS(j) / colliProb_Count_MC(j);
    colliVar_IS(j) = colliVar_IS(j) / colliProb_Count_MC(j);
end

x = 1:lengthCount;
x_real = (0:lengthCount + 1) * 0.05;
colliError = [colliError_AIS, colliError_IS];
area_MC = bar(x, colliError);
% for k = 1:lengthCount
%     text(x(k)-0.15, colliError_AIS(k), num2str(colliError_AIS(k), '%.3f'), ...
%         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom')
%     text(x(k)+0.15, colliError_IS(k), num2str(colliError_IS(k), '%.3f'), ...
%         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom')
% end
axis([0, lengthCount + 1, 0, 0.06])
title('AIS and IS with '+label_data)
xlabel('Collision Probability of MC')
ylabel('Collision Probability Error')
legend('AIS','IS')
set(gca, 'xticklabel', x_real)

%%



