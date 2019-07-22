clear all; clc; close all;
%%
%workspace limits
xrange = [-0.5, 0.5];
yrange = [-0.5, 0.5];
zrange = [-0.5, 0.5];

global num_modes cm ng nd xg xr sig delta_t conf_thresh conf_max alpha_max;

%workspace params.
% ng = 3; %num of goals
max_ng = 6;
% ng = 4; %num of goals
ng = datasample(2:max_ng, 1); %spawn random number of goals. Maximum number is 6. At least 
nd = 3; %num of dimensions. by definition R^3
cm_options = {{1,2,3}, {[1,2], 3}};
max_nm = length(cm_options);
% cm = cm_options{datasample(1:max_nm, 1)};
% cm = {1,2,3}; %{1,2,3}, %{[1,3], 2} %{[2,3], 1};
cm = cm_options{1};
num_modes = length(cm); %

%spawn random goals and robot position. 
xg = [rand(1,ng)*range(xrange) + xrange(1); rand(1,ng)*range(yrange) + yrange(1); rand(1,ng)*range(zrange) + zrange(1)]; %random goal positions. These will be treated as fixed parameters.
xr = [rand(1,1)*range(xrange) + xrange(1); rand(1,1)*range(yrange) + yrange(1); rand(1,1)*range(zrange) + zrange(1)];
xr_true = xr;

%disamb parameters.
sig = 0.01; %For Fisher information
delta_t = 0.1; %For compute projections. 

%arbitration function parameters
conf_thresh = (1.2/ng);
conf_max = (1.4/ng);
alpha_max = 0.7;

%%
%% plot the goals and robot
figure;
scatter3(xg(1,1:ng), xg(2,1:ng), xg(3,1:ng), 230, 'k', 'filled'); grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
scatter3(xr(1), xr(2), xr(3), 230, 'r', 'filled');
offset = [-0.3, 0.3];
line(xrange+offset, [0,0], [0,0], 'Color', 'r', 'LineWidth', 1.5); %draw x and y axes.
line([0,0], yrange+offset, [0,0], 'Color', 'g','LineWidth', 1.5);
line([0,0], [0,0], zrange+offset, 'Color', 'b','LineWidth', 1.5);
axis([xrange+offset, yrange+offset, zrange+offset]);
axis square;
view([142,31]);
%%
%%
intent_type = 'dft';
%%
% Show best disambiguating modes for the entire workspace. 
min_ws = -0.5;
max_ws = 0.5;
num_steps = 12;
step_size = (max_ws - min_ws)/num_steps;
ax_p = (min_ws:step_size:max_ws)';
[X,Y,Z] = meshgrid(ax_p);
ws_points = [X(:) Y(:) Z(:)];
disamb_modes_ENT = zeros(size(ws_points, 1), 1);
disamb_modes_KL = zeros(size(ws_points, 1), 1);
pg0 = (1/ng)*ones(ng,1); %at every workspace point the starting probability distribution is uniform. 
% pg0 = rand(ng, 1); pg0 = pg0/sum(pg0);
current_optimal_mode_ENT = cm{datasample(1:num_modes, 1)};
current_optimal_mode_KL = cm{datasample(1:num_modes, 1)};

for i=1:size(ws_points,1)
    xr = ws_points(i, :)';
    current_optimal_mode_ENT_index = compute_optimal_mode_ENT_R3(intent_type, xr, pg0); 
    if length(current_optimal_mode_ENT_index) > 1 %when there are equivalent modes. 
        current_optimal_mode_ENT = cm{current_optimal_mode_ENT_index(1)}; %pick the first one. 
        current_optimal_mode_ENT_index = current_optimal_mode_ENT_index(1); %update the index. 
    else
        if current_optimal_mode_ENT_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
            current_optimal_mode_ENT = cm{current_optimal_mode_ENT_index}; 
        end
    end
    disamb_modes_ENT(i) = current_optimal_mode_ENT;
    if current_optimal_mode_ENT_index == -1 %when all 3 are equivalent
        disamb_modes_ENT(i) = 4;
    end
    
    %KL:
    current_optimal_mode_KL_index = compute_optimal_mode_KL_R3(intent_type, xr, pg0); 
    if length(current_optimal_mode_KL_index) > 1 %when there are equivalent modes. 
        current_optimal_mode_KL = cm{current_optimal_mode_KL_index(1)}; %pick the first one. 
        current_optimal_mode_KL_index = current_optimal_mode_KL_index(1); %update the index. 
    else
        if current_optimal_mode_KL_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
            current_optimal_mode_KL = cm{current_optimal_mode_KL_index}; 
        end
    end
    disamb_modes_KL(i) = current_optimal_mode_KL; 
    if current_optimal_mode_KL_index == -1 %when all 3 are equivalent
        disamb_modes_KL(i) = 4;
    end
end

%%
xrange = [-0.8, 0.8];
yrange = [-0.8, 0.8];
zrange = [-0.8, 0.8];
figure;grid on; hold on;
colors = {'r','g','b','m'};

for i=1:length(colors)
    scatter3(ws_points(disamb_modes_ENT == i, 1), ws_points(disamb_modes_ENT == i, 2), ws_points(disamb_modes_ENT == i, 3), 70, colors{i}, 'filled'); hold on;
end
scatter3(xg(1,1:ng), xg(2,1:ng), xg(3,1:ng), 580, 'k', 'filled');
line(xrange, [0,0], [0,0], 'Color', 'r', 'LineWidth', 3.5); %draw x and y axes.
line([0,0], yrange, [0,0], 'Color', 'g','LineWidth', 5.5);
line([0,0], [0,0], zrange, 'Color', 'b','LineWidth', 3.5);
axis([xrange, yrange, zrange]);
axis square;
view([32,16]);
xlabel('Spatial X'); ylabel('Spatial Y');  zlabel('Spatial Z'); title('Best Disamb Control Modes');
title('ENTROPY BASED METRIC')

%%
figure;grid on; hold on;
colors = {'r', 'g', 'b', 'm'};
for i=1:length(colors)
    scatter3(ws_points(disamb_modes_KL == i, 1), ws_points(disamb_modes_KL == i, 2), ws_points(disamb_modes_KL == i, 3), 70, colors{i}, 'filled'); hold on;
end
scatter3(xg(1,1:ng), xg(2,1:ng), xg(3,1:ng), 580, 'k', 'filled');
line(xrange, [0,0], [0,0], 'Color', 'r', 'LineWidth', 3.5); %draw x and y axes.
line([0,0], yrange, [0,0], 'Color', 'g','LineWidth', 5.5);
line([0,0], [0,0], zrange, 'Color', 'b','LineWidth', 3.5);
axis([xrange, yrange, zrange]);
axis square;
view([32,16]);
xlabel('Spatial X'); ylabel('Spatial Y');  zlabel('Spatial Z'); title('Best Disamb Control Modes');
title('KL BASED METRIC')