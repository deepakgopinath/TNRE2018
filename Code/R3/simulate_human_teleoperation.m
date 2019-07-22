clear all; close all; clc;
%% 

%global variables. 
global num_modes cm ng max_ng nd xg xr xrange yrange zrange goal_transition_P dist_threshold sig delta_t conf_thresh conf_max alpha_max num_samples sparsity_factor amp_sparsity_factor kappa projection_time;

%workspace limits
xrange = [-0.5, 0.5];
yrange = [-0.5, 0.5];
zrange = [-0.5, 0.5];
%workspace params.
% ng = 3; %num of goals
max_ng = 5;

ng = datasample(2:max_ng, 1); %spawn random number of goals. Maximum number is 6. At least 
nd = 3; %num of dimensions. by definition R^3
% cm_options = {{1,2,3}, {[1,2], 3}};
cm_options = {{[1,2,3]}};
max_nm = length(cm_options);
cm = cm_options{datasample(1:max_nm, 1)};
% cm = {1,2,3}; %{1,2,3}, %{[1,3], 2} %{[2,3], 1};
num_modes = length(cm); %
init_mode_index = datasample(1:num_modes, 1);

%spawn random goals and robot position. 
xg = [rand(1,ng)*range(xrange) + xrange(1); rand(1,ng)*range(yrange) + yrange(1); rand(1,ng)*range(zrange) + zrange(1)]; %random goal positions. These will be treated as fixed parameters.
xr = [rand(1,1)*range(xrange) + xrange(1); rand(1,1)*range(yrange) + yrange(1); rand(1,1)*range(zrange) + zrange(1)];
xr_start = xr;

delta_t = 0.002; %For compute projections. 
%human parameters
sparsity_factor = rand/8;
amp_sparsity_factor = rand/8; % how often the amplitude wiull be less that maximum. 
kappa = 20.0; % concentration paarameter for vonMisesFisher distribution
dist_threshold = 0.01;
goal_transition_P = eye(ng) + rand(ng)/100;
s = repmat(sum(goal_transition_P, 2),1,ng);
goal_transition_P = goal_transition_P./s;



fprintf('The sparsity and amp factor are %f, %f\n', sparsity_factor, amp_sparsity_factor);

%% plot the goals and robot
figure;
scatter3(xg(1,1:ng), xg(2,1:ng), xg(3,1:ng), 230, 'k', 'filled'); grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
scatter3(xr_start(1), xr_start(2), xr_start(3), 230, 'r', 'filled');
offset = [-0.3, 0.3];
line(xrange+offset, [0,0], [0,0], 'Color', 'r', 'LineWidth', 1.5); %draw x and y axes.
line([0,0], yrange+offset, [0,0], 'Color', 'g','LineWidth', 1.5);
line([0,0], [0,0], zrange+offset, 'Color', 'b','LineWidth', 1.5);
axis([xrange+offset, yrange+offset, zrange+offset]);
axis square;
view([142,31]);

%% Generate the random goal towards which the simulated human would move. 
random_goal_index = randsample(ng, 1);
random_goal = xg(:, random_goal_index);

%% Use human policy
traj = xr_start;
uh_history = [];
true_goal_index_history = random_goal_index;
num_trajectories = 500;
disp(xr);
goal_switch_marker = randi(5);
counter = 0;
for i=1:num_trajectories
    %respawn goals
    sparsity_factor = rand/8;
    amp_sparsity_factor = rand/8; % how often the amplitude wiull be less that maximum. 
    fprintf('The sparsity and amp factor are %f, %f\n', sparsity_factor, amp_sparsity_factor);
    while true
        if mod(counter, goal_switch_marker) == 0
            [random_goal, random_goal_index] = generate_new_goal(random_goal_index);
            goal_switch_marker = randi(5);
%             disp(goal_switch_marker);
            counter = 0;
        end
        uh = generate_full_uh(random_goal, xr);
        if rand < 0.01
            rand_ind = randsample(nd, 1);
            uh(rand_ind) = 0;
        end
        if rand < 0.05
            uh = uh;
        end
        xr = sim_kinematics_R3(xr, uh);
        uh_history = [uh_history, uh];
        traj = [traj, xr];
        true_goal_index_history = [true_goal_index_history, random_goal_index];
        if check_proximity(random_goal, xr)
            break;
        end
        counter = counter + 1;
    end
    %%
    min_ind = randi(round(length(uh_history)/2));
    max_ind = min(length(uh_history), min_ind + randi(200));
    uh_history_zeroed = uh_history;
    uh_history_zeroed(:, min_ind:max_ind) = 0;
    xr = xr_start;
    traj_zeroed = xr_start;
    for j=1:size(uh_history_zeroed, 2)-1
        uh = uh_history_zeroed(:, j);
        xr = sim_kinematics_R3(xr, uh);
        traj_zeroed = [traj_zeroed, xr];
    end
    traj_zeroed = [traj_zeroed, xr];
    %%
    
    %save xg, xr, nd, ng,  traj, true_goal_history, 
    filename = strcat('simulated_trajectories/trajectory_', int2str(i), '.mat');
    save(filename, 'xg', 'xr','xr_start', 'min_ind', 'max_ind','ng', 'traj', 'traj_zeroed', 'uh_history_zeroed','delta_t', 'uh_history', 'goal_transition_P', 'true_goal_index_history', 'xrange','yrange','zrange');
    %respawn goals
    respawn_goals_and_goal_transition_P();
    xr_start = xr;
    %reinit variables to hold trajectories
    traj = xr;
    uh_history = [];
    random_goal_index = randsample(ng, 1);
    true_goal_index_history = random_goal_index;
    goal_switch_marker = randi(5);
    counter = 0;
end

%%
% hold on; 
% % scatter3(traj(:, 1), traj(:, 2), traj(:, 3), 'k', 'filled');
% scatter3(traj(1, :), traj(2,:), traj(3,:), 'k', 'filled');


%% function to respawn goals

function respawn_goals_and_goal_transition_P
    global ng max_ng xrange yrange zrange xg xr goal_transition_P;
    ng = datasample(2:max_ng, 1);
    xg = [rand(1,ng)*range(xrange) + xrange(1); rand(1,ng)*range(yrange) + yrange(1); rand(1,ng)*range(zrange) + zrange(1)]; %random goal positions. These will be treated as fixed parameters.
    xr = [rand(1,1)*range(xrange) + xrange(1); rand(1,1)*range(yrange) + yrange(1); rand(1,1)*range(zrange) + zrange(1)];
    goal_transition_P = eye(ng) + rand(ng)/100;
    s = repmat(sum(goal_transition_P, 2),1,ng);
    goal_transition_P = goal_transition_P./s;
end
%%
function uh = generate_full_uh(xg, xr) %full unnomralized uh
    global nd sparsity_factor kappa;
    mu = xg - xr;
    if ~any(mu)
        uh = zeros(nd, 1);
    else
        uh = randvonMisesFisherm(nd, 1, kappa*10000, mu);
    end
    
    %add sparsity
    if rand < sparsity_factor
        uh = zeros(nd, 1);
    end
end
%%
function isclose = check_proximity(xg, xr)
    global dist_threshold;
    if norm(xg-xr) < dist_threshold
        isclose = true;
    else
        isclose = false;
    end
end

%%
function [new_goal, new_goal_index] = generate_new_goal(random_goal_index)
    global goal_transition_P xg;
    new_goal_index = find(mnrnd(1, goal_transition_P(random_goal_index, :)) == 1);
    new_goal = xg(:, new_goal_index);
    
end