clc; clear all; close all;
%%
%global variables. 
global num_modes cm ng nd xg xr xrange yrange zrange goal_transition_P dist_threshold sig delta_t conf_thresh conf_max alpha_max num_samples sparsity_factor amp_sparsity_factor kappa projection_time;

%workspace limits
xrange = [-0.5, 0.5];
yrange = [-0.5, 0.5];
zrange = [-0.5, 0.5];

ng = 3;
nd = 3;
cm_options = {{[1,2,3]}};
max_nm = length(cm_options);
cm = cm_options{datasample(1:max_nm, 1)};
% cm = {1,2,3}; %{1,2,3}, %{[1,3], 2} %{[2,3], 1};
num_modes = length(cm); %
init_mode_index = datasample(1:num_modes, 1);

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

%%
random_goal_index = randsample(ng, 1);
random_goal = xg(:, random_goal_index);


%%
traj = xr_start;
uh_history = [];
true_goal_index_history = random_goal_index;
% disp(xr);
goal_switch_marker = randi(5);
counter = 0;
while true
    if mod(counter, goal_switch_marker) == 0
        [random_goal, random_goal_index] = generate_new_goal(random_goal_index);
        goal_switch_marker = randi(5);
%         disp(goal_switch_marker);
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

%% modify uh and recreate a new traj
min_ind = randi(round(length(uh_history)/2));
max_ind = min(length(uh_history), min_ind + randi(500));
uh_history_zeroed = uh_history;
uh_history_zeroed(:, min_ind:max_ind) = 0;
xr = xr_start;
traj_zeroed = xr_start;
for i=1:size(uh_history_zeroed, 2)-1
    uh = uh_history_zeroed(:, i);
    xr = sim_kinematics_R3(xr, uh);
    traj_zeroed = [traj_zeroed, xr];
end
traj_zeroed = [traj_zeroed, xr];
%% bayesian inference
% len_traj = size(traj, 2);
% traj_new=traj; %make time along columns
traj_new = traj_zeroed;
% uh_history_new=uh_history;
uh_history_new=uh_history_zeroed;
len_traj = size(traj_new, 2);
%%
pgs_BAYES_uh = zeros(ng, len_traj);
pgs_BAYES_uh(:, 1) = (1/ng)*ones(ng, 1); % prior at t=0
pgs_BAYES_ANCA = zeros(ng, len_traj);
pgs_BAYES_ANCA(:, 1) = (1/ng)*ones(ng, 1);
pgs_DFT = zeros(ng, len_traj);
% pgs_DFT(:, 1) = (1/ng)*ones(ng, 1);
pgs_DFT(:, 1) = rand(3,1); pgs_DFT(:, 1) = pgs_DFT(:, 1)/sum(pgs_DFT(:, 1));

%%
start_pos = traj_new(:, 1);
global running_cost optimal_cost_from_start;
running_cost = 0; %S to U
optimal_cost_from_start = cost_optimal_traj(start_pos, xg);
for j=1:len_traj-1
    curr_pos = traj_new(:, j);
    next_pos = traj_new(:, j+1);
    uh = uh_history_new(:, j);
    [ll_bayes_uh, pgs_BAYES_uh(:, j+1)] = compute_bayes_R3_new(uh, curr_pos, pgs_BAYES_uh(:, j), 1);
    [ll_bayes_anca, pgs_BAYES_ANCA(:, j+1)] = compute_bayes_anca(curr_pos, next_pos, pgs_BAYES_ANCA(:, j), 1.0);
    pgs_DFT(:, j+1) = compute_p_of_g_dft_R3(uh, curr_pos, pgs_DFT(:, j));
end

%%
hold on; 
scatter3(traj_new(1, :), traj_new(2,:), traj_new(3,:), 'k', 'filled');
%%
figure;
plot(uh_history_new');
%%
subplot(1,3,1);
% hold on;
plot(pgs_BAYES_uh', 'LineWidth', 2.0); grid on;
xlabel('\bf Time Steps'); title('Bayesian uh');
ylim([0, 1.0]); % b, r, yellow
subplot(1,3,2);
% hold on;
plot(pgs_BAYES_ANCA', 'LineWidth', 2.0); grid on;
xlabel('\bf Time Steps'); title('Bayesian anca');
ylim([0, 1.0]); % b, r, yellow
subplot(1,3,3);
% hold on;
plot(pgs_DFT', 'LineWidth', 2.0); grid on;
xlabel('\bf Time Steps'); title(' DFT');
ylim([0, 1.0]); % b, r, yellow
%%
true_goal_index_history_removed = true_goal_index_history;
true_goal_index_history_removed(min_ind:max_ind) = -1;
%%
[~, bayes_uh_g] = max(pgs_BAYES_uh);
bayes_uh_g(min_ind:max_ind) = -1;
bayes_uh_accuracy = sum(bayes_uh_g == true_goal_index_history_removed)/length(true_goal_index_history_removed);
fprintf('Goal Accuracy BAYES_uh %f\n', bayes_uh_accuracy);
figure; grid on; hold on;
plot(true_goal_index_history_removed, 'r', 'LineWidth',  2); hold on;
plot(bayes_uh_g'+4, 'b','LineWidth',  2);

[~, bayes_anca_g] = max(pgs_BAYES_ANCA);
bayes_anca_g(min_ind:max_ind) = -1;
bayes_anca_accuracy = sum(bayes_anca_g == true_goal_index_history_removed)/length(true_goal_index_history_removed);
fprintf('Goal Accuracy BAYES_anca %f\n', bayes_anca_accuracy);
plot(true_goal_index_history_removed, 'r', 'LineWidth',  2); hold on;
plot(bayes_anca_g'+8, 'g','LineWidth',  2);

[~, bayes_dft_g] = max(pgs_DFT);
bayes_dft_g(min_ind:max_ind) = -1;
bayes_dft_accuracy = sum(bayes_dft_g == true_goal_index_history_removed)/length(true_goal_index_history_removed);
fprintf('Goal Accuracy DFT %f\n', bayes_dft_accuracy);
plot(true_goal_index_history_removed, 'r', 'LineWidth',  2); hold on;
plot(bayes_dft_g'+12, 'k','LineWidth',  2);





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