clear all; clc; close all;
%%
reaching_goal_pos = [   0.328, -0.516, 0.1;
                        -0.116, -0.579, 0.132;
                        0.124, -0.531, 0.442;
                        -0.346, -0.389, 0.220;
                        -0.376, -0.180, 0.102
                        ]';
r_ng = size(reaching_goal_pos, 2);
pouring_goal_pos = [    0.3314, -0.206, 0.1;
                        0.306, -0.550, 0.256;
                        -0.05, -0.627, 0.225;
                        -0.370, -0.55504, 0.3955
                        ]';
goal_pos = [0.328, -0.516, 0.1;
            -0.116, -0.579, 0.132;
            0.124, -0.531, 0.442]';
                    
p_ng = size(reaching_goal_pos, 2);
ng = size(goal_pos, 2);
%%
% [pos, ori, user_vel, gps] = process_bag_tf('H2PH2REJ2T16.bag');
% [pos, ori, user_vel, gps] = process_bag_tf('H2PH2REHAT2.bag');
[pos, ori, user_vel, gps] = process_bag_tf('bag1.bag');

%%

% subplot(2,2,1);
figure;
hold on;
step_size = 30;
%%
scatter3(pos(1, 1:step_size:end),pos(2, 1:step_size:end),pos(3, 1:step_size:end), 'b', 'filled'); hold on;
scatter3(pos(1, 1),pos(2,1),pos(3, 1), 100, 'r', 'filled'); hold on;
scatter3(pos(1,end),pos(2, end),pos(3, end),100, 'g', 'filled'); hold on;

xlabel('\bf X (m)'); ylabel('\bf Y (m)'); zlabel('\bf Z (m)');
axis([-0.3, 0.6, -0.6, 0.1, 0, 0.7]);
view([-160, 24]);

colors = {'k', 'm', 'r', 'g', 'b'};
for i=1:size(goal_pos, 2)
    scatter3(goal_pos(1,i), goal_pos(2,i), goal_pos(3,i), 450, colors{i}, 'filled'); grid on; hold on;
end

%% ANCA's DISTANCE BASED COST FUNCTION IMPLMENTATION. 

% ng = r_ng;
% goal_pos = reaching_goal_pos;

% ng = p_ng;
% goal_pos = pouring_goal_pos;


T = size(pos, 2);
running_cost = 0; %S to U
optimal_cost_from_start = zeros(ng, 1); %S to G's, one time computation r_ng - for reaching, p_ng - for pouring
optimal_cost_to_go = zeros(ng, 1); % U to G
start_pos = pos(:, 1);
optimal_cost_from_start = cost_optimal_traj(start_pos, goal_pos);
pgs_BAYES = zeros(ng, length(T));
pgs_BAYES(:, 1) = (1/ng)*ones(ng,1); %init pg
%%
for i=1:T-1
    curr_pos = pos(:, i+1);
    optimal_cost_to_go = cost_optimal_traj(curr_pos, goal_pos);
    running_cost = running_cost + sum((pos(:, i+1) - pos(:, i)).^2); %seocnd terms is the distance between subsequent terms. 
    disp(running_cost);
    ll = compute_likelihood(running_cost, optimal_cost_to_go, optimal_cost_from_start);
    prior = pgs_BAYES(:, i);
    prior = prior + 0.01*rand(ng, 1); %to avoid collapse of posterior. 
    prior = prior/sum(prior);
    pgs_BAYES(:, i+1) = ll.*prior;
    pgs_BAYES(:, i+1) = pgs_BAYES(:, i+1)/sum(pgs_BAYES(:, i+1));
end

%%
subplot(1,2,1)
plot(gps', 'LineWidth', 2.0); grid on;
xlabel('\bf Time Steps'); ylabel('\bf Goal Probabilities'); title('DFT Inference')
ylim([0, 1.0]);
subplot(1,2,2);
plot(pgs_BAYES', 'LineWidth', 2.0); grid on;
xlabel('\bf Time Steps'); title('Bayesian');
ylim([0, 1.0]); % b, r, yellow

%% GOAL INFERENCE

[~,dft_g] = max(gps);
[~, bayes_g] = max(pgs_BAYES);

%% DFT GOAL INFERENCE WITH TRAJ
figure;
subplot(1,2,1);
hold on;
step_size = 40;
colors = {'b', 'r', 'y'};
for i=1:step_size:size(pos, 2)
    scatter3(pos(1, i),pos(2, i),pos(3, i), colors{dft_g(i)}, 'filled'); hold on;
end
% scatter3(pos(1,end),pos(2, end),pos(3, end),100, 'g', 'filled'); hold on;

xlabel('\bf X (m)'); ylabel('\bf Y (m)'); zlabel('\bf Z (m)'); title('DFT Inference')
axis([-0.3, 0.6, -0.6, 0.2, 0, 0.7]);
view([-217, 34]);

colors = {'b', 'r', 'y'};
for i=1:size(goal_pos, 2)
    scatter3(goal_pos(1,i), goal_pos(2,i), goal_pos(3,i), 450, colors{i}, 'filled'); grid on; hold on;
end

%% BAYESIAN GOAL INFERENCE WITH TRAJ

subplot(1,2,2);
hold on;
colors = {'b', 'r', 'y'};
for i=1:step_size:size(pos, 2)
    scatter3(pos(1, i),pos(2, i),pos(3, i), colors{bayes_g(i)}, 'filled'); hold on;
end
% scatter3(pos(1,end),pos(2, end),pos(3, end),100, 'g', 'filled'); hold on;

xlabel('\bf X (m)'); ylabel('\bf Y (m)'); zlabel('\bf Z (m)'); title('Bayesian Inference')
axis([-0.3, 0.6, -0.6, 0.2, 0, 0.7]);
view([-217, 34]);

colors = {'b', 'r', 'y'};
for i=1:size(goal_pos, 2)
    scatter3(goal_pos(1,i), goal_pos(2,i), goal_pos(3,i), 450, colors{i}, 'filled'); grid on; hold on;
end
