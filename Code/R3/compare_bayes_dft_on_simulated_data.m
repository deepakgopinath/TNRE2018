clc; clear all; close all;

%% LOAD directory with simulated data.

trajectories_folder = 'simulated_trajectories';
fnames = dir(trajectories_folder);
num_trajectories = length(fnames)-2;

dft_accuracy_list = zeros(num_trajectories, 1);
bayes_anca_accuracy_list = zeros(num_trajectories, 1);
bayes_uh_accuracy_list = zeros(num_trajectories, 1);
% bayes_uh_n_accuracy = zeros(num_trajectories, 1);
onscreen = true;

global running_cost optimal_cost_from_start;
%%
for i=3:num_trajectories+2
     n = fnames(i).name; 
     load(n); % load the simulated trajectory.
     disp(n);
     traj = traj_zeroed;
     uh_history=uh_history_zeroed;
     len_traj = size(traj, 2);
     if ~strcmp(n, 'trajectory_474.mat')% 363 383 226 408 435 474* 77
         continue
     end
     %initialize probabilitie vectors
     pgs_BAYES_ANCA = zeros(ng, len_traj);
     pgs_BAYES_ANCA(:, 1) = (1/ng)*ones(ng, 1);
     pgs_BAYES_uh = zeros(ng, len_traj);
     pgs_BAYES_uh(:, 1) = (1/ng)*ones(ng, 1);
     pgs_DFT = zeros(ng, len_traj);
     pgs_DFT(:, 1) = (1/ng)*ones(ng, 1);
     
     %grab starting position
     start_pos = traj(:, 1);
     running_cost = 0; %S to U
     optimal_cost_from_start = cost_optimal_traj(start_pos, xg);
     for j=1:len_traj-1
        curr_pos = traj(:, j);
        next_pos = traj(:, j+1);
        uh = uh_history(:, j);
        [ll_bayes_uh, pgs_BAYES_uh(:, j+1)] = compute_bayes_R3_new(uh, curr_pos, pgs_BAYES_uh(:, j), 1);
        [ll_bayes_anca, pgs_BAYES_ANCA(:, j+1)] = compute_bayes_anca(curr_pos, next_pos, pgs_BAYES_ANCA(:, j), 1.0);
        pgs_DFT(:, j+1) = compute_p_of_g_dft_R3(uh, curr_pos, pgs_DFT(:, j));
     end
    %%
    if onscreen
        figure;
        subplot(2,3, 1);
        colors = {'b', 'r', 'k', 'g', 'm'};
        for i=1:size(reaching_goal_pos, 2)
            scatter3(xg(1,i), xg(2,i), xg(3,i), 230, colors{i}, 'filled'); grid on; hold on;
        end
%         scatter3(xg(1,1:ng), xg(2,1:ng), xg(3,1:ng), 230, 'k', 'filled'); grid on; hold on;
        xlabel('\bf X (m)'); ylabel('\bf Y (m)'); zlabel('\bf Z (m)');
        scatter3(xr_start(1), xr_start(2), xr_start(3), 230, 'r', 'filled');
        offset = [-0.3, 0.3];
        line(xrange+offset, [0,0], [0,0], 'Color', 'r', 'LineWidth', 1.5); %draw x and y axes.
        line([0,0], yrange+offset, [0,0], 'Color', 'g','LineWidth', 1.5);
        line([0,0], [0,0], zrange+offset, 'Color', 'b','LineWidth', 1.5);
        axis([xrange+offset, yrange+offset, zrange+offset]);`
        view([142,31]);
        hold on; 
        scatter3(traj(1, :), traj(2,:), traj(3,:), 'k', 'filled');
        scatter3(traj(1, 1), traj(2,1), traj(3,1), 'r', 'filled'); 
        scatter3(traj(1, end), traj(2,end), traj(3,end), 'g', 'filled');
        %%
        subplot(2,3,2); 
        plot(uh_history', 'LineWidth', 2);grid on;
        xlabel('\bf Time Steps'); ylabel('Velocity m/s');
        legend({'x', 'y', 'z'});
        %%
        
        
        subplot(2,3,4);
        plot(pgs_BAYES_ANCA', 'LineWidth', 2.0); grid on;
        xlabel('\bf Time Steps');  ylabel('\bf Goal Probability'); title('\fontsize{8} GOAL PROBABILITIES - Memory');
        ylim([0, 1.0]); % b, r, yellow
        subplot(2,3,5);
        plot(pgs_BAYES_uh', 'LineWidth', 2.0); grid on;
        xlabel('\bf Time Steps'); title('\fontsize{8} GOAL PROBABILITIES - Recursive');
        ylim([0, 1.0]); % b, r, yellow
        subplot(2,3,6)
        plot(pgs_DFT', 'LineWidth', 2.0); grid on;
        xlabel('\bf Time Steps');  title('\fontsize{8} GOAL PROBABILITIES - DFT')
        ylim([0, 1.0]);
%         subplot(1,4,4);
%         plot(pgs_BAYES_uh_n', 'LineWidth', 2.0); grid on;
%         xlabel('\bf Time Steps'); title('Bayesian uh n');
%         ylim([0, 1.0]); % b, r, yellow
    end
    close all;
    %% 363 383 226 408 435 474* 77
    true_goal_index_history_removed = true_goal_index_history;
    true_goal_index_history_removed(min_ind:max_ind) = -1;
    %%
    [~, bayes_uh_g] = max(pgs_BAYES_uh);
    bayes_uh_g(min_ind:max_ind) = -1;
    bayes_uh_accuracy = sum(bayes_uh_g == true_goal_index_history_removed)/length(true_goal_index_history_removed);
    
    [~, bayes_anca_g] = max(pgs_BAYES_ANCA);
    bayes_anca_g(min_ind:max_ind) = -1;
    bayes_anca_accuracy = sum(bayes_anca_g == true_goal_index_history_removed)/length(true_goal_index_history_removed);
    
    [~, dft_g] = max(pgs_DFT);
    dft_g(min_ind:max_ind) = -1;
    dft_accuracy = sum(dft_g == true_goal_index_history_removed)/length(true_goal_index_history_removed);

    fprintf('DFT Accruacy %f\n', sum(dft_g == true_goal_index_history_removed)/length(true_goal_index_history_removed))
    fprintf('BAYES ANCA Accruacy %f\n', sum(bayes_anca_g == true_goal_index_history_removed)/length(true_goal_index_history_removed))
    fprintf('BAYES uh Accruacy %f\n', sum(bayes_uh_g == true_goal_index_history_removed)/length(true_goal_index_history_removed))
    
    fprintf('##################################\n');
    dft_accuracy_list(i-2) = sum(dft_g == true_goal_index_history_removed)/length(true_goal_index_history_removed);
    bayes_anca_accuracy_list(i-2) = sum(bayes_anca_g == true_goal_index_history_removed)/length(true_goal_index_history_removed);
    bayes_uh_accuracy_list(i-2) = sum(bayes_uh_g == true_goal_index_history_removed)/length(true_goal_index_history_removed);
    
end

%%
fprintf('Mean DFT acc %f\n', mean(dft_accuracy_list));
fprintf('Mean Bayes anca acc %f\n', mean(bayes_anca_accuracy_list));
fprintf('Mean Bayes uh acc %f\n', mean(bayes_uh_accuracy_list));

%% plot box plots
figure; hold on; grid on;
y1 = 1.0; y2 = 2.0; y3 = 3.0; y4= 4.0;
% boxplot(dft_accuracy);
bh1 = boxplot(100*dft_accuracy_list, 'positions', y1, 'whisker', 1,'Widths', 0.3);set(bh1(:,1),'linewidth',2);
bh2 = boxplot(100*bayes_anca_accuracy_list, 'positions', y2, 'whisker', 1,'Widths', 0.3);set(bh2(:,1),'linewidth',2);
bh3 = boxplot(100*bayes_uh_accuracy_list, 'positions', y3, 'whisker', 1,'Widths', 0.3);set(bh3(:,1),'linewidth',2);
% bh4 = boxplot(100*bayes_uh_n_accuracy, 'positions', y4, 'whisker', 1,'Widths', 0.3);set(bh4(:,1),'linewidth',2);
axis([0,4,0,120]);
set(gca,'fontWeight','bold','Xtick',1:1:3);
set(gca, 'XTickLabel', {'DFT', 'Memory', 'Recursive'});
% set(gca, 'fontWeight', 'normal', 'YTick', 0:10:100);
ylabel('\bf \fontsize{11} Inference Accurancy (%)');
title('Goal Inference Accuracy Comparison');

%%
figure;
x = [dft_accuracy_list, bayes_anca_accuracy_list, bayes_uh_accuracy_list];
[p,tbl,stats] = kruskalwallis(x,[],'off');
c = multcompare(stats);