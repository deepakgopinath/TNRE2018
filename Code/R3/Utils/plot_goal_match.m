figure;


subplot(4,2,1);
scatter(1:length(curr_goal_ENT), curr_goal_ENT == random_goal_index, 'r', 'filled'); grid on;
title('ENT GOAL MATCH'); xlabel('Time Steps'); ylabel('Matched/Unmatched'); ylim([0.0, 1.0]); xlim([0, 120]);
xlim([0, 120])
subplot(4,2,2);
scatter(1:length(curr_goal_ENT), curr_goal_ENT, 'r', 'filled'); grid on;
title('ENT GOAL INFERENCE'); xlabel('Time Steps'); ylabel('Goal Index');ylim([1, ng]);xlim([0, 120]);

subplot(4,2,3);
scatter(1:length(curr_goal_KL), curr_goal_KL == random_goal_index, 'b', 'filled'); grid on;
title('KL GOAL MATCH'); xlabel('Time Steps'); ylabel('Matched/Unmatched');ylim([0.0, 1.0]);xlim([0, 120]);
subplot(4,2,4);
scatter(1:length(curr_goal_KL), curr_goal_KL, 'b', 'filled'); grid on;
title('KL GOAL INFERENCE'); xlabel('Time Steps'); ylabel('Goal Index');ylim([1, ng]);xlim([0, 120]);

% subplot(4,2,3);
% scatter(1:length(curr_goal_FI), curr_goal_FI == random_goal_index, 'b', 'filled'); grid on;
% title('FI GOAL MATCH'); xlabel('Time Steps'); ylabel('Matched/Unmatched');ylim([0.0, 1.0]);
% subplot(4,2,4);
% scatter(1:length(curr_goal_FI), curr_goal_FI, 'b', 'filled'); grid on;
% title('FI GOAL INFERENCE'); xlabel('Time Steps'); ylabel('Goal Index');ylim([1, ng]);

subplot(4,2,5);
scatter(1:length(curr_goal_DISAMB), curr_goal_DISAMB == random_goal_index, 'g', 'filled'); grid on;
title('DISAMB GOAL MATCH'); xlabel('Time Steps'); ylabel('Matched/Unmatched');ylim([0.0, 1.0]);xlim([0, 120]);
subplot(4,2,6);
scatter(1:length(curr_goal_DISAMB), curr_goal_DISAMB,'g', 'filled'); grid on;
title('DISAMB GOAL INFERENCE'); xlabel('Time Steps'); ylabel('Goal Index');ylim([1, ng]);xlim([0, 120]);

subplot(4,2,7);
scatter(1:length(curr_goal_POT), curr_goal_POT == random_goal_index, 'k','filled'); grid on;
title('POT GOAL MATCH'); xlabel('Time Steps'); ylabel('Matched/Unmatched');ylim([0.0, 1.0]);xlim([0, 120]);
subplot(4,2,8);
scatter(1:length(curr_goal_POT), curr_goal_POT, 'k', 'filled'); grid on;
title('POT GOAL INFERENCE'); xlabel('Time Steps'); ylabel('Goal Index');ylim([1, ng]);xlim([0, 120]);