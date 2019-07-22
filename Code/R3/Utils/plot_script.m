figure; 
subplot(3,4,1);
plot(pgs_ENT','LineWidth', 1.5); grid on;
ylim([0.0, 1.0]);
title('ENTROPY PG'); xlabel('Time Steps'); ylabel('PG');

subplot(3,4,2);
plot(pgs_KL', 'LineWidth', 1.5); grid on;
title('KL PG'); xlabel('Time Steps'); ylabel('PG')
ylim([0.0, 1.0]);

% subplot(3,4,2);
% plot(pgs_FI', 'LineWidth', 1.5); grid on;
% title('FI PG'); xlabel('Time Steps'); ylabel('PG')
% ylim([0.0, 1.0]);

subplot(3,4,3);
plot(pgs_DISAMB', 'LineWidth', 1.5); grid on;
title('DISAMB PG'); xlabel('Time Steps'); ylabel('PG')
ylim([0.0, 1.0]);

subplot(3,4,4);
plot(pgs_POT', 'LineWidth', 1.5); grid on;
title('POT PG'); xlabel('Time Steps'); ylabel('PG')
ylim([0.0, 1.0]);

subplot(3,4,5);
scatter(1:length(optimal_modes_ENT), optimal_modes_ENT, 'r', 'filled'); 
title('ENTROPY MODES');
% 
subplot(3,4,6);
scatter(1:length(optimal_modes_KL), optimal_modes_KL, 'b', 'filled');
title('KL MODES');
% subplot(3,4,6);
% scatter(1:length(optimal_modes_FI), optimal_modes_FI, 'b', 'filled');
% title('FI MODES');

subplot(3,4,7);
scatter(1:length(optimal_modes_DISAMB), optimal_modes_DISAMB, 'g', 'filled');
title('DISAMB MODES');

subplot(3,4,8);
scatter(1:length(optimal_modes_POT), optimal_modes_POT, 'k', 'filled');
title('POT MODES');
%%
color_scheme = {'b', 'r', 'y', 'pur', 'green', 'm', 'k'};
fprintf('THE RANDOM GOAL IS %d and COLOR is %s\n', random_goal_index, color_scheme{random_goal_index});
% fprintf('THE NUMBER OF MODES SWITCHES POT %d, ENT %d, FI %d and DISAMB %d\n',sum(abs(diff(optimal_modes_POT))), sum(abs(diff(optimal_modes_ENT))), sum(abs(diff(optimal_modes_FI))), sum(abs(diff(optimal_modes_DISAMB))));

fprintf('THE NUMBER OF MODES SWITCHES POT %d, ENT %d, KL %d and DISAMB %d\n',sum(abs(diff(optimal_modes_POT))), sum(abs(diff(optimal_modes_ENT))), sum(abs(diff(optimal_modes_KL))), sum(abs(diff(optimal_modes_DISAMB))));


%b,r,y, purple, green

subplot(3,4,9);
plot(1:length(alpha_ENT), (curr_goal_ENT == random_goal_index).*alpha_ENT, 'r', 'LineWidth', 1.5);
ylim([0.0, 1.0]);
title('ALPHA ENT');

subplot(3,4,10);
plot(1:length(alpha_KL), (curr_goal_KL == random_goal_index).*alpha_KL, 'b','LineWidth', 1.5);
ylim([0.0, 1.0]);
title('ALPHA KL');
% subplot(3,4,10);
% plot(1:length(alpha_FI), (curr_goal_FI == random_goal_index).*alpha_FI, 'b','LineWidth', 1.5);
% ylim([0.0, 1.0]);
% title('ALPHA FI');

subplot(3,4,11);
plot(1:length(alpha_DISAMB), (curr_goal_DISAMB == random_goal_index).*alpha_DISAMB, 'g','LineWidth', 1.5);
ylim([0.0, 1.0]);
title('ALPHA DISAMB');

subplot(3,4,12);
plot(1:length(alpha_POT), (curr_goal_POT == random_goal_index).*alpha_POT, 'k','LineWidth', 1.5);
ylim([0.0, 1.0]);
title('POT DISAMB');

