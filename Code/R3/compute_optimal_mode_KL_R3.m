function [ best_mode ] = compute_optimal_mode_KL_R3(intent_type, xr_T, pg)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

global cm nd;
EID_AR = zeros(length(cm), 1); %Information density for each mode. 
for i=1:length(cm)
    curr_mode = cm{i};
    if length(curr_mode) == 3 %mode with 3 dimensions per mode here. 
        [x,y,z] = meshgrid([-0.2, 0.2]);
        combinations = [x(:) y(:) z(:)];
        for j=1:size(combinations, 1) % 8 combinations
            uh_t = zeros(nd,1);
            uh_t(curr_mode(1)) = combinations(j, 1); uh_t(curr_mode(2)) = combinations(j, 2); uh_t(curr_mode(3)) = combinations(j, 3);
            EID_AR(i) = EID_AR(i) + (1/size(combinations, 1))*compute_projected_kl_R3(uh_t, intent_type, xr_T, pg);
        end
    elseif length(curr_mode) == 2
        [x,y] = meshgrid([-0.2, 0.2]); %4 combinations of max vel for each 2d control mode. 
        combinations = [x(:), y(:)];
        for j=1:size(combinations, 1) %4 combinations possible. 
            uh_t = zeros(nd,1);
            uh_t(curr_mode(1)) = combinations(j, 1); uh_t(curr_mode(2)) = combinations(j, 2);
            EID_AR(i) = EID_AR(i) + (1/size(combinations, 1))*compute_projected_kl_R3(uh_t, intent_type, xr_T, pg);
        end
    elseif length(curr_mode) == 1
        possible_vel = [-0.2, 0.2];
        for j=1:length(possible_vel)
            uh_t = zeros(nd,1);
            uh_t(curr_mode) = possible_vel(j);
            EID_AR(i) = EID_AR(i) + (1/length(possible_vel))*compute_projected_kl_R3(uh_t, intent_type, xr_T, pg);
        end
    end
end

best_mode = compute_best_mode(EID_AR);
end


