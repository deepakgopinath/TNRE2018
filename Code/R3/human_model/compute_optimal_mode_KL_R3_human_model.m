function [ best_mode ] = compute_optimal_mode_KL_R3_human_model( intent_type, xr_T, pg )
%COMPUTE_OPTIMAL_MODE_KL_R2_HUMAN_MODEL Summary of this function goes here
%   Detailed explanation goes here

global cm xg ng num_samples;

%for each mode in cm forward project using full human model and alpha value
%etc. 
EID_AR = zeros(length(cm), 1);
for i=1:length(cm)
    curr_mode = cm{i};
    for j=1:ng
        projected_kl = zeros(num_samples, 1);
        for k=1:num_samples
            curr_goal = xg(:, j);
            projected_kl(k) = compute_projected_kl_R3_human_model(curr_mode, curr_goal, intent_type, xr_T, pg);
        end
        expected_projected_kl = mean(projected_kl);
        EID_AR(i) = EID_AR(i) + (1/ng)*expected_projected_kl;
    end
end
best_mode = compute_best_mode(EID_AR);
end



