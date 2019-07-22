function [ best_mode ] = compute_optimal_mode_DISAMB_R3( intent_type, curr_x, pg )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

% first compute Dk for each dimension. Then compute Dm for each mode. Then
% perform best mode compiutation. 
global cm nd;
Dk = zeros(nd, 1);
Dcm = zeros(length(cm), 1); %Information density for each mode. 
%compute Dk first. Always euqal to the number of dimensions. 
for i=1:nd
    uh_p = zeros(nd, 1); uh_n = zeros(nd, 1);
    uh_p(i) = 0.2; uh_n(i) = -0.2;  %bpoth along positive and negative directions. 
    Dk(i) = compute_disamb_heuristic_R3(uh_p, intent_type, curr_x, pg) + compute_disamb_heuristic_R3(uh_n, intent_type, curr_x, pg); 
end

for i=1:length(cm)
    Dcm(i) = sum(Dk(cm{i}));
end
best_mode = compute_best_mode(Dcm);
end

