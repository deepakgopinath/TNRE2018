function [ projected_pg ] = compute_pg_projection_R3(uh, projection_time, varargin)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
global delta_t ng;
intent_type = varargin{1};
curr_x = varargin{2};
if strcmp(intent_type, 'conf')
    pg = compute_conf_R3(uh, curr_x); %confidence function does not care of what the current pg is and therefore ignores pg0
elseif strcmp(intent_type, 'dft')
    %For every 'uh' let the pg system evolve from the 'current' p(g). At
    %the time of intervention store the p(g) away in a variable that will
    %be used for each iteration. And the resulting p(g) is the one for which the entropy will be calculated. 
    
    pg0 = varargin{3}; %pg at the time of optimal mode computation
%     projection_time = 1; %let the DS evolve for projection_time seconds
    t_stamps = 0:delta_t:projection_time; %time stamps for projection. 
    pgs_project = zeros(ng, length(t_stamps)); %array to store evolved probabilities. 
    pgs_project(:, 1) = pg0; %init pg0
    for i=1:length(t_stamps)-1
        pgs_project(:, i+1) = compute_p_of_g_dft_R3(uh, curr_x, pgs_project(:, i));
        curr_x = sim_kinematics_R3(curr_x, uh);
    end
    pg = pgs_project(:, end); %pg after the projected DS evolution. 
elseif strcmp(intent_type, 'bayes')
    pg0 = varargin{3};
%     projection_time = 1;
    t_stamps = 0:delta_t:projection_time; %time stamps for projection. 
    pgs_project = zeros(ng, length(t_stamps)); %array to store evolved probabilities.
    pgs_project(:, 1) = pg0; %init pg0 at the time of assistance. 
    for i=1:length(t_stamps)-1
        pgs_project(:, i+1) = compute_bayes_R3(uh, curr_x, pgs_project(:, i));
        curr_x = sim_kinematics_R3(curr_x, uh);
    end
    pg = pgs_project(:, end);
end
% pg = compute_p_of_g(uh, xr);
projected_pg = pg;
end

