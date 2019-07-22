function [ projected_pg ] = compute_pg_projection_R3_human_model(curr_mode, curr_goal, projection_time, varargin )
%COMPUTE_PROJECTED_ENTROPY_R2_WITH_HUMAN_MODEL Summary of this function goes here
%   Detailed explanation goes here
global delta_t ng nd;

intent_type = varargin{1};
curr_x = varargin{2};

if strcmp(intent_type, 'conf')
    uh = generate_model_uh(curr_goal, curr_x);
    zero_dim = setdiff(1:nd,curr_mode);
    for jj=1:length(zero_dim) %mode conditioned humna policy
        uh(zero_dim(jj)) = 0; %zero out the components of the uh that are not accessible via the current control mode
    end
    if norm(uh) > 0.2
        uh = 0.2*(uh./(norm(uh) + realmin));
    end
    pg = compute_conf_R3(uh, curr_x);
elseif strcmp(intent_type, 'dft')
    pg0 = varargin{3};
    t_stamps = 0:delta_t:projection_time;
    pgs_project = zeros(ng, length(t_stamps)); %array to store evolved probabilities. 
    pgs_project(:, 1) = pg0; %init pg0
    for i=1:length(t_stamps)-1
        curr_goal_index = datasample(find(pgs_project(:, i) == max(pgs_project(:, i))), 1);
        uh = generate_model_uh(curr_goal, curr_x); %generate human policy
        zero_dim = setdiff(1:nd,curr_mode);
        for jj=1:length(zero_dim) %mode conditioned humna policy
            uh(zero_dim(jj)) = 0; %zero out the components of the uh that are not accessible via the current control mode
        end
        if norm(uh) > 0.2
            uh = 0.2*(uh./(norm(uh) + realmin));
        end
        %generate autonomy
        ur = generate_autonomy(curr_goal_index, curr_x); %autonomy command in full 2D space
        alpha = alpha_from_confidence(pgs_project(curr_goal_index, i)); %linear belnding param
        alpha = 0.0;
        blend_vel = (1-alpha)*uh + alpha*ur; %blended vel
        pgs_project(:, i+1) = compute_p_of_g_dft_R3(uh, curr_x, pgs_project(:, i));
        curr_x = sim_kinematics_R3(curr_x, blend_vel);
    end
    pg = pgs_project(:, end);
elseif strcmp(intent_type, 'bayes')
    pg0 = varargin{3};
    t_stamps = 0:delta_t:projection_time;
    pgs_project = zeros(ng, length(t_stamps)); %array to store evolved probabilities. 
    pgs_project(:, 1) = pg0; %init pg0
    for i=1:length(t_stamps)-1
        curr_goal_index = datasample(find(pgs_project(:, i) == max(pgs_project(:, i))), 1);
        uh = generate_model_uh(curr_goal, curr_x); %generate human policy
        zero_dim = setdiff(1:nd,curr_mode);
        for jj=1:length(zero_dim) %mode conditioned humna policy
            uh(zero_dim(jj)) = 0; %zero out the components of the uh that are not accessible via the current control mode
        end
        if norm(uh) > 0.2
            uh = 0.2*(uh./(norm(uh) + realmin));
        end
        %generate autonomy
        ur = generate_autonomy(curr_goal_index, curr_x); %autonomy command in full 2D space
        alpha = alpha_from_confidence(pgs_project(curr_goal_index, i)); %linear belnding param
        alpha = 0.0;
        blend_vel = (1-alpha)*uh + alpha*ur; %blended vel
        pgs_project(:, i+1) = compute_bayes_R3(uh, curr_x, pgs_project(:, i));
        curr_x = sim_kinematics_R3(curr_x, blend_vel);
    end
    pg = pgs_project(:, end);
end
projected_pg = pg;
end

%% uh
function uh = generate_model_uh(xg, xr) %full unnomralized uh
    global nd kappa;
    mu = xg - xr; %noiseless human velocity model. 
    if ~any(mu)
        uh = zeros(nd, 1);
    else
        uh = randvonMisesFisherm(nd, 1, kappa, mu);
    end
%     uh = randvonMisesFisherm(nd, 1, kappa, mu);
end

%% simple potential field type of autonomy. 
function ur = generate_autonomy(goal_index, curr_x)
    global xg;
    ur = xg(:, goal_index) - curr_x;
    if norm(ur) > 0.20
        ur = 0.2*ur/norm(ur);
    end
end
%% arbitration function

function alpha = alpha_from_confidence(goal_pg)
   global conf_thresh conf_max alpha_max
   if goal_pg <= conf_thresh
       alpha = 0.0;
   elseif goal_pg > conf_thresh && goal_pg <= conf_max
       alpha = (goal_pg - conf_thresh)*(alpha_max/(conf_max - conf_thresh));
   else
       alpha = alpha_max;
   end
end