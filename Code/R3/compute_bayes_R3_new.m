function [ ll, posterior ] = compute_bayes_R3_new( uh, xr, prior,exp_factor )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    global ng goal_transition_P;
    ll = likelihood_func(uh, xr);
    prior = prior + 0.0*rand(ng, 1); %to avoid collapse of posterior. 
    prior = prior/sum(prior);
%     disp(ll);
    posterior = (ll.^(exp_factor)).*(goal_transition_P'*prior);
    posterior = posterior/sum(posterior); %make sure that the posterior is always normalized. 
end

function likelihood = likelihood_func(uh, xr)
    global ng xg;
    likelihood = zeros(ng, 1);
    if norm(uh)~=0
        uh = uh/norm(uh);
    else
        likelihood = (1/ng)*ones(ng, 1);
%         likelihood = zeros(ng, 1) + realmin;
        return;
    end
    for i=1:ng
        dir_vec = xg(:, i) - xr; %vector joining the robot position and the goal_x. 
        dir_vec = dir_vec/norm(dir_vec); %as long as xr is not xg, this would be nonzero;
        likelihood(i) = p_of_u_given_g(uh, dir_vec);
    end
end

function [q] = p_of_u_given_g(uh, dir_vec) %assumes that the uh that is expected to see for a goal is the one which is directly pointing at it. 
    ang = acos(round(dot(uh, dir_vec), 5));
    q = prob(ang);
end

function p = prob(a) %zero mean gaussian, 
    sigma = 0.5*pi; %for the time being use this std dev
    p = (1/sqrt(2*pi*sigma^2))*exp((-1/2.0)*(a^2/sigma^2));
end