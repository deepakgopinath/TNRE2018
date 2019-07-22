function [ posterior ] = compute_bayes_n_R3( uh_list, xr, prior )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

%To perform nth order bayes. 
    global ng goal_transition_P;
    n = size(uh_list, 2); % > 1
    ll = ones(ng, 1);
    for i=n:-1:1
        ll = ll.*likelihood_func(uh_list(:, i), xr);
    end
    prior = prior + 0.0*rand(ng, 1); %to avoid collapse of posterior. 
    prior = prior/sum(prior);
    posterior = ll.*sum(goal_transition_P.*prior)';
    posterior = posterior/sum(posterior); %make sure that the posterior is always normalized. 
end

function likelihood = likelihood_func(uh, xr)
    global ng xg;
    likelihood = zeros(ng, 1);
    if norm(uh)~=0
        uh = uh/norm(uh);
    else
        likelihood = ones(ng, 1);
        return;
    end
    for i=1:ng
        dir_vec = xg(:, i) - xr; %vector joining the robot position and the goal_x. 
        dir_vec = dir_vec/norm(dir_vec); %as long as xr is not xg, this would be nonzero;
        likelihood(i) = p_of_u_given_g(uh, dir_vec);
    end
end

function [q] = p_of_u_given_g(uh, dir_vec) %assumes that the uh that is expected to see for a goal is the one which is directly pointing at it. 
%     q = prob(uh(1) - dir_vec(1))*prob(uh(2) - dir_vec(2))*prob(uh(3) - dir_vec(3)); %p(ux, uy, uz) = p(ux)p(uy)p(uz);
    ang = acos(round(dot(uh, dir_vec), 5));
    q = prob(ang);
end

function p = prob(a) %zero mean gaussian, 
    sigma = pi/2; %for the time being use this std dev
    p = (1/sqrt(2*pi*sigma^2))*exp((-1/2.0)*(a^2/sigma^2));
end