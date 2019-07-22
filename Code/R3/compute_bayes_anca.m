function [ ll, posterior ] = compute_bayes_anca( curr_pos, next_pos, prior, exp_factor )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    global xg running_cost optimal_cost_from_start goal_transition_P;
    prior = prior/sum(prior);
    optimal_cost_to_go = cost_optimal_traj(curr_pos, xg);
    running_cost = running_cost + sum((next_pos - curr_pos).^2);
    ll = compute_likelihood(running_cost, optimal_cost_to_go, optimal_cost_from_start);
    posterior = ll.^(exp_factor).*goal_transition_P'*prior;
    posterior = posterior/sum(posterior); 
end

