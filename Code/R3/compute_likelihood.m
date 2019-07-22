function ll = compute_likelihood(running_cost, optimal_cost_to_go, optimal_cost_from_start)
    running_cost = repmat(running_cost, size(optimal_cost_to_go, 1), 1); %vector now for ease of calculation
    ll = exp(-running_cost - optimal_cost_to_go)./exp(-optimal_cost_from_start);
end