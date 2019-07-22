function [ cost_optimal ] = cost_optimal_traj( curr_pos, goal_pos )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

ng = size(goal_pos, 2);
curr_pos = repmat(curr_pos, 1, ng);
cost_optimal = (sum((curr_pos - goal_pos).^2, 1))';

end

