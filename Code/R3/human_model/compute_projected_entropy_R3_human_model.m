    function [ Upsilon ] = compute_projected_entropy_R3_human_model( curr_mode, curr_goal, varargin )
%COMPUTE_PROJECTED_ENTROPY_R2_FULL_MODEL Summary of this function goes here
%   Detailed explanation goes here

global projection_time;
projected_pg = compute_pg_projection_R3_human_model(curr_mode, curr_goal, projection_time, varargin{:});
Upsilon = compute_pg_entropy(projected_pg);
end

