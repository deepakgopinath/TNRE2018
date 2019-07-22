function [ Upsilon ] = compute_projected_entropy_R3( uh, varargin )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
projection_time = 1;
projected_pg = compute_pg_projection_R3(uh, projection_time, varargin{:});
Upsilon = compute_pg_entropy(projected_pg);
end

