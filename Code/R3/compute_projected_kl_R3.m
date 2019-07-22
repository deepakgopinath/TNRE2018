function [ Upsilon ] = compute_projected_kl_R3( uh, varargin )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
projection_time = 1;
projected_pg = compute_pg_projection_R3(uh, projection_time, varargin{:});
Upsilon = compute_kl_div(projected_pg, varargin{3});

end

