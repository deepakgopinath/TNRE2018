function [ kl_div ] = compute_kl_div( pg_projected, pg_baseline ) %p i
%COMPUTE_KL_DIV Summary of this function goes here
%   Detailed explanation goes here
kl_div = -nansum(pg_projected.*log2( (pg_baseline./pg_projected) + realmin));
end

