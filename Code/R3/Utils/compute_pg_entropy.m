function [ Upsilon ] = compute_pg_entropy( pg )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
Upsilon = -nansum(pg.*log2(pg+realmin));
end

