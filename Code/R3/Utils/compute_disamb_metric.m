function [ Dk ] = compute_disamb_metric( pg1, pg2 )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

w = 0.5;
pg1_sorted = sort(pg1, 'ascend');  %sort for easy access of second component

%Short-term contribution
Gamma = max(pg1);
Omega = pg1_sorted(end) - pg1_sorted(end-1); %already sorted. 
Lambda = sum(pdist(pg1));

%longterm contribution
Upsilon = sum(pdist(pg2-pg1));

Dk = w*Gamma*Omega*Lambda + (1-w)*Upsilon;
end

