function [ sample ] = sample_triangular_distribution( variance )
%SAMPL_TRIANGULAR_DISTRIBUTION Summary of this function goes here
%   Detailed explanation goes here

sample = variance*random('unif', -1, 1)*random('unif', -1, 1);

end

