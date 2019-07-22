clc; clear all;close all;
%%
N = 20000;
samples = zeros(N, 1);
for i=1:N
    samples(i) = sample_triangular_distribution(pi);
end
%%
histogram(samples)