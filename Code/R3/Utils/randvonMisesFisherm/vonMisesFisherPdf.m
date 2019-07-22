function [ pdf ] = vonMisesFisherPdf( x, mu, kappa )
%VONMISESFISHERPDF Summary of this function goes here
%   Detailed explanation goes here

if size(x, 2) > 1
    x = x'; %make it into column vector
end

if norm(mu) ~= 1.0
    mu = mu./norm(mu);
end
p = size(x, 1); %dimensionality of the vector
x = x./norm(x);

C = kappa^(p/2.0 -1) * ((2*pi)^(p/2.0))^(-1) * (besseli(p/2.0 -1, kappa))^(-1);
pdf = C * exp(kappa * dot(mu, x));

end