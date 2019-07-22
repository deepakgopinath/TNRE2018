function x = cart2hypersph(cart)
% cart2hypersph  Cartesian coordinates to hyperspherical coordinates
% x = cart2hypersph(cart), with d x n matrix cart, each column being a
%       d-dim. points. Each column of the output x = [r, t_1, ... t_{d-1}],
%       where the radial coordinate 'r' is in R+ and the angular coordinate
%       t_{d-1} ranges over [-pi, pi) while all the other angular
%       coordinates t_1,...t_{d-2} range over [0, pi).
%
% This code is based on
% http://en.wikipedia.org/wiki/Hypersphere#Hyperspherical_coordinates
% looked up on 2011/4/21
% see also hypersph2cart.pdf
%
% S. Jung 2011/4/21

if nargin==0, help cart2hypersph, return, end     %   Display help

[d n] = size(cart);
r = sqrt(sum(cart.^2)); % radial coordinate 

angularcomp = zeros(d-1,n); 
 
for i = 1:(d-2)
    partialr = sqrt(sum(cart(i+1:end,:).^2,1));
    sph = atan2(partialr,cart(i,:));
    angularcomp(i,:) = sph;
end
angularcomp(d-1,:) = atan2(cart(d,:),cart(d-1,:));

x = [r ; angularcomp];


