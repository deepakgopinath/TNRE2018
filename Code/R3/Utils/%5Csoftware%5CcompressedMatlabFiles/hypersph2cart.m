function C = hypersph2cart(x)
% cart2hypersph  Cartesian coordinates to hyperspherical coordinates
% C = cart2hypersph(x), with d x n matrix x of spherical coordinates, returns
%       Cartesian coordinates C. First row of x is the radial coordinate, second
%       to last coordinates are the angular components.
%
% % Example:
% cart = randn(3,5);
% x = cart2hypersph(cart);
% C = hypersph2cart(x)
% % C should be the same as cart.
%
% This code is based on
% http://en.wikipedia.org/wiki/Hypersphere#Hyperspherical_coordinates
% looked up on 2011/4/21
% see also hypersph2cart.pdf
%
% S. Jung 2011/4/21

if nargin==0, help hypersph2cart, return, end     %   Display help

[d n] = size(x);

r = x(1,:); % radial coordinate.
C = zeros(d,n);
sph = x(2:d,:);
premult = r;
for i=1:(d-1)
    C(i,:) = premult .* cos(sph(i,:));
    premult = premult.* sin(sph(i,:));
end
C(d,:) = premult;


