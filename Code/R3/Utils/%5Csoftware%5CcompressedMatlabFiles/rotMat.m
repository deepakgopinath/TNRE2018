function rot = rotMat(b,a,alpha)
% ROTMAT returns a rotation matrix that rotates unit vector b to a
%
%   rot = rotMat(b) returns a d x d rotation matrix that rotate
%   unit vector b to the north pole (0,0,...,0,1)
%
%   rot = rotMat(b,a) returns a d x d rotation matrix that rotate
%   unit vector b to a
%
%   rot = rotMat(b,a,alpha) returns a d x d rotation matrix that rotate
%   unit vector b towards a by alpha (in radian)
%
%   rot = rotMat(u,theta) returns 3 x 3 rotation matrix with axis u and
%   angle theta (for u in 3-space).
%
%    See also .
% Last updated 10/27/2011
% Sungkyu Jung


[s1 s2]=size(b);
d = max(s1,s2);
b= b/norm(b);
if min(s1,s2) ~= 1 || nargin==0 , help rotMat, return, end

if s1<=s2;    b = b'; end
iaxisangle = 0; % default is rotation in higher dimension d >=2

if nargin == 2;
    if length(a) == 1; % then axis-angle representation
        u = b;
        theta = a;
        iaxisangle = 1;
    else
        alpha = acos(a'*b);
    end
end

if iaxisangle
    ucross = [ 0 -u(3) u(2);
        u(3) 0 -u(1);
        -u(2) u(1) 0];
    rot = eye(3) + sin(theta)*ucross + (1-cos(theta)) * ucross^2;
else
    if nargin == 1;
        a = [zeros(d-1,1); 1];
        alpha = acos(a'*b);
    end
    
    if abs(a'*b - 1) < 1e-15; rot = eye(d); return, end
    if abs(a'*b + 1) < 1e-15; rot = -eye(d); return, end
    
    c = b - a * (a'*b); c = c / norm(c);
    A = a*c' - c*a' ;
    
    rot = eye(d) + sin(alpha)*A + (cos(alpha) - 1)*(a*a' +c*c');
end



