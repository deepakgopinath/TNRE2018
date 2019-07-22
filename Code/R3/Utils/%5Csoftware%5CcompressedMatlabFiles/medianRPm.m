function gm = medianRPm(vdata,Error)
% medianSm  geometric meadian of data on Real Projective Space RP(m) 
%           of dimension m. Modification of Fletcher et al. (2009) method. 
% 
% gm = medianRPm(vdata)
% gm = medianRPm(vdata,Error)
%
% 
% Inputs:   vdata   - a matrix (m+1)-by-n : a column vector represents a 
%                      point on S^m (An element [a] in RP(m) is identified
%                      with two antipodal points (a,-a) in S^m. The columns 
%                      of input matrix vdata can be either a or -a.
%           Error   - a scalar for optimization 
%                 
% Output:
%           gm    - geometric median on RP(m)
% 
% Note: 
% - Small, C.G. (1990). A survey of multidimensional medians. 
%   International Statistical Review, 58, 263--277. 
% - Robust principal component analysis for functional data
%   N. Locantore, J. S. Marron, D. G. Simpson, N. Tripoli, J. T. Zhang, 
%   K. L. Cohen (1990), TEST, Volume 8, Number 1, 1-73.
%   
%   In Euclidean space this is the geometric median, and the algorithm 
%   illustrated in Locantore et al. is in fact an representation of
%   gradient descent algorithm for the geometric median ($\argmin_x\sum \|x -x_i \|_2$). 
%   An algorithm for geometric median on manifold is available at
% - Fletcher PT, Venkatasubramanian S, Joshi S.,
%   The geometric median on Riemannian manifolds with application to robust atlas estimation.
%   Neuroimage. 2009 Mar;45(1 Suppl):S143-52. Epub 2008 Nov 13.)
%   
% Additional notes:
%   Distance function in RP(m) is simply defined as 
%          d([x],[v]) = min{ d(x,v), d(-x,v) } where d is the geodesic
%          distance on S^m.
% 9/20/2011
% Sungkyu Jung

if nargin == 1
    Error = 1e-10;
end

[m n] = size(vdata);

vini = vdata(:,1);
% Initial candidate for geodesic mean
diff = 1;
while diff > Error 
    rot = rotMat(vini);
    rvdata = rot*vdata;
    signvec = rvdata(end,:)<0;
    rvdata(:,signvec) = -1*rvdata(:,signvec); 
    
    logvdata = LogNPd(rvdata);
    % Log mapped points at vini 
    dist = sqrt(sum(logvdata.^2));
    % distance of points from vini
    
    id = dist > 1e-10;
    % indicates that the distance is positive,
    % if the distance is zero, then it is excluded from computation
    
    if all(id==0)
        if diff==1; vnew=vini; end
        diff = 1e-11;
    else
        logvnew = sum(logvdata(:,id).*repmat(1./dist(id),m-1,1),2)/sum(1./dist(id));
        vnew = rot\ExpNPd(logvnew) ;
        diff = norm(vnew - vini);
        % see the difference
        vini = vnew;
    end
end;

gm = vnew;

end


function rot = rotMat(b,a,alpha)
% ROTMAT returns a rotation matrix that rotates unit vector b to a
%
%   rot = rotMat(b) returns a d x d rotation matrix that rotate
%   unit vector b to the north pole (0,0,...,0,1)
%
%   rot = rotMat(b,a ) returns a d x d rotation matrix that rotate
%   unit vector b to a
%
%   rot = rotMat(b,a,alpha) returns a d x d rotation matrix that rotate
%   unit vector b towards a by alpha (in radian)
%
%    See also .

% Last updated Nov 7, 2009
% Sungkyu Jung


[s1 s2]=size(b);
d = max(s1,s2);
b= b/norm(b);
if min(s1,s2) ~= 1 || nargin==0 , help rotMat, return, end  

if s1<=s2;    b = b'; end

if nargin == 1;
    a = [zeros(d-1,1); 1];
    alpha = acos(a'*b);
end

if nargin == 2;
    alpha = acos(a'*b);
end
if abs(a'*b - 1) < 1e-15; rot = eye(d); return, end
if abs(a'*b + 1) < 1e-15; rot = -eye(d); return, end

c = b - a * (a'*b); c = c / norm(c);
A = a*c' - c*a' ;

rot = eye(d) + sin(alpha)*A + (cos(alpha) - 1)*(a*a' +c*c');
end

function Exppx = ExpNPd(x)
% EXPNP Riemannian exponential map at North pole of S^k
%       ExpNP(v) returns (k+1) x n matrix where each column is a point on a 
%                sphere and the input v is k x n matrix where each column 
%                is a point on tangent  space at north pole.
%
%
%   See also LogNPd.

% Last updated Oct 20, 2009
% Sungkyu Jung

[d n] = size(x);
nv = sqrt(sum(x.^2));
Exppx = [repmat(sin(nv)./nv,d,1).*x ;cos(nv)];
Exppx(:,nv < 1e-16) = repmat([zeros(d,1);1],1,sum(nv < 1e-16));
end

function Logpx = LogNPd(x)
% LOGNP Riemannian log map at North pole of S^k
%       LogNP(x) returns k x n matrix where each column is a point on tangent
%       space at north pole and the input x is (k+1) x n matrix where each column 
%       is a point on a sphere.
%
%
%   See also ExpNPd.

% Last updated Oct 20, 2009
% Sungkyu Jung

[d n] = size(x);
scale = acos(x(end,:))./sqrt(1-x(end,:).^2);
scale(isnan(scale)) =1;
Logpx = repmat(scale,d-1,1).*x(1:(end-1),:);
end