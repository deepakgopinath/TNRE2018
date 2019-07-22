function [cdadir, gamma,cutpoint] = cda(X,Y,paramstruct)
% CDA: (Continuum Discriminant Analysis) For two groups, this code returns
% cdadir (set of directions perpendicular to the separating hyperplain)
% corresponding to a grid of size ngrid = 100.
% [cdadir, gamma, cutpoint] = cda(X,Y)
%       X : n x d (or d x n) matrix of data
%       Y : n x 1 (or 1 x n) vector of class labels.
% The output cdadir is a matrix of direction vectors that correspond to
% values of gamma, and also an array of cutpoints (where a projection of x
% onto cdadir is larger than this cutpoint, then x is classified to group 1)
%
% For more general CDA, use paramstruct.
% [cdadir, gamma, cutpoint] = cda(X,Y,paramstruct)
%
%       paramstruct - a Matlab structure of input parameters
%                    Use: "help struct" and "help datatypes" to
%                         learn about these.
%                    Create one, using commands of the form:
%
%       paramstruct = struct('field1',values1, ...
%                            'field2',values2, ...
%                            'field3',values3) ;
%
%                          where any of the following can be used,
%                          these are optional, misspecified values
%                          revert to defaults
%
%                    Version for easy copying and modification:
%
%       paramstruct = struct('ngrid',0,...
%                            'BalancedDesign',0)
%
%    fields            values
%    ngrid             (default = 100) number of grid for tuning parameter
%                       gamma
%    range             [l u] (range of gamma, default governed by alpha values)
%    BalancedDesign    0 or 1 (default = 0) When '1', assume balanced
%                          design, i.e. all subgroup sizes are equal.
%
% Sungkyu Jung. 2012.
%
% See also lda
parameters = struct('BalancedDesign',0);

ngrid = 100;

if nargin > 2;
    if isfield(paramstruct,'ngrid');
        ngrid = paramstruct.ngrid;
    end
    
    if isfield(paramstruct,'BalancedDesign') ;    %  then change to input value
        parameters.BalancedDesign = paramstruct.BalancedDesign;
    end ;
    if isfield(paramstruct,'range') ;    %  then change to input value
        parameters.range = paramstruct.range;
    end ;
end

[N d]=size(X);
n = length(Y);
if N ~= n
    X = X';
    [N d]=size(X);
end
if N ~=n
    disp('cda.m: inconsistent labeling. Check your input!');
    return
end
label = unique(Y);
K = length(label);

nvec = zeros(K,1); % K-vector of class sizes
muvec = zeros(d,K); % each column is the class-wise mean

if iscell(Y)
    for k = 1:K;
        nvec(k) = sum(strcmp(Y,label(k)));
        muvec(:,k) = mean(X(strcmp(Y,label(k)),:));
    end
else
    for k = 1:K;
        nvec(k) = sum(Y==label(k));
        muvec(:,k) = mean(X(Y==label(k),:));
    end
end

if K > 2;
    disp('Warning: Current version of cda.m only supports two groups.');
    disp('Warning: Restrict classes to the first two groups');
    N = nvec(1) + nvec(2);
    X = X(:,1:N);
end

mu = mean(X);
Xtilde =  X - repmat(mu,N,1);
[U Lambda V]=svd(Xtilde',0);
Xcanon = Lambda * V;
Xcanon = Xcanon(1:end-1,:);
lambda = diag(Lambda); lambda = lambda(1:end-1);
U = U(:,1:end-1);

dd = (muvec(:,1) - muvec(:,2));
delta = U' * dd;

SB =  (nvec(1)*nvec(2)) / N^2 * delta * delta' ;
ST = Xcanon * Xcanon' / N;

d0 = length(delta);
evalues = lambda.^2/n;



% Ridge Solutions
if isfield(parameters,'range');
    range = parameters.range;
    % Given range, find appr. corresponding alphas;
    alpha1 = range(1) / (1-range(1)) * (delta'*ST*delta) / (delta'*delta);
    alpha2 = range(2) / (1-range(2)) * (delta'*ST*delta) / (delta'*delta);
    alphavec = linspace(alpha1,alpha2,ngrid);
    
    wvec = zeros(d0,ngrid);
    gammavec = zeros(1,ngrid);
    for ialpha = 1:ngrid
        alpha = alphavec(ialpha);
        w = (ST + alpha* eye(d0))\ delta;
        if sign(alpha)==0
            w = w/norm(w);
        else
            w = sign(alpha)*w/norm(w);
        end
        wvec(:,ialpha) = w;
        gammavec(ialpha) = alpha/ (w'*ST*w + alpha);
    end
    % mean difference direction
    md = delta / norm(delta);
    
    % combine CDA directions including extreme cases (fd, md, pc)
    cdadir = U*wvec(:,1:ngrid);
    gamma  = gammavec(1:ngrid);
    
    
else
    alphalim = evalues(1)*10;
    %atemp = linspace(0,evalues(1),ceil(ngrid/2));
    %atemp2 = linspace(evalues(1),alphalim, 1+ngrid-ceil(ngrid/2));    
    alphavec = [linspace(0,alphalim,ngrid) linspace(-alphalim, -evalues(1)*1.01, ngrid)];
    
    wvec = zeros(d0,2*ngrid);
    gammavec = zeros(1,2*ngrid);
    for ialpha = 1:2*ngrid
        alpha = alphavec(ialpha);
        w = (ST + alpha* eye(d0))\ delta;
        if ialpha > ngrid  % then (ST + alpha* eye(d0)) is negative definite
            w = -w/norm(w);
        else           % then (ST + alpha* eye(d0)) is positive definite
            w = w/norm(w);
        end
        wvec(:,ialpha) = w;
        gammavec(ialpha) = alpha/ (w'*ST*w + alpha);
    end
    
    
    % mean difference direction
    md = delta / norm(delta);
    
    
    if abs(delta(1)) > 1e-5
        % combine CDA directions including extreme cases (fd, md, pc)
        cdadir = [U*wvec(:,1:ngrid) U*md U*wvec(:,(ngrid+1):end) U(:,1)];
        gamma  = [gammavec(1:ngrid) 1 gammavec((ngrid+1):end) inf];
        
    else
        disp('Message from cda.m: Mean Difference is orthogonal to the first e-vector of Total Variance');
        
        Lambda = diag(ST);
        Lambda2ind = Lambda < Lambda(1);
        d02 = sum(Lambda2ind);
        gammacondition = sum((Lambda(1) * delta(Lambda2ind).^2) ./ ((Lambda(1) - Lambda(Lambda2ind)).^2)) ...
            / sum( delta(Lambda2ind).^2 ./  (Lambda(1) - Lambda(Lambda2ind)) )  ;
        egammavec = linspace(gammacondition,10*gammacondition,ngrid);
        ewvec = zeros(d0,ngrid);
        cfactor = (delta(Lambda2ind) ./  (Lambda(1) - Lambda(Lambda2ind)) ) ...
            / sqrt( sum( delta(Lambda2ind).^2 ./  (Lambda(1) - Lambda(Lambda2ind)) )) ;
        
        for igamma = 1:ngrid
            gamma = egammavec(igamma);
            %z1 = (d0 - d02);
            z2 = sqrt(Lambda(1)/gamma)* cfactor;
            z1 = ones(d0-d02,1)*sqrt((1- z2'*z2)/(d0-d02));
            ewvec(:,igamma) = [z1; z2];
        end
        
        
        % combine CDA directions including extreme cases (fd, md, pc)
        cdadir = [U*wvec(:,1:ngrid) U*md U*wvec(:,(ngrid+1):end) U*ewvec U(:,1)];
        gamma  = [gammavec(1:ngrid) 1 gammavec((ngrid+1):end) egammavec inf];
        
        
    end
    
    
end


if parameters.BalancedDesign == 0;
    cutpoint = mu* cdadir - log(nvec(1)/nvec(2));
else
    cutpoint = mu* cdadir;
end

