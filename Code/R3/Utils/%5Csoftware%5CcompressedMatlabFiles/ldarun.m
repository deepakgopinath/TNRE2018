function [ldadir,cutpoint, mcr] = ldarun(X,Y,R,L2penalty,L1penalty,paramstruct)
% ldarun: (Linear Discriminant Analysis)
% For two groups, this code runs LDA on a dataset, returning R-fold c.v.
% miss-classification rate.
%
% returning ldadir (the direction perpendicular to the separating hyperplain) and
% cutpoint (where a projection of x onto ldadir is larger than this
% cutpoint, then x is classified to group 1), and mcr (missclassification rate).
%
% [ldadir, cutpoint, mcr] = ldarun(X,Y,R)
%       X : n x d (or d x n) matrix of data
%       Y : n x 1 (or 1 x n) vector of class labels.
%       R : R = 10 for R-fold cross-validation
% [ldadir, cutpoint, mcr] = ldarun(X,Y,R,L2penalty,L1penalty)
%       L2 penalty : default 0
%       L1 penalty : default 0 (not yet implemented)
% L2penalty = theta > 0 shrinks the within-covariance toward theta*eye(d)
% L1penalty = 1 gives the entire piecewise linear solution of SLDA by Wu et
% al. (2009)
%
% For more general LDA, use paramstruct.
% [ldadir, cutpoint] = lda(X,Y,R,L2penalty,L1penalty,paramstruct)
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
%    paramstruct = struct('AdaptiveLasso',0,...
%                         'BalancedDesign',0,...
%                         '',)
%
%    fields            values
%    AdaptiveLasso     0 or 1 (default 0) When L1penalty > 0, turning on
%                         'AdaptiveLasso' gives the adaptive solution path.
%                         When L1penalty = 0, and AdaptiveLasso = 1, then
%                         no L1 penalty is inforced, and set AdaptiveLasso
%                         = 0.
%    BalancedDesign    0 or 1 (default = 0) When '1', assume balanced
%                          design, i.e. all subgroup sizes are equal.
%
% Sungkyu Jung. 2012.

% See also: lda


parameters = struct('AdaptiveLasso',0,...
    'BalancedDesign',0);
 


if nargin == 2;
    R = 10;
    L2penalty = 0;
    L1penalty = 0;
end

if nargin == 3;
    L2penalty = 0;
    L1penalty = 0;
end

if nargin == 4;
    L1penalty = 0;
end

if nargin > 5;
    if isfield(paramstruct,'BalancedDesign') ;    %  then change to input value
        parameters.AdaptiveLasso = paramstruct.AdaptiveLasso;
    end ;
    if isfield(paramstruct,'BalancedDesign') ;    %  then change to input value
        parameters.BalancedDesign = paramstruct.BalancedDesign;
    end ;
end


[N d]=size(X);
n = length(Y);
if N ~= n
    X = X';
    [N d]=size(X);
end
if N ~=n
    disp('ldarun.m: inconsistent labeling. Check your input!');
    return
end

label = unique(Y);
K = min(length(label),2);
nvec = zeros(K,1); % K-vector of class sizes
for k = 1:K
    nvec(k) = sum(Y==label(k));
end


[ldadir, cutpoint]=lda(X,Y,L2penalty,L1penalty,parameters);

if L1penalty ~= 0;
    [minBIC minBICind] = min(ldadir.BIC);
    ldadir = ldadir.path(:,minBICind);
    cutpoint = cutpoint(minBICind);
end
%% Now do R-fold cross validation
[~,rind] = sort(rand(N,1));
correctClassify = 0;
for r = 1:R;
    
    rid = (1:floor(N/R))+(r-1)*floor(N/R);
    if r == R ;
        rid = ((r-1)*floor(N/R)+1):N;
    end
    
    trainingset = ones(N,1);    trainingset(rind(rid)) = 0;
    testingset = zeros(N,1);    testingset(rind(rid)) = 1;
    
    trainX = X(trainingset==1,:);
    trainY = Y(trainingset==1);
    testX = X(testingset==1,:);
    testY = (Y(testingset==1)==label(1)) - 0.5;
    
    [tldadir, tcutpoint]=lda(trainX,trainY,L2penalty,L1penalty,parameters);
    if L1penalty ~= 0;
        [minBIC minBICind] = min(tldadir.BIC);
        tldadir = tldadir.path(:,minBICind);
        tcutpoint = tcutpoint(minBICind);
    end
    correctClassify = correctClassify + ...
        sum(((testX*tldadir - tcutpoint) .* testY) > 0);
end
mcr = 1 - correctClassify/N;


%% then return graphics;
    Xp = X*ldadir;
    Xgroup1 = Xp(Y == label(1));
    Xgroup2 = Xp(Y == label(2));
    
    [kde1 xgrid1]=kdeSIMPLE(Xgroup1');
    kde1 = kde1*nvec(1)/N*2;
    heights = linspace(max(kde1),2*max(kde1),nvec(1));
    plot(xgrid1, kde1,'r'); hold on;
    scatter(Xgroup1,heights,5,'r');
    text(mean(Xgroup1),max(kde1),'Group 1');
    [kde2 xgrid2]=kdeSIMPLE(Xgroup2');
    kde2 = kde2*nvec(2)/N*2;
    heights = linspace(max(kde2),2*max(kde2),nvec(2));
    plot(xgrid2, kde2,'b'); hold on;
    scatter(Xgroup2,heights,5,'b');
    text(mean(Xgroup2),max(kde2),'Group 2');
    plot([cutpoint cutpoint],[0,1.5*max(kde1)],'k')
    text(cutpoint,1.5*max(kde1),['cutpoint, mcr = ' num2str(mcr*100) '%']);
    xlabel('v_{LDA}');
    title(['LDA with l2 : ' num2str(L2penalty) ', l1 : ' num2str(L1penalty) ])
    


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if K > 2
    disp('ldarun.m. K > 2 not allowed');
    return;
end


end



%% simple kde with an input as row vector.
function [kdeh xgrid]=kdeSIMPLE(rowvec)
%rowvec = imat(1,:);
n = length(rowvec);
xgrid = linspace(min(rowvec),max(rowvec),201);
nbin = length(xgrid);
dsd = std(rowvec) ; diqr = iqr(rowvec) ;
a = min([dsd; (diqr / 1.34)]) ; %  Using Silverman's notation
h = .9 * a * n^(-1/5) ; % Silverman's eqn. 3.31 'rule of thumb'


kdeh = repmat((rowvec ./ h),nbin,1) - repmat((xgrid' ./ h),1,n) ;
%  efficient way to divide all dif's by h
%  variable name "kde" is used to avoid creating too many biggies
kdeh = exp(-(kdeh .^2) / 2) ;
%  exponential part of Gaussian density
kdeh = sum(kdeh,2)' ;
%  sum part of kde, and make result a column vector
kdeh = kdeh / (n * h * sqrt(2 * pi)) ;
%  normalize, and mult by Gaussain density constant

end