function [cdadir, gamma,cutpoint, gammaindex, mcr] = cdacv(X,Y,R,paramstruct)
% CDACV: (Cross-Validated Continuum Discriminant Analysis) For two groups, 
% this code returns cdadir (set of directions perpendicular to the 
% separating hyperplain) corresponding to a grid of size ngrid (= 100) and 
% also returns the cross-validated choice of gamma.
% [cdadir, gamma,cutpoint, gammaindex, CV] = cdacv(X,Y,R)
%       X : n x d (or d x n) matrix of data
%       Y : n x 1 (or 1 x n) vector of class labels.
%       R (default = 10) R-fold cross validation will be performed
%
% The output 'cdadir' is a matrix of direction vectors that correspond to 
% values of 'gamma', and also an array of 'cutpoints' (where a projection of x 
% onto ldadir is larger than this cutpoint, then x is classified to group 1)
% 'gammaindex' is the index of gamma that gives the smallest CV
% missclassification rate.
% 'mcr' returns CV missclassification rate for various gammas
%
% For more general CDA, use paramstruct.
% [cdadir, gamma,cutpoint, gammaindex, CV] = cdacv(X,Y,R,paramstruct)
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
%    igraphics         (default = false) when 'true', returns a graphic.
%
% Sungkyu Jung. 2012.
%
% See also cda,lda


if nargin  == 2
    R = 10;
end

parameters = struct('BalancedDesign',0);
ngrid = 100;
parameters.ngid = ngrid;
parameters.igraphics = false;

if nargin > 3;
    if isfield(paramstruct,'ngrid');
        ngrid = paramstruct.ngrid;
    end
    
    if isfield(paramstruct,'BalancedDesign') ;    %  then change to input value
        parameters.BalancedDesign = paramstruct.BalancedDesign;
    end ;
    
    if isfield(paramstruct,'igraphics');
        parameters.igraphics = paramstruct.igraphics;
    end
    
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

% First run CDA with all dataset
[cdadir, gamma, cutpoint] = cda(X,Y,parameters);
gammaLength = length(gamma);


label = unique(Y);
if iscell(Y)
    Y = strcmp(Y,label(1));
    label = unique(Y); 
end

K = length(label);
nvec = zeros(K,1); % K-vector of class sizes 



    for k = 1:K;
        nvec(k) = sum(Y==label(k)); 
    end 
    
if K > 2;
    disp('Warning: Current version of cdacv.m only supports two groups.');
    disp('Warning: Restrict classes to the first two groups');
    N = nvec(1) + nvec(2);
    X = X(:,1:N);
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
    
    [tcdadir, tgamma, tcutpoint]=cda(trainX,trainY,parameters);
    correctClassify = correctClassify + ...
        sum(((testX*tcdadir - repmat(tcutpoint,length(testY),1)) .* repmat(testY,1,gammaLength)) > 0,1);
end

mcr = 1 - correctClassify./N;

[minmcr, gammaindex] = min(mcr);

if parameters.igraphics  % then return graphics on current Figure;
    Xp = X*cdadir(:,gammaindex);
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
    plot([cutpoint(gammaindex) cutpoint(gammaindex)],[0,1.5*max(kde1)],'k')
    text(cutpoint(gammaindex),1.5*max(kde1),['cutpoint, mcr = ' num2str(minmcr*100) '%']);
    xlabel('v_{CDA}');
    title(['CDA, Cross Validation '])
end

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