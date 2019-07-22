function [delta] = qda(X,Y,L2penalty)
% qda: (Quadratic Discriminant Analysis) For any number of groups, returns
% quadratic discriminant functions (QDA) deltas. A new observation Z (d x
% 1) is classified to class k if delta{k}(Z) >= delta{j}(Z) for all j = 1,...,K.
% [delta] = qda(X,Y)
%       X : n x d (or d x n) matrix of data
%       Y : n x 1 (or 1 x n) vector of class labels.
% [delta] = qda(X,Y,L2penalty)
%       L2 penalty : default 0
% 
% Sungkyu Jung. 2012.
%
% See also qdarun,lda,ldarun,
 
if nargin == 2; 
    L2penalty = 0; 
end

[N d]=size(X);
n = length(Y);
if N ~= n
    X = X';
    [N d]=size(X);
end
if N ~=n
    disp('lda.m: inconsistent labeling. Check your input!');
    return
end
label = unique(Y);
K = length(label);

nvec = zeros(K,1); % K-vector of class sizes
muvec = zeros(d,K); % each column is the class-wise mean
Sigmaarry = zeros(d,d,K); % covariance matrix for each class
sumlogevalue = zeros(K,1);
for k = 1:K
    nvec(k) = sum(Y==label(k));
    muvec(:,k) = mean(X(Y==label(k),:));
    Sigmaarry(:,:,k) = cov(X(Y==label(k),:))+L2penalty*eye(d);
    sumlogevalue(k)=sum(log(eig(Sigmaarry(:,:,k))));
    if isinf(sumlogevalue(k))
        disp('Warning: Nonsingular covariance matrix: increase the sample size or add L2penalty');
        disp('       : Adding 1e-5 for numerical stability')
        Sigmaarry(:,:,k) = Sigmaarry(:,:,k) + (1e-5) * eye(d);
        sumlogevalue(k)=sum(log(eig(Sigmaarry(:,:,k))));
    end
end

delta = cell(K,1);
for k = 1:K
delta{k} = @(x) -1/2*sumlogevalue(k) ...
    - 1/2*(x-muvec(:,k))' * (Sigmaarry(:,:,k) \  (x-muvec(:,k)) )...
    + log(nvec(k)/N);
end

end
