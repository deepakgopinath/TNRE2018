function [delta, mcr] = qdarun(X,Y,R,L2penalty)
% qdarun: (Quadratic Discriminant Analysis)  
% This code runs QDA on a dataset, returning R-fold c.v. miss-classification rate.
% A new observation Z (d x 1) is classified to class k if 
% delta{k}(Z) >= delta{j}(Z) for all j = 1,...,K. 
% On a two dimensional hyperplain given by PC1 + LDAdir, draws graphical 
% results of the classification.
% [delta, mcr] = qdarun(X,Y,R)
%       X : n x d (or d x n) matrix of data
%       Y : n x 1 (or 1 x n) vector of class labels.
%       R : R = 10 for R-fold cross-validation
% [delta, mcr] = qdarun(X,Y,R,L2penalty)
%       L2 penalty : default 0
% 
% Sungkyu Jung. 2012.
%
% See also qdarun,lda,ldarun,
 
if nargin == 2;
    R = 10;
    L2penalty = 0;
end

if nargin == 3; 
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
delta = cell(K,1);
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
    delta{k} = @(x) -1/2*sumlogevalue(k) ...
        - 1/2*(x-muvec(:,k))' * (Sigmaarry(:,:,k) \  (x-muvec(:,k)) )...
        + log(nvec(k)/N);
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
    testY = (Y(testingset==1));
    
    [tdelta] = qda(trainX,trainY,L2penalty)  ;
        for ntest = 1:length(testY);
            testXvec = testX(ntest,:)';
            tdeltaValuevec = zeros(K,1);
            for k = 1:K
                tdeltaValuevec(k) = tdelta{k}(testXvec);
            end
            [tmp,testmaxind]=max(tdeltaValuevec);
            correctClassify = correctClassify + ...
                (testY(ntest) == label(testmaxind));
        end
end
mcr = 1 - correctClassify/N;


%% then return graphics;
    % Two-directions to project: 
    % If K>=3 then use the first two reduced-rank LDA directions.
    LDAdir =  lda(X,Y,L2penalty);
    xdir = LDAdir(:,1);
    xdir = xdir / norm(xdir);
    if size(LDAdir,2) > 1
        ydir = LDAdir(:,2);
        ydir = ydir / norm(ydir);
    else % if not, y-axis is the part of pc1dir, orth. to x
    [u tmp tmp2]=princomp(X);
    pc1dir = u(:,1);
    ydir = pc1dir - (xdir'*pc1dir) * xdir; ydir = ydir / norm(ydir);
    end
    dir = [xdir ydir];   
    % project X onto the 2-plane
    % 
    
    projX = X * dir;
    hold on;
    gscatter(projX(:,1),projX(:,2),Y)
    if K > 2
        xlabel('LDAdir 1');    ylabel('LDAdir 2');
    else
        xlabel('LDAdir');    ylabel('Orth. PC 1');
    end
    xlim = get(gca,'xlim');
    ylim = get(gca,'ylim');
    
%     x = linspace(xlim(1),xlim(2),10);
%     y = linspace(ylim(1),ylim(2),10); 
%     J = zeros(10,10); 
%     for i = 1:10;
%         for j = 1:10;
%             newx = x(i)*xdir + y(j)*ydir;
%             deltaValuevec = zeros(K,1);
%             for k = 1:K
%                 deltaValuevec(k) = delta{k}(newx);
%             end 
%             [tmp,maxind]=max(deltaValuevec)
%             J(i,j) = maxind;
%         end
%     end
%     [x,y] = meshgrid(x,y);            
%     gscatter(x(:),y(:),J(:),'grb','sod',10,'off')
    title(['QDA with l2 = ' num2str(L2penalty) ', MCrate = ' num2str(mcr*100) ' %'])
    

end
