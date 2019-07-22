function [ldadir, cutpoint] = lda(X,Y,L2penalty,L1penalty,paramstruct)
% lda: (Linear Discriminant Analysis) For two groups, this code returns
% ldadir (the direction perpendicular to the separating hyperplain) and
% cutpoint (where a projection of x onto ldadir is larger than this
% cutpoint, then x is classified to group 1). For K groups, this
% code does the reduced-rank LDA, and returns a canonical basis of
% dimension < K.
% [ldadir, cutpoint] = lda(X,Y)
%       X : n x d (or d x n) matrix of data
%       Y : n x 1 (or 1 x n) vector of class labels.
% [ldadir, cutpoint] = lda(X,Y,L2penalty,L1penalty)
%       L2 penalty : default 0
%       L1 penalty : default 0
% L2penalty = theta > 0 shrinks the within-covariance toward theta*eye(d)
% L1penalty = 1 gives the entire piecewise linear solution of SLDA by Wu et
% al. (2009), and return ldadir.W and ldadir.BIC.
%
% For more general LDA, use paramstruct.
% [ldadir, cutpoint] = lda(X,Y,L2penalty,L1penalty,paramstruct)
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
%
% See also ldarun, qda, cda, naiveLDA
parameters = struct('AdaptiveLasso',0,...
    'BalancedDesign',0);

if nargin == 2;
    L2penalty = 0;
    L1penalty = 0;
end

if nargin == 3;
    L1penalty = 0;
end

if nargin > 4;
    if isfield(paramstruct,'AdaptiveLasso') ;    %  then change to input value
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
    disp('lda.m: inconsistent labeling. Check your input!');
    return
end
label = unique(Y);
K = length(label);

nvec = zeros(K,1); % K-vector of class sizes
muvec = zeros(d,K); % each column is the class-wise mean

if K > 2 || L1penalty ~= 0 % Sw will not be used if K == 2 or L1penalty ==0
    Sw = zeros(d,d); %With-in (common) covariance matrix
    
    if iscell(Y)
        for k = 1:K;
            nvec(k) = sum(strcmp(Y,label(k)));
            muvec(:,k) = mean(X(strcmp(Y,label(k)),:));
            Sw = Sw + cov(X(strcmp(Y,label(k)),:))*(nvec(k)-1)/N;
        end
    else
        for k = 1:K;
            nvec(k) = sum(Y==label(k));
            muvec(:,k) = mean(X(Y==label(k),:));
            Sw = Sw + cov(X(Y==label(k),:))*(nvec(k)-1)/N;
        end
    end
else
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
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if K == 2
    if L1penalty == 0
        % A usual LDA uses the following
        % ldadir = Swd\(muvec(:,1)-muvec(:,2));
        % In case of ill-conditioned Swd, we need to seek numerical
        % stability.
        
        mu = mean(X);
        Xtilde =  X - repmat(mu,N,1);
        [U Lambda V]=svd(Xtilde',0);
        Xcanon = Lambda * V;
        SuffDim = min(N-1,d);
        Xcanon = Xcanon(1:SuffDim,:);
        lambda = diag(Lambda); lambda = lambda(1:SuffDim);
        U = U(:,1:SuffDim);
        delta = U' * (muvec(:,1) - muvec(:,2));
        SB =  (nvec(1)*nvec(2)) / N^2 * (delta * delta') ;
        ST = (Xcanon * Xcanon') / N;
        SW = ST - SB;
        ldadir = U * (pinv(SW + L2penalty*eye(SuffDim)) * delta);
        
        if parameters.BalancedDesign == 0;
            cutpoint = mean(muvec,2)'* ldadir - log(nvec(1)/nvec(2));
        else
            cutpoint = mean(muvec,2)'* ldadir;
        end
        
    else % Now follow WZL's sparse LDA
        
        Swd = Sw + L2penalty*eye(d);
        
        if parameters.AdaptiveLasso == 1 % Adjustment for Adaptive Lasso
            wini = Swd\(muvec(:,1)-muvec(:,2));
            wtilde = abs(wini);
            Swd = Swd .* (wtilde*wtilde');
            muvec(:,1) = muvec(:,1) .* wtilde;
            muvec(:,2) = muvec(:,2) .* wtilde;
        end
        
        g = muvec(:,1)-muvec(:,2);
        
        % initialize (1)
        [~,A] = min(diag(Swd)./(g.^2));
        w = zeros(d,1);
        w(A) = 1/g(A);
        
        % initialize (2)
        Saw = Swd(A,A)*w(A); %  Swd(A,:)*w
        v = [2*(Saw - Swd(:,A)*w(A))./(g-g(A)) ;  %Swd(:,A)*w(A) = Swd*w
            -2*(Swd(:,A)*w(A) + Saw)./(g+g(A))];
        [tmp,J]= max(abs(2*Saw + v.*g(A)));
        v = v(J);
        if J>d; J = J-d;end
        A = [A ; J];
        W = w;
        
        % interation 1
        keepgoing = (length(A) ~= d);
        cnt = 2;
        try
            
            while(keepgoing)
                % compute new direction by 8-10;
                Saw = Swd(A,A)*w(A);
                mxi = -sign(2*Saw + v*g(A));
                a = length(A);
                C = zeros(a+2);
                b= zeros(a+2,1);
                b(a+2) = 1;
                
                C(1:a,1:a) = 2*Swd(A,A);
                C(1:a,a+1) = g(A);
                C(1:a,a+2) = mxi;
                C(a+1,1:a) = g(A)';
                C(a+2,1:a) = mxi;
                sol = C\b;
                gamma = zeros(d,1);
                gamma(A) = sol(1:a); % dw/dt
                dv = sol(a+1);    % dv/dt
                
                % find which variable to add
                D = [];
                for aa = 1:a
                    a1 = A(aa);
                    dpos = (2*(Swd*w - repmat(w'*Swd(:,a1),d,1))+v*(g-g(a1))) ./ ...
                        (2*(repmat(gamma'*Swd(:,a1),d,1) - Swd*gamma)+dv*(g(a1)-g));
                    dneg = (2*(Swd*w + repmat(w'*Swd(:,a1),d,1))+v*(g+g(a1))) ./ ...
                        (2*(-repmat(gamma'*Swd(:,a1),d,1) + Swd*gamma)-dv*(g(a1)+g));
                    dpos(A) = -10;
                    dneg(A) = -10;
                    D = [D; dpos; dneg];
                end
                positiveInd = D>0;
                D(~positiveInd) = Inf;
                [d1,addvarind] = min(D);
                %if addvarind <= 2*d;             disp('a1 = 1 is okay');         end
                addvarind = mod(addvarind,d);
                if addvarind == 0;   addvarind = d;   end
                
                % find which variable to remove
                d2 = d1+100;
                D2 = -w(A)./gamma(A);
                positiveInd = D2>0;
                if sum(positiveInd)>0
                    D2(~positiveInd) = Inf;
                    [d2,killvar] = min(D2);
                    killvarind = zeros(a,1); killvarind(killvar) = 1;
                end
                
                % Distance till Stop;
                d3 = (-2*Swd(a1,:)*w - v*g(a1)) / (2*Swd(a1,:)*gamma +dv*g(a1) );
                
                % Choose kill or add;
                if d3> 0 && d3 <= d1 && d3 <= d2
                    dd = d3;
                    keepgoing = false;
                else
                    if d1 < d2
                        dd = d1;
                        A = [A ; addvarind];
                    else
                        dd = d2;
                        A = A(~killvarind);
                    end
                    v = v + dd*dv;
                    w = w+ dd*gamma;
                    W = [W,w];
                    cnt = cnt+1;
                    if cnt > 3*d
                        % disp('Infinite Loop');
                        % Note Wu et al suggested to stop at d^2/2
                        % We stop at 2*d for faster evaluation
                        break;
                    end
                end
            end
        catch err
            dimW = size(W,2);
            if dimW < d
                disp(['Sparse LDA stopped due instable direction, at ' num2str(dimW) 'th iteration']);
            end
            W = W(:,1:end-1);
        end
        
        % BIC calculation
        BIC = zeros(size(W,2),1);
        for r = 1:size(W,2)
            rtau = (d-sum(W(:,r)==0));
            BIC(r) = log(W(:,r)'*Swd*W(:,r)/(N-rtau-1)) + rtau*log(N)/N;
        end
        
        if parameters.AdaptiveLasso == 1 % Adjustment for Adaptive Lasso
            wini = Swd\(muvec(:,1)-muvec(:,2));
            Wtilde = W .* repmat(wtilde,1, size(W,2));
            ldadir.path = Wtilde;
            for k = 1:K
                muvec(:,k) = mean(X(Y==label(k),:));
            end
        end
        
        ldadir.path = W;
        ldadir.BIC = BIC;
        cutpoint = mean(muvec,2)'* ldadir.path;
        
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if K > 2 % For multi-class (not used if K == 2), do reduced-rank LDA
    % through simultaneous diagonalization.
    % Ref: https://forums.cs.tau.ac.il/~nin/Courses/NC07/SVD_EVD.pdf
    % L1penalty is ignored.
    
    Swd = Sw + L2penalty*eye(d);
    Sb = cov(muvec');
    dd = min(rank(Sb),K-1); %% We will reduce the dimension to 'dd'
    [U,D] = eig(Swd);  T1 = U*diag(1./sqrt(diag(D)));
    [T2,Dtmp] = eig(T1'*Sb*T1);
    T = T1*T2;
    ldadir = T(:,1:dd);
end

end
