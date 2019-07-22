function C = clustRPm(A,K)%
% CLUSTRPM give K-medois clustering for the points on RP(m)
% 
% C = clustRPm(A)    automatically finds the number of cluster
%                     (computer-intensive) - not yet implemented
% C = clustRPm(A,K)  finds K cluster
%                  
% input A: (m+1) x n matrix, whose column is an element of RP(m) but is 
%  represented by a direction vector in (m+1) dim'l space.
% output C: cluster index (n x 1) vector having values 1,...,K
%
% The algorithm is different than a usual one: There is a couple tweak to
% speed up the computation
% 
% Sungkyu Jung 9/21/2011

[d n]=size(A);
% n  = number of points in RP(m)
% d  = m + 1 "dimension"

if nargin == 1;
   % then proceed to find number of K

else % then find K clusters    
    
    % step 1: compute distance matrix
    D = real(acos(abs(A'*A))) ; % n x n distance matrix
    
    % step 2: find intial membership C
    % randomly pick K cluster center and compute membership C then find
    % within cluster variation W(C). Compare W(C) for best inital C.
    RR = 10; % 5 or 10
    Carr = zeros(RR,n); % array of membership C
    WCarr = zeros(RR,1); % array of within-cluster variation
    for r = 1:RR
      rid = randi(n,[K,1]);
      % cmpDmat = D(rid,:); % d(mu_i, x_j)
      % [~,C]=min(cmpDmat); % then assign membership
      [tmp,Carr(r,:)]=min(D(rid,:));   
      WCarr(r) = WC(Carr(r,:),D);
    end
    [wcini,minid]=min(WCarr); %wcini is used in the next step
    C = Carr(minid,:); % initial C that gives smallest within-cluster variation
    
    % step 3: iterative update
    % assign new center then update membership
    uid = zeros(K,1); % id for new center;
    atmp = 1:n;
    cnt = 0;
    WCarr = wcini;
    while true
        cnt = cnt+1;
        for k = 1:K
            [tmp,id]=min(sum(D(C==k,C==k)));
            atmp2 = atmp(C==k);
            uid(k) = atmp2(id);
        end
        [tmp,Cn]=min(D(uid,:));
        WCarr = [WCarr WC(Cn,D)];
        WCdiff = WCarr(end-1)-WCarr(end);
        if cnt > 10 || WCdiff <= 0
            break;
        else C = Cn; 
        end
    end
      
      
      
    % step 4: use geometric median to computer new center;
    muarr = zeros(d,K);
    % new center
    for k = 1:K
        muarr(:,k) = medianRPm(A(:,C == k));
    end
    % then 1-step update (for speed)
    muD = real(acos(abs(muarr'*A)));
    [tmp,C]=min(muD);
    
   
end
            
 
end


function wc = WC(C,D) %within cluster variation

K = max(C);
wc = 0;
for k = 1:K
    wc = wc + sum(sum(D(C == k, C ==k)))*0.5;
end
end