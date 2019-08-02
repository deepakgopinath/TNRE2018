clear all; clc; close all;
%%
n = 2;
p0 = (1/n)*ones(n, 1);
p0 = [0.6, 0.4]';

P = eye(n) + rand(n)/1;
s = repmat(sum(P, 2),1,n);
P = P./s;


%%
T = 100;
pt = zeros(n, T);
pt(:, 1) = p0;
for i=1:T-1
    pt(:, i+1) = P'*pt(:, i);
end


%%

n = 2;
dt = 0.1;
w12 = min(10, rand*10);
w21 = min(10, rand*10);

p0 = rand(n, 1);
% p0 = 1/n*ones(n, 1);
p0 = p0/sum(p0);

P = [1 - w12*dt, w12*dt; %total number of independent parameters is n(n-1)
    w21*dt, 1 - w21*dt];

Q = [-w12, w21;
     w12, -w21];

T = 100;
ptP = zeros(n, T);
ptP(:, 1) = p0;
for i=1:T-1
    dx = -100*P'*ptP(:, i);
    x = ptP(:, i) + dx*dt + (1/n)*ones(n, 1);
%     x = (eye(n) - P')*ptP(:, i);
    x(x <= 0) = realmin;
    x = x/sum(x);
    ptP(:, i+1) = x;
end

ptQ = zeros(n, T);
ptQ(:, 1) = p0;
for i=1:T-1
    dPdt = Q*ptQ(:, i);
%     ptQ(:, i+1) = ptQ(:, i) + dPdt*dt;
    ptQ(:, i+1) = (eye(n) + Q*dt)*ptQ(:, i);
    
    
end


%%
n = 3;
dt = 0.1;
w12 = 1;
w13 = 2;
w21 = 3;
w23 = 3;
w31 = 2;
w32 = 1;

P = [1- (w12 + w13)*dt, w12*dt, w13*dt;
    w21*dt, 1- (w21 + w23)*dt, w23*dt;
    w31*dt, w32*dt, 1-(w31 + w32)*dt];

Q = [-(w21 + w31), w21, w31;
       w12, -(w12 + w32), w32;
       w13, w23, -(w13+w23)];

QfP = Q_from_P(P, dt);

%%
function Q = Q_from_P(P, dt)
assert (size(P, 1) == size(P, 2))
N = size(P, 1);
Q = zeros(N, N);
for i=1:N
    for j=1:N
        if j==i
            continue
        end
        Q(i, j) = P(j, i)/dt;
    end
    Q(i, i) = -sum(Q(i, :));
end

end

 
