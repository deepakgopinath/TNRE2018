function [ pg ] = compute_dft_Q( uh, xr, pg )
%COMPUTE_DFT_Q Summary of this function goes here
%   Detailed explanation goes here
global ng delta_t goal_transition_P;
Q = Q_from_P(goal_transition_P, delta_t);
h = 1/ng;
tau = 0.01;
rest_state = h*ones(ng, 1);
% rest_state = [0.7, 0.3, 0.1]';
curr_pg = compute_curr_input_R3(uh, xr);
lambda = 0*ones(ng,ng);
lambda(1: ng+1: ng*ng) = 0;
lambda = 200*eye(ng) + lambda; %200

dpgdt = Q*pg;
dpgdt = dpgdt + rest_state/(tau) + lambda*sigmoid(curr_pg);
pg = pg + dpgdt*delta_t;
pg(pg <= 0) = realmin;
pg = pg/sum(pg);

end

%%

function out = sigmoid(u)
    out = 1./(1 + exp(-u));
    out = out - 0.5;
end

function curr_input = compute_curr_input_R3(uh, xr)
    global ng xg;
    if size(xr, 2) > 1
        xr = xr';
    end
    curr_input = (1/ng)*ones(ng, 1);
%     curr_input = zeros(ng, 1);
    
    for i=1:ng %directedness based confidence function. 
        normxs = (xg(:, i) - xr)/(norm((xg(:, i) - xr)) + realmin); %ith goal position;
        normuh = uh/(norm(uh) +realmin); %add realmin to avoid divide by zero errors. 
        costh = dot(normxs, normuh);
        curr_input(i) = (1 + costh)/2;
    end
    
end
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