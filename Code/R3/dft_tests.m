clc; clear all; close all;
%%
ng = 3;
pg = rand(3,1);
pg = pg/sum(pg);

goal_transition_P = eye(ng) + rand(ng)/10;
s = repmat(sum(goal_transition_P, 2),1,ng);
goal_transition_P = goal_transition_P./s;

pg_hist = pg;
for i=1:1000
    pg = goal_transition_P'*pg;
    pg_hist = [pg_hist, pg];
end
%%

A = eye(2);
pg = [0.3;0.7];
dt=0.01;
for i=1:10
    pg = pg + A*pg*dt;
end


%%
plot(pg_hist');
hold on; grid on;
axis([0, 1000, 0, 1]);