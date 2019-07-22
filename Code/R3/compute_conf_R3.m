function conf = compute_conf_R3( uh, xr )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
global ng xg;
if size(xr, 2) > 1
    xr = xr';
end
conf = (1/ng)*ones(ng, 1);
for i=1:ng %directedness based confidence function. 
    normxs = (xg(:, i) - xr)/(norm((xg(:, i) - xr)) + realmin); %ith goal position;
    normuh = uh/(norm(uh) +realmin); %add realmin to avoid divide by zero errors. 
    costh = dot(normxs, normuh);
    conf(i) = (1 + costh)/2;
end
conf = conf/sum(conf);
end

