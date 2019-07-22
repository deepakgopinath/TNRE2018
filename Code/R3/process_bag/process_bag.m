function [u, t] = process_bag(bagfile)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

clear rosbag_wrapper;
clear ros.Bag;
bag = ros.Bag.load(bagfile);
bag.info()
topic1 = '/control_input';
[msgs, meta] = bag.readAll({topic1});
u = zeros(3, length(meta));
t = zeros(length(meta), 1);
for i=1:length(meta)
    u(:, i) = msgs{i}.data(1:3);
    t(i) = meta{i}.time.time;
end
t = t - t(1);
end

