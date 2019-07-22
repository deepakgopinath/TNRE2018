function [ positions, orientations, user_vel, gps ] = process_bag_tf( bagfile )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global start_time bag;
clear rosbag_wrapper;
clear ros.Bag;
bag = ros.Bag.load(bagfile);
bag.info();
tree = ros.TFTree(bag);
tree.allFrames();
source_frame = 'mico_link_base';
target_frame = 'mico_link_hand';
%get user_vel
% get_start_time();
start_time = bag.time_begin;
user_vel = get_user_vel();

gp_ts = get_goal_probabilities();
gps = zeros(size(gp_ts, 1) - 1, size(user_vel, 2));
for j=1:size(gp_ts, 1) - 1
    gps(j, :) = interp1(gp_ts(end, :), gp_ts(j, :), user_vel(end, :), 'linear', 'extrap');
end
    
    
tftimes = user_vel(end, :)' + start_time;
tftimes(tftimes > (tree.time_end - 1)) = tree.time_end - 1;
tftimes(tftimes < (tree.time_begin + 1)) = tree.time_begin + 1; 
mico_xyz = tree.lookup(source_frame, target_frame, tftimes);
positions = zeros(3, length(mico_xyz));
orientations = zeros(4, length(mico_xyz));
for ii=1:length(mico_xyz)
    positions(:, ii) = mico_xyz(ii).translation;
    orientations(:, ii) = mico_xyz(ii).rotation;                      
end


end

function user_vel = get_user_vel()
    global start_time;
    topic = '/user_vel';
    [msgs, meta] = get_bag_data(topic);
    user_vel = zeros(length(msgs{1}.data(1:6)) + 1, length(msgs));
    for i=1:length(msgs)
        user_vel(1:end-1, i) = msgs{i}.data(1:6);
        user_vel(end, i) = meta{i}.time.time - start_time;
    end
    user_vel = trim_data(user_vel, true);
%     user_vel = round(gp, 4);
   
end

function  get_start_time()
    global start_time;
    topic = '/start_end';
    [~, meta] = get_bag_data(topic);
    start_time = meta{1}.time.time;
end
function gp = get_goal_probabilities()
    global start_time;
    topic = '/goal_probabilities';
    [msgs, meta] = get_bag_data(topic);
    gp = zeros(length(msgs{1}.data) + 1, length(msgs));
    
    for i=1:length(msgs)
        gp(1:end-1, i) = msgs{i}.data;
        gp(end, i) = meta{i}.time.time - start_time;
    end
    gp = trim_data(gp, true);
    gp = round(gp, 4);
%     disp(gp);
    
end

function [msgs, meta] = get_bag_data(topic)
    global bag;
    [msgs, meta] = bag.readAll({topic});
end

function [td] = trim_data(d, isrow)
    if isrow
        ts = d(end, :);
    else
        ts = d(:, end);
    end
    first_t = find(ts < 0);
    if ~isempty(first_t)
        first_t = first_t(end);
        if isrow()
            d(:, 1:first_t) = [];
        else
            d(1:first_t, :) = [];
        end
    end
    td = d;
end