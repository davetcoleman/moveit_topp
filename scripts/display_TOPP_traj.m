% read a optimized trajectory from a text file and plot

clear
clc
format long g
clf
 
% input
num_joints = 1;
filename = '/home/dave/ros/current/ws_acme/src/moveit_topp/data/topp_optimized_traj.csv';

% read
raw = csvread(filename,1,0);
joints = [1:1:num_joints];
i=1;
timestamp = raw(:,i);i=i+1;
%timestamp = timestamp - timestamp(1) + time_offset;

% Preallocate memory
pos= zeros(size(timestamp,1),num_joints);

% Convert input data to better structures
for j = joints
    pos(:,j)=raw(:,i);i=i+1;
end
    
plot(timestamp, pos);