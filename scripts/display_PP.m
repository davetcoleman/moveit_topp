% read a optimized trajectory from a text file and plot

%clear
%clc
format long g
clf

% input
num_joints = 7;
filename = '/home/dave/ros/current/ws_acme/src/moveit_topp/data/spline_pp_traj.csv';

% read
raw = csvread(filename,1,0);
joints = [1:1:num_joints];
timestamp = raw(:,1)';

% Create struct
piecewise_polyn2.form = 'pp';
piecewise_polyn2.breaks = [timestamp 1];
piecewise_polyn2.coefs = raw(:,2:5);
piecewise_polyn2.pieces = size(timestamp,2);
piecewise_polyn2.order = 4;
piecewise_polyn2.dim = 1;
    
% Interpolate
discretization = 0.01;
time = [0:discretization:size(timestamp,2)];
pos = ppval(piecewise_polyn2, time);

% Plot
plot(time, pos, '--','DisplayName','Position');