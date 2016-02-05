% read a optimized trajectory from a text file and plot

clear
clc
format long g
clf

% input
num_joints = 7;
filename = '/home/dave/ros/current/ws_acme/src/moveit_topp/data/spline_pp_traj.csv';

% Create struct
piecewise_polyn2.form = 'pp';
piecewise_polyn2.breaks = [];
piecewise_polyn2.coefs = [];
piecewise_polyn2.pieces = 0;
piecewise_polyn2.order = 4;
piecewise_polyn2.dim = num_joints;

% read
%raw = csvread(filename,0,0);
fid = fopen(filename);

while true
    % add time
    line = fgetl(fid);
    if ischar(line) == false
        break
    end
    piecewise_polyn2.breaks = [piecewise_polyn2.breaks, str2num(line)];
    % get dimension and ignore - assume all are num_joints
    line = fgetl(fid);
    % get coefficients
    for i = 1:num_joints
        line = fgetl(fid);
        c = strsplit(line,',');
        num_line = [str2num(cell2mat(c(1))), str2num(cell2mat(c(2))), str2num(cell2mat(c(3))), str2num(cell2mat(c(4)))];
        piecewise_polyn2.coefs = [piecewise_polyn2.coefs; num_line];
    end
    piecewise_polyn2.pieces = piecewise_polyn2.pieces + 1;
end
fclose(fid);

piecewise_polyn2.breaks = [piecewise_polyn2.breaks, 1];

joints = [1:1:num_joints];
    
% Interpolate
discretization = 0.01;
time = [0:discretization:size(piecewise_polyn2.breaks,2)];
pos = ppval(piecewise_polyn2, time);

% Plot
h = plot(time, pos, '-');
set(h,{'DisplayName'},{'Discretized PP Pos 1';'Discretized PP Pos 2';'Discretized PP Pos 3';'Discretized PP Pos 4';'Discretized PP Pos 5';'Discretized PP Pos 6';'Discretized PP Pos 7'});
    
% Adjust size of plot
plot_gca = get(gca, 'Position');
plot_gca(1) = 0.03; % x
plot_gca(2) = 0.06; % y
plot_gca(3) = 0.95; % percent width
plot_gca(4) = 0.9; % percent height
set(gca, 'Position', plot_gca)

xlabel('Time')
ylabel('Radians')
legend('Location','best');
figure(1)

