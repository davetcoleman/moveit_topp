% read a optimized trajectory from a text file and plot

clear
clc
format long g
clf
 
% input
num_joints = 7;
filename = '/home/dave/ros/current/ws_acme/src/moveit_topp/data/topp_optimized_traj.csv';

% read
traj_data  = load_moveit_traj(filename, num_joints);
                                  
hold on;    
plot(traj_data.timestamp, traj_data.pos, 'DisplayName','PP Fitted Position');
%plot(traj_data.timestamp, traj_data.vel(:,1), 'DisplayName','PP Fitted Velocity');
%plot(traj_data.timestamp, traj_data.acc(:,1), 'DisplayName','PP Fitted Acceleration');

% Adjust size of plot
plot_gca = get(gca, 'Position');
plot_gca(1) = 0.03; % x
plot_gca(2) = 0.06; % y
plot_gca(3) = 0.95; % percent width
plot_gca(4) = 0.9; % percent height
set(gca, 'Position', plot_gca)

title('Joint 1')
xlabel('Time')
ylabel('Radians')
legend('Location','northwest');
figure(1)
