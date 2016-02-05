% read a optimized trajectory from a text file and plot

clear
clc
format long g
clf
 
% settings
show_pos = 1;
show_vel = 0;

% input
num_joints = 7;
filename = '/home/dave/ros/current/ws_acme/src/moveit_topp/data/topp_optimized_traj.csv';

% read
traj_data  = load_moveit_traj(filename, num_joints);
                                  
hold on;    
if show_pos
    h = plot(traj_data.timestamp, traj_data.pos);
    set(h,{'DisplayName'},{'PP Fitted Pos 1';'PP Fitted Pos 2';'PP Fitted Pos 3';'PP Fitted Pos 4';'PP Fitted Pos 5';'PP Fitted Pos 6';'PP Fitted Pos 7'});
end
if show_vel
    h = plot(traj_data.timestamp, traj_data.vel);
    set(h,{'DisplayName'},{'PP Fitted Vel 1';'PP Fitted Vel 2';'PP Fitted Vel 3';'PP Fitted Vel 4';'PP Fitted Vel 5';'PP Fitted Vel 6';'PP Fitted Vel 7'});
end
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
legend('Location','best');
figure(1)
