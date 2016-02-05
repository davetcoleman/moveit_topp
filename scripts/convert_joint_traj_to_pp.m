% read a text file of waypoints and convert to splines

clear
clc
format long g
clf
hold on

% Settings
use_moveit_data = 1;
show_pos = 1;
show_vel = 0;
show_acc = 0;
show_optimized = 1;

% ------------------------------------------------------------------------
% Input trajectory
if (use_moveit_data)
    num_joints = 7;
    filename = '/home/dave/ros/current/ws_acme/src/moveit_topp/data/moveit_joint_traj.csv';
    moveit_data  = load_moveit_traj(filename, num_joints);
       
    % scale time
    time_scaling_factor = 1; %0.05;
    moveit_data.timestamp = moveit_data.timestamp / time_scaling_factor;
    
    only_use_one_joint = false;
    if (only_use_one_joint)
        num_joints = 1;
        joint_id = 1;
        in_pos = moveit_data.pos(:,joint_id)';
        in_time = moveit_data.timestamp;
    else
        in_pos = moveit_data.pos';
        in_time = moveit_data.timestamp;
    end
else
    in_pos =   [0    0.2 0 -0.2 -0.5 0;
                0.25 0.5 0 -0.5  0   0];
    num_joints = 2;
    %in_pos =   [0 3.14 1.57 0 -5 -3 -2 2 3 3 0];
    in_time = 0:1:size(in_pos,2)-1;
end

% ------------------------------------------------------------------------
% Input PP Fitted Trajectory
if (show_optimized)
    filename2 = '/home/dave/ros/current/ws_acme/src/moveit_topp/data/topp_optimized_traj.csv';

    % read
    traj_data  = load_moveit_traj(filename2, num_joints);

    % Show fitted
    for joint = 1:num_joints
        title_str = sprintf(' Joint %i',joint);
        if (show_pos); plot(traj_data.timestamp, traj_data.pos(:,joint), 'DisplayName',strcat('TOPP ',title_str,' Position')); end;
        if (show_vel); plot(traj_data.timestamp, traj_data.vel(:,joint), 'DisplayName',strcat('TOPP ',title_str,' Velocity')); end;
        if (show_acc); plot(traj_data.timestamp, traj_data.acc(:,joint), 'DisplayName',strcat('TOPP ',title_str,' Acceleration')); end;
    end
end
% ------------------------------------------------------------------------

% Show input
if (show_pos); plot(in_time, in_pos, 'o','DisplayName','Input Command'); end;

% Fit to spline
piecewise_polyn = spline(in_time, in_pos);

% Interpolate
discretization = 0.01;
time = [0:discretization:in_time(end)];
pos = ppval(piecewise_polyn, time);
% vel_time = time(:,1:end-1);
% vel_time=repmat(vel_time,num_joints,1);
% size(vel_time)
% size(diff(pos')')
% vel = diff(pos')'./diff(vel_time);
% vel = [vel vel(end)]; %TODO this repeats the final velocity value twice
% acc = diff(vel)./diff(time);
% acc = [acc acc(end)]; %TODO this repeats the final velocity value twice

% Plot
if (show_pos); plot(time, pos, '--', 'DisplayName','PP Pos'); end;
%if (show_vel); plot(time, vel,'DisplayName','Velocity'); end;
%if (show_acc); plot(time, acc,'DisplayName','Acceleration'); end;

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

% output as trajectory string
filename = '/home/dave/ros/current/ws_acme/src/moveit_topp/data/matlab_pp_traj.csv';
fileID = fopen(filename,'w');

polyn_string = getPPString(in_time, in_pos);

fprintf(fileID, polyn_string );
