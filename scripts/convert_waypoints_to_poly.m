% read a text file of waypoints and convert to splines

%clear
%clc
format long g
%clf

use_moveit_data = true;

% Input trajectory
if (use_moveit_data)
    time_offset = 0;
    num_joints = 7;
    moveit_data  = load_moveit_traj('/home/dave/ros/current/iiwa_trajectory_data/arm_moveit_trajectory_0.csv',...
                                    num_joints, time_offset);
    only_use_one_joint = true;
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

% Show input
hold on
plot(in_time, in_pos, 'o-','DisplayName','Input Command');

% Fit to spline
piecewise_polyn = spline(in_time, in_pos);

% Interpolate
discretization = 0.01;
time = [0:discretization:in_time(end)-1];
pos = ppval(piecewise_polyn, time);

% Plot
plot(time, pos, 'DisplayName','Position');
%plot(time, vel,'DisplayName','Velocity');
%plot(time, acc,'DisplayName','Acceleration');

% Adjust size of plot
plot_gca = get(gca, 'Position');
plot_gca(1) = 0.03; % x
plot_gca(2) = 0.06; % y
plot_gca(3) = 0.95; % percent width
plot_gca(4) = 0.9; % percent height
set(gca, 'Position', plot_gca)

xlabel('Time')
ylabel('Radians')
legend('Location','northwest');
figure(1)

%     filename = '/home/dave/ros/current/ws_acme/src/moveit_topp/data/matlab_traj.csv';
%     fileID = fopen(filename,'w');
%     for piece = 1:piecewise_polyn.pieces
%        duration = piecewise_polyn.breaks(piece + 1) - piecewise_polyn.breaks(piece);
%        fprintf(fileID, '%d\n', duration);
%        fprintf(fileID, '%i\n', piecewise_polyn.dim);
%        for dim = 1:piecewise_polyn.dim
%             chunk = piecewise_polyn.coefs(piece*piecewise_polyn.dim+dim-piecewise_polyn.dim,:);
%             fprintf(fileID, '%f %f %f %f \n', chunk(4), chunk(3), chunk(2), chunk(1));
%        end
%     end

% output as trajectory string
filename = '/home/dave/ros/current/ws_acme/src/moveit_topp/data/matlab_pp_traj.csv';
fileID = fopen(filename,'w');

polyn_string = find_matrix_spline(in_time, in_pos);

fprintf(fileID, polyn_string );
