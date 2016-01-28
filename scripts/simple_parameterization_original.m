% my crappy attempt to create a smooth trajectory

clear
clc
format long g
clf

% Input trajectory
in_pos =   [0 4 5 0 -20 -5 -2 2 3 3 0];

%in_pos =   [0 1 4 5 3 0 -5 -10 -15 -19 -20 -19 -15 -10 -5 -2 2 3 3 1 0];

in_pos =   [0  1 2 4 5 0      -5 -10 -12 -15 -18 -20 -18 -15  -10   -5 -2 2 3 3 1 0];

in_time = 0:1:size(in_pos,2)-1;

% Constraints
max_velocity = 1;
max_acceleration = 1;

% Guess
max_time = size(in_pos,2)
target_time = 5;
scale_time = max_time / target_time


% Show input
hold on
plot(in_time / scale_time, in_pos, 'o-','DisplayName','Input Command');

% -------------------------------------------------
for i = 1:1:1

    % Interpolate
    discretization = 0.01;
    time = [0:discretization:max_time-1];
    cs = spline(in_time, [0 in_pos 0]);
    pos = ppval(cs, time);
    time_delta = zeros(size(pos,2),1); % change in time between points, starts at zero


    % scale down time
    time = time / scale_time;

    % Derive
    time_delta = diff(time);
    pos_delta = diff(pos);
    vel = [0, pos_delta ./ time_delta]; % add a zero in front to make same size as other data structures
    vel_delta = diff(vel);
    acc = [0, vel_delta ./ time_delta]; % add a zero in front to make same size as other data structures

    % find max
    [max_acc idx] = max(acc);
    max_acc
    max_acc_pos = pos(idx)
    plot([time(idx) time(idx)],[-30 30],'DisplayName','Max Acceleration');
    
    [min_acc idx] = min(acc);
    max_acc
    min_acc_pos = pos(idx)
    plot([time(idx) time(idx)],[-30 30],'DisplayName','Min Acceleration');

    % Plot
    plot(time, pos,'DisplayName','Position');
    plot(time, vel,'DisplayName','Velocity');
    plot(time, acc,'DisplayName','Acceleration');

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
    %w = waitforbuttonpress;

end