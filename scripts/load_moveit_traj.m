function data = load_moveit_traj(filename, num_joints)
%
% This function converts a CSV file from a MoveIt! trajectory into a Matlab
% struct
%
    raw = csvread(filename,1,0);
    i=1;
    data.timestamp = raw(:,i);i=i+1;
    %data.timestamp = data.timestamp - data.timestamp(1);

    joints = [1:1:num_joints];

    % Preallocate memory
    data.pos= zeros(size(data.timestamp,1),7);
    data.vel= zeros(size(data.timestamp,1),7);
    data.acc = zeros(size(data.timestamp,1),7);

    % Convert input data to better structures
    for j = joints
        data.pos(:,j)=raw(:,i);i=i+1;
        data.vel(:,j)=raw(:,i);i=i+1;
        data.acc(:,j)=raw(:,i);i=i+1;

        % calculate the derivatives
        m_t_delta=diff(data.timestamp);
        m_pos_delta(:,j)=diff(data.pos(:,j));
        data.calc_vel(:,j)=[0; m_pos_delta(:,j) ./ m_t_delta]; % add a zero in front to make same size as other data structures
        m_vel_delta(:,j)=diff(data.calc_vel(:,j));
        data.calc_acc(:,j)=[0; m_vel_delta(:,j) ./ m_t_delta]; % add a zero in front to make same size as other data structures
        m_acc_delta(:,j)=diff(data.calc_acc(:,j));
        data.calc_jer(:,j)=[0; m_acc_delta(:,j) ./ m_t_delta]; % add a zero in front to make same size as other data structures
    end
end