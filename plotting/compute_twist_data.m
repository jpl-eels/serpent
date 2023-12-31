function twist_data = compute_twist_data(data, time_bounds)
    % Extraction and filtering
    twist_data = struct;
    twist_data.gt.timestamps = extract_timestamps(data.gt.odom);
    twist_data.gt.twists = extract_twists(data.gt.odom);
    if time_bounds.filter
        start_t = twist_data.gt.timestamps(1) + time_bounds.start;
        end_t = start_t + time_bounds.duration;
        filter_indices = and(twist_data.gt.timestamps >= start_t, ...
            twist_data.gt.timestamps <= end_t);
        twist_data.gt.timestamps = twist_data.gt.timestamps(filter_indices);
        twist_data.gt.twists = twist_data.gt.twists(filter_indices);
    end

    % Compute data
    twist_data.gt.linear_velocities = extract_linear_velocities(...
        twist_data.gt.twists);
    twist_data.gt.linear_speeds = vecnorm( ...
        twist_data.gt.linear_velocities, 2, 2);
    twist_data.gt.angular_velocities = extract_angular_velocities( ...
        twist_data.gt.twists);
    twist_data.gt.angular_speeds = vecnorm( ...
        twist_data.gt.angular_velocities, 2, 2);

    twist_data.num_odom = length(data.odoms);
    twist_data.names = data.names;
    twist_data.entries = cell(twist_data.num_odom, 1);
    for i = 1:twist_data.num_odom
        odometry = data.odoms{i};
        twists = extract_twists(odometry);
        entry = struct;
        entry.timestamps = extract_timestamps(odometry);
        
        % Filtering
        gt_bounds_filter = ...
            entry.timestamps <= twist_data.gt.timestamps(end) & ...
            entry.timestamps >= twist_data.gt.timestamps(1);
        entry.timestamps = entry.timestamps(gt_bounds_filter);
        twists = twists(gt_bounds_filter);

        entry.linear_velocities = extract_linear_velocities(twists);
        entry.linear_speeds = vecnorm(entry.linear_velocities, 2, 2);
        entry.angular_velocities = extract_angular_velocities(twists);
        entry.angular_speeds = vecnorm(entry.angular_velocities, 2, 2);
        % Note it would be more efficient if we did a 6-dim vec rather than
        % two 3-dim vecs.
        [linear_aes, linear_aes_timestamps, linear_res, ...
            linear_res_timestamps] = compute_vector_errors(...
            twist_data.gt.timestamps, twist_data.gt.linear_velocities, ...
            entry.timestamps, entry.linear_velocities);
        entry.ae.timestamps = linear_aes_timestamps;
        entry.re.timestamps = linear_res_timestamps;
        entry.ae.linear_velocities = linear_aes;
        entry.ae_norm.linear_velocity = vecnorm( ...
            entry.ae.linear_velocities, 2, 2);
        entry.ae_norm.linear_velocity_rmse = ...
            rms(entry.ae_norm.linear_velocity);
        entry.re.linear_velocities = linear_res;
        entry.re_norm.linear_velocity = vecnorm( ...
            entry.re.linear_velocities, 2, 2);
        entry.re_norm.linear_velocity_rmse = ...
            rms(entry.re_norm.linear_velocity);
        [angular_aes, ~, angular_res, ~] = compute_vector_errors(...
            twist_data.gt.timestamps, twist_data.gt.angular_velocities, ...
            entry.timestamps, entry.angular_velocities);
        entry.ae.angular_velocities = angular_aes;
        entry.ae_norm.angular_velocity = vecnorm( ...
            entry.ae.angular_velocities, 2, 2);
        entry.ae_norm.angular_velocity_rmse = ...
            rms(entry.ae_norm.angular_velocity);
        entry.re.angular_velocities = angular_res;
        entry.re_norm.angular_velocity = vecnorm( ...
            entry.re.angular_velocities, 2, 2);
        entry.re_norm.angular_velocity_rmse = ...
            rms(entry.re_norm.angular_velocity);
        twist_data.entries{i} = entry;
    end
end