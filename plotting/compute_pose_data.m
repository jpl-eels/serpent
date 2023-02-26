function pose_data = compute_pose_data(data, align_first_pose, ...
    align_trajectories, align_opt_params)
    if align_first_pose && align_trajectories
        error("Both align_first_pose and align_trajectories were " + ...
            "true but only one can be.");
    end
    
    pose_data = struct;
    pose_data.gt.timestamps = extract_timestamps(data.gt.odom);
    pose_data.gt.poses = extract_poses(data.gt.odom);
    pose_data.gt.positions = extract_positions(pose_data.gt.poses);
    pose_data.gt.distances = vecnorm(pose_data.gt.positions, 2, 2);
    gt_quaternions = extract_quaternions(pose_data.gt.poses);
    gt_axang = quat2axang(gt_quaternions);
    pose_data.gt.rots_axang = gt_axang(:, 1:3) .* gt_axang(:, 4);
    pose_data.gt.angles = abs(gt_axang(:, 4));

    pose_data.num_odom = length(data.odoms);
    pose_data.names = data.names;
    pose_data.entries = cell(pose_data.num_odom, 1);
    for i = 1:pose_data.num_odom 
        odometry = data.odoms{i};
        entry = struct;
        entry.timestamps = extract_timestamps(odometry);
        entry.poses = extract_poses(odometry);

        % Filtering
        gt_bounds_filter = ...
            entry.timestamps <= pose_data.gt.timestamps(end) & ...
            entry.timestamps >= pose_data.gt.timestamps(1);
        entry.timestamps = entry.timestamps(gt_bounds_filter);
        entry.poses = entry.poses(gt_bounds_filter);

        % Alignment
        if align_first_pose
            entry = align_to_first_pose(pose_data.gt, entry);
        elseif align_trajectories
            entry = align_trajectory(pose_data.gt, entry, ...
                align_opt_params);
        end
        entry.positions = extract_positions(entry.poses);
        entry.quaternions = extract_quaternions(entry.poses);
        entry.distances = vecnorm(entry.positions, 2, 2);
        axang = quat2axang(entry.quaternions);
        entry.rots_axang = axang(:, 1:3) .* axang(:, 4);
        entry.angles = abs(axang(:, 4));
        [apes, ape_timestamps, rpes, rpe_timestamps] = ...
            compute_pose_errors(pose_data.gt.timestamps, ...
            pose_data.gt.poses, entry.timestamps, entry.poses);
        entry.ape.timestamps = ape_timestamps;
        [position_apes, rot_axang_apes] = split_poses(apes);
        entry.ape.positions = position_apes;
        entry.ape.rots_axang = rot_axang_apes;
        entry.ape_norm.position = vecnorm(position_apes, 2, 2);
        entry.ape_norm.rot_axang = vecnorm(rot_axang_apes, 2, 2);
        entry.ape_norm.position_rmse = rms(entry.ape_norm.position);
        entry.ape_norm.rot_axang_rmse = rms(entry.ape_norm.rot_axang);
        [position_rpes, rot_axang_rpes] = split_poses(rpes);
        entry.rpe.timestamps = rpe_timestamps;
        entry.rpe.positions = position_rpes;
        entry.rpe.rots_axang = rot_axang_rpes;
        entry.rpe_norm.position = vecnorm(position_rpes, 2, 2);
        entry.rpe_norm.rot_axang = vecnorm(rot_axang_rpes, 2, 2);
        entry.rpe_norm.position_rmse = rms(entry.rpe_norm.position);
        entry.rpe_norm.rot_axang_rmse = rms(entry.rpe_norm.rot_axang);
        pose_data.entries{i} = entry;
    end
end