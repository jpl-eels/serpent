function pose_data = compute_pose_data(data)
    pose_data = struct;
    pose_data.gt.timestamps = extract_timestamps(data.gt.odom);
    gt_poses = extract_poses(data.gt.odom);
    pose_data.gt.positions = extract_positions(gt_poses);
    pose_data.gt.distances = vecnorm(pose_data.gt.positions, 2, 2);
    gt_quaternions = extract_quaternions(gt_poses);
    gt_axang = quat2axang(gt_quaternions);
    pose_data.gt.rots_axang = gt_axang(:, 1:3) .* gt_axang(:, 4);
    pose_data.gt.angles = gt_axang(:, 4);

    pose_data.num_odom = length(data.odoms);
    pose_data.names = data.names;
    pose_data.entries = cell(pose_data.num_odom, 1);
    for i = 1:pose_data.num_odom 
        odometry = data.odoms{i};
        entry = struct;
        entry.timestamps = extract_timestamps(odometry);
        poses = extract_poses(odometry);
        entry.positions = extract_positions(poses);
        entry.distances = vecnorm(entry.positions, 2, 2);
        quaternions = extract_quaternions(poses);
        axang = quat2axang(quaternions);
        entry.rots_axang = axang(:, 1:3) .* axang(:, 4);
        entry.angles = axang(:, 4);
        [ape_timestamps, apes] = compute_ape(...
            pose_data.gt.timestamps, gt_poses, entry.timestamps, poses);
        entry.ape.timestamps = ape_timestamps;
        [position_apes, rot_axang_apes] = split_poses(apes);
        entry.ape.positions = position_apes;
        entry.ape.rots_axang = rot_axang_apes;
        entry.ape_norm.position = vecnorm(position_apes, 2, 2);
        entry.ape_norm.rot_axang = vecnorm(rot_axang_apes, 2, 2);
        [rpe_timestamps, rpes] = compute_rpe(...
            pose_data.gt.timestamps, gt_poses, entry.timestamps, poses);
        [position_rpes, rot_axang_rpes] = split_poses(rpes);
        entry.rpe.timestamps = rpe_timestamps;
        entry.rpe.positions = position_rpes;
        entry.rpe.rots_axang = rot_axang_rpes;
        entry.rpe_norm.position = vecnorm(position_rpes, 2, 2);
        entry.rpe_norm.rot_axang = vecnorm(rot_axang_rpes, 2, 2);
        pose_data.entries{i} = entry;
    end
end