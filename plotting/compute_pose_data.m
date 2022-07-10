function pose_data = compute_pose_data(data, align_first_pose)
    pose_data = struct;
    pose_data.gt.timestamps = extract_timestamps(data.gt.odom);
    gt_poses = extract_poses(data.gt.odom);
    pose_data.gt.positions = extract_positions(gt_poses);
    pose_data.gt.distances = vecnorm(pose_data.gt.positions, 2, 2);
    gt_quaternions = extract_quaternions(gt_poses);
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
        poses = extract_poses(odometry);

        % Filtering
        gt_bounds_filter = ...
            entry.timestamps <= pose_data.gt.timestamps(end) & ...
            entry.timestamps >= pose_data.gt.timestamps(1);
        entry.timestamps = entry.timestamps(gt_bounds_filter);
        poses = poses(gt_bounds_filter);

        % Align to first pose
        if align_first_pose
            % Find gt transform at timestamp of first pose
            gt_index = 1;
            while pose_data.gt.timestamps(gt_index) < entry.timestamps(1)
                gt_index = gt_index + 1;
            end
            if pose_data.gt.timestamps(gt_index) == entry.timestamps(1)
                first_gt_T = pose_to_matrix(gt_poses(gt_index));
                first_gt_position = first_gt_T(1:3, 4)';
                first_gt_orientation = rotm2quat(first_gt_T(1:3, 1:3));
            else
                [first_gt_position, first_gt_orientation] = ...
                    interp_transform(pose_data.gt.timestamps(gt_index - 1), ...
                    pose_data.gt.timestamps(gt_index), ...
                    gt_poses(gt_index - 1), gt_poses(gt_index), ...
                    entry.timestamps(1));
                first_gt_T = [quat2rotm(first_gt_orientation), ...
                    first_gt_position'; 0 0 0 1];
            end

            % Align all poses
            entry.positions = zeros(length(poses), 3);
            quaternions = zeros(length(poses), 4);
            entry.positions(1, :) = first_gt_position;
            quaternions(1, :) = first_gt_orientation;
            previous_T = pose_to_matrix(poses(1));
            previous_aligned_T = [quat2rotm(first_gt_orientation), ...
                first_gt_position'; 0 0 0 1];
            poses(1) = matrix_to_pose(first_gt_T);
            for j = 2:length(poses)
                T = pose_to_matrix(poses(j));
                relative_tf = previous_T \ T;
                aligned_T = previous_aligned_T * relative_tf;
                poses(j) = matrix_to_pose(aligned_T);
                entry.positions(j, :) = extract_position(poses(j));
                quaternions(j, :) = extract_quaternion(poses(j));
                previous_T = T;
                previous_aligned_T = aligned_T;
            end
        else
            entry.positions = extract_positions(poses);
            quaternions = extract_quaternions(poses);
        end
        entry.distances = vecnorm(entry.positions, 2, 2);
        axang = quat2axang(quaternions);
        entry.rots_axang = axang(:, 1:3) .* axang(:, 4);
        entry.angles = abs(axang(:, 4));
        [apes, ape_timestamps, rpes, rpe_timestamps] = ...
            compute_pose_errors(pose_data.gt.timestamps, gt_poses, ...
            entry.timestamps, poses);
        entry.ape.timestamps = ape_timestamps;
        [position_apes, rot_axang_apes] = split_poses(apes);
        entry.ape.positions = position_apes;
        entry.ape.rots_axang = rot_axang_apes;
        entry.ape_norm.position = vecnorm(position_apes, 2, 2);
        entry.ape_norm.rot_axang = vecnorm(rot_axang_apes, 2, 2);
        [position_rpes, rot_axang_rpes] = split_poses(rpes);
        entry.rpe.timestamps = rpe_timestamps;
        entry.rpe.positions = position_rpes;
        entry.rpe.rots_axang = rot_axang_rpes;
        entry.rpe_norm.position = vecnorm(position_rpes, 2, 2);
        entry.rpe_norm.rot_axang = vecnorm(rot_axang_rpes, 2, 2);
        pose_data.entries{i} = entry;
    end
end