function interpolated_trajectory = interpolate_trajectory(...
    ref_timestamps, trajectory)
    interpolated_trajectory.timestamps = ref_timestamps;

    if (trajectory.timestamps(1) > ref_timestamps(1) || ...
            trajectory.timestamps(end) < ref_timestamps(end))
        error("Reference timestamps need to wrap trajectory timestamps.");
    end

    % Iterate over timestamps
    traj_index = 1;
    for index = 1:length(ref_timestamps)
        while traj_index + 1 <= length(trajectory.timestamps) && ...
                trajectory.timestamps(traj_index) <= ref_timestamps(index)
            traj_index = traj_index + 1;
        end
        [position, orientation] = ...
            interp_transform(trajectory.timestamps(traj_index - 1), ...
            trajectory.timestamps(traj_index), ...
            trajectory.poses(traj_index - 1), ...
            trajectory.poses(traj_index), ref_timestamps(index));
        T = [quat2rotm(orientation), position'; 0 0 0 1];
        interpolated_trajectory.poses(index) = matrix_to_pose(T);
    end
end

