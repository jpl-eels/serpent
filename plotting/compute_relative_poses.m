function rel_poses = compute_relative_poses(poses)
    previous_T = pose_to_matrix(poses(1));
    for i = 2:length(poses)
        T = pose_to_matrix(poses(i));
        relative_tf = previous_T \ T;
        rel_poses(i - 1) = matrix_to_pose(relative_tf);
        previous_T = T;
    end
end

