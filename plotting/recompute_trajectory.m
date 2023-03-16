% Recompute a trajectory given a new first pose. Trajectory shape is kept.
function new_trajectory = recompute_trajectory(trajectory, first_pose)
    new_trajectory.timestamps = trajectory.timestamps;
    new_trajectory.poses(1) = first_pose;
    previous_T = pose_to_matrix(trajectory.poses(1));
    previous_aligned_T = pose_to_matrix(first_pose);
    for i = 2:length(trajectory.poses)
        T = pose_to_matrix(trajectory.poses(i));
        relative_tf = previous_T \ T;
        aligned_T = previous_aligned_T * relative_tf;
        new_trajectory.poses(i) = matrix_to_pose(aligned_T);
        previous_T = T;
        previous_aligned_T = aligned_T;
    end
end

