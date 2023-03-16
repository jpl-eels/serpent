function len = compute_trajectory_length(poses)
    len = 0.0;
    previous_T = pose_to_matrix(poses(1));
    for i = 2:length(poses)
        T = pose_to_matrix(poses(i));
        relative_tf = previous_T \ T;
        relative_t = relative_tf(1:3, 4);
        len = len + vecnorm(relative_t);
        previous_T = T;
    end
end

