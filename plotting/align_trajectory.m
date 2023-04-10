function aligned_trajectory = align_trajectory(ref_trajectory, ...
    trajectory, optimisation_parameters)

    % Compute interpolated ref trajectory so that positions can be compared
    interp_ref_trajectory = interpolate_trajectory(...
        trajectory.timestamps, ref_trajectory);
    interp_ref_trajectory.positions = ...
        extract_positions(interp_ref_trajectory.poses);

    % Align all poses as initial guess
    aligned_trajectory = recompute_trajectory(trajectory, ...
        interp_ref_trajectory.poses(1));
    aligned_trajectory.positions = extract_positions(...
        aligned_trajectory.poses);

    % Optimise the correction parameters
    r_c = zeros(3, 1);
    t_c = zeros(3, 1);
    R_c = eye(3);
    gamma = optimisation_parameters.gamma;
    eta = optimisation_parameters.eta;
    iterations = optimisation_parameters.iterations;
    max_t_change = optimisation_parameters.max_t_change * ones(3, 1);
    max_r_change = optimisation_parameters.max_r_change * ones(3, 1);
    initial_loss = -1;
    final_loss = -1;
    for it = 1:iterations
        % Compute gradient of loss
        loss = 0.0;
        grad_loss = 0.0;
        for i = 1:length(aligned_trajectory.poses)
            t_1_i = aligned_trajectory.positions(i, :)';
            t_ref_i = interp_ref_trajectory.positions(i, :)';
            t_i = R_c * t_1_i + t_c;
            grad_loss = grad_loss + gamma^(i - 1) * ...
                [skew(t_1_i); eye(3)] * (t_i - t_ref_i);
            loss  = loss + gamma^(i - 1) * norm(t_i - t_ref_i)^2;
        end
        grad_loss = grad_loss * 2.0;

        % Update parameters
        delta_r_c = min(max(eta * grad_loss(1:3), -max_r_change), ...
            max_r_change);
        delta_t_c = min(max(eta * grad_loss(4:6), -max_t_change), ...
            max_t_change);
        r_c = r_c - delta_r_c;
        t_c = t_c - delta_t_c;
        R_c = axang2rotm([r_c' / norm(r_c), norm(r_c)]);
        
        if it == 1
            initial_loss = loss;
        elseif it == iterations
            final_loss = loss;
        end
    end
    optimised_T = [R_c, t_c; 0 0 0 1] ...
        * pose_to_matrix(interp_ref_trajectory.poses(1));

    % Align all poses
    aligned_trajectory = recompute_trajectory(trajectory, ...
        matrix_to_pose(optimised_T));
    aligned_trajectory.positions = extract_positions(aligned_trajectory.poses);

    % Print initial and final loss
    fprintf("Trajectory alignment loss: %.3f => %.3f\n", initial_loss, ...
        final_loss);
    fprintf("Trajectory correction (r, t): [%.3f, %.3f, %.3f, %.3f, " + ...
        "%.3f, %.3f]\n", r_c(1), r_c(2), r_c(3), t_c(1), t_c(2), t_c(3));
end

