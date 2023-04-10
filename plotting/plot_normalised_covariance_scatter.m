function fig = plot_normalised_covariance_scatter(ref_trajectory, ...
    trajectory, registration_cov_data)
    % Interpolate trajectory
    interp_ref_trajectory = interpolate_trajectory(...
        trajectory.timestamps, ref_trajectory);

    % Compute relative poses
    trajectory_rel_poses = compute_relative_poses(trajectory.poses);
    trajectory_rel_positions = ...
        extract_positions(trajectory_rel_poses);
    interp_ref_trajectory_rel_poses = ...
        compute_relative_poses(interp_ref_trajectory.poses);
    interp_ref_trajectory_rel_positions = ...
        extract_positions(interp_ref_trajectory_rel_poses);

    % Compute translation error
    t_error = trajectory_rel_positions - interp_ref_trajectory_rel_positions;

    if (size(t_error, 1) ~= length(registration_cov_data.covariance))
        error("t_error and registration covariance had different lengths");
    end

    % Determine scaling factors from covariance (in ROS, t before r)
    sdwidth = 2.0; % 2 stddevs
    norm_t_error = zeros(size(t_error));
    t_error_len = zeros(size(t_error, 1));
    for i = 1:length(t_error)
        t_error_len(i) = vecnorm(t_error(i, :));

        % Translation covariance only
        [v,d]=eig(registration_cov_data.covariance{i}(1:3, 1:3)); 
        if any(d(:) < 0)
           fprintf('warning: negative eigenvalues\n');
           d = max(d,0);
        end
        d = sdwidth * sqrt(d); % convert variance to sdwidth*sd
        % sdwidth * sqrt(d) is how you would scale the sphere, so the
        % inverse is how you would scale the errors?
        d_inv = inv(d);
        norm_t_error(i, :) = (v * d_inv * t_error(i, :)')';
        norm_t_error_vecnorm = vecnorm(norm_t_error(i, :));
    end

%     % Create figure
%     fig = figure;
%     hold on;
%     scatter3(t_error(:, 1), t_error(:, 2), t_error(:, 3), 'b.');
%     ax = gca();
%     plot_gaussian_ellipsoid([0, 0, 0], [1e-4, 0, 0; 0, 1e-4, 0; 0, 0, 1e-4], 2, 100, ax);
%     axis equal;
%     fraction = length(t_error_len(t_error_len < 1e-2)) / length(t_error_len);
%     fprintf("%.3f within sphere of radius %.3f\n", fraction, radius);

    % Create figure
    fig = figure;
    hold on;
    scatter3(norm_t_error(:, 1), norm_t_error(:, 2), norm_t_error(:, 3), 'b.');
    [x,y,z] = sphere(100);
    h = surf(x, y, z, 'EdgeAlpha', 0, 'FaceAlpha', 0.5);
    axis equal;
    fraction = length(norm_t_error_vecnorm(norm_t_error_vecnorm < 1)) ...
        / length(norm_t_error_vecnorm);
    fprintf("%.6f within normalised covariance sphere\n", fraction);
end

