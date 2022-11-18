function fig_cov = plot_pose_with_cov(pose_with_cov_data, plot_opts)
    % Computation
    timestamps = pose_with_cov_data.timestamps;
    sigmas = pose_with_cov_data.sigmas;
    if plot_opts.start_from_time_zero
        timestamps = timestamps - timestamps(1);
    end

    % Covariance plot
    fig_cov = figure("Position", plot_opts.figure_dims, "Name", ...
        "Covariance");
    set(0, "CurrentFigure", fig_cov);

    % Plot covariances
    semilogy(timestamps, sigmas(:, 1));
    hold on;
    semilogy(timestamps, sigmas(:, 2));
    semilogy(timestamps, sigmas(:, 3));
    semilogy(timestamps, sigmas(:, 4));
    semilogy(timestamps, sigmas(:, 5));
    semilogy(timestamps, sigmas(:, 6));

    % Formatting
    title("Covariance Sigmas");
    xlabel("t (s)");
    ylabel("$\sigma$ (m, rad)", 'interpreter', 'latex');
    grid on;
    legend(["$x$", "$y$", "$z$", "$r_x$", "$r_y$", "$r_z$"], ...
        'interpreter', 'latex');
end