function [fig_trajectory, fig_trajectory_orientation, ...
    fig_trajectory_velocity] = plot_trajectory(pose_data, twist_data, ...
    plot_opts)
    % Figures
    fig_trajectory = figure("Position", plot_opts.figure_dims, "Name", ...
        "Trajectory");
    fig_trajectory_orientation = figure("Position", ...
        plot_opts.figure_dims, "Name", "Trajectory Orientation");
    fig_trajectory_velocity = figure("Position", plot_opts.figure_dims, ...
        "Name", "Trajectory Velocity");

    % Ground Truth
    set(0, "CurrentFigure", fig_trajectory);
    plot3(pose_data.gt.positions(:, 1), pose_data.gt.positions(:, 2), ...
        pose_data.gt.positions(:, 3), plot_opts.gt_linespec);
    hold on;
    xlabel("x");
    ylabel("y");
    zlabel("z");
    grid on;
    set(0, "CurrentFigure", fig_trajectory_orientation);
    plot3(pose_data.gt.positions(:, 1), pose_data.gt.positions(:, 2), ...
        pose_data.gt.positions(:, 3), plot_opts.gt_linespec);
    hold on;
    quiver3(pose_data.gt.positions(:, 1), pose_data.gt.positions(:, 2), ...
        pose_data.gt.positions(:, 3), pose_data.gt.rots_axang(:, 1), ...
        pose_data.gt.rots_axang(:, 2), pose_data.gt.rots_axang(:, 3), ...
        plot_opts.gt_linespec);
    xlabel("x");
    ylabel("y");
    zlabel("z");
    grid on;
    set(0, "CurrentFigure", fig_trajectory_velocity);
    plot3(pose_data.gt.positions(:, 1), pose_data.gt.positions(:, 2), ...
        pose_data.gt.positions(:, 3), plot_opts.gt_linespec);
    hold on;
    quiver3(pose_data.gt.positions(:, 1), pose_data.gt.positions(:, 2), ...
        pose_data.gt.positions(:, 3), ...
        twist_data.gt.linear_velocities(:, 1), ...
        twist_data.gt.linear_velocities(:, 2), ...
        twist_data.gt.linear_velocities(:, 3), plot_opts.gt_linespec);
    xlabel("x");
    ylabel("y");
    zlabel("z");
    grid on;
    
    % Trajectories
    traj_names_with_gt = ["Reference", pose_data.names];
    traj_orient_names_with_gt = ["Reference", "Reference orientation", ...
        repmat("", 1, pose_data.num_odom*2)];
    traj_vel_names_with_gt = ["Reference", "Reference velocity", ...
        repmat("", 1, pose_data.num_odom*2)];
    for i = 1:pose_data.num_odom
        traj_orient_names_with_gt(2 * i + 1) = pose_data.names(i);
        traj_orient_names_with_gt(2 * i + 2) = ...
            join([pose_data.names(i), " orientation"], "");
        traj_vel_names_with_gt(2 * i + 1) = pose_data.names(i);
        traj_vel_names_with_gt(2 * i + 2) = join([pose_data.names(i), ...
            " velocity"], "");
        colour = plot_opts.colours{i};
        positions = pose_data.entries{i}.positions;
        rots_axang = pose_data.entries{i}.rots_axang;
        linear_velocities = twist_data.entries{i}.linear_velocities;
        set(0, "CurrentFigure", fig_trajectory);
        plot3(positions(:, 1), positions(:, 2), positions(:, 3), ...
            "Color", colour);
        set(0, "CurrentFigure", fig_trajectory_orientation);
        plot3(positions(:, 1), positions(:, 2), positions(:, 3), ...
            "Color", colour);
        quiver3(positions(:, 1), positions(:, 2), positions(:, 3), ...
            rots_axang(:, 1), rots_axang(:, 2), rots_axang(:, 3), ...
            "Color", colour);
        set(0, "CurrentFigure", fig_trajectory_velocity);
        plot3(positions(:, 1), positions(:, 2), positions(:, 3), ...
            "Color", colour);
        quiver3(positions(:, 1), positions(:, 2), positions(:, 3), ...
            linear_velocities(:, 1), linear_velocities(:, 2), ...
            linear_velocities(:, 3), "Color", colour);
    end

    % Legend
    set(0, "CurrentFigure", fig_trajectory);
    legend(traj_names_with_gt, "Location", "best");
    set(0, "CurrentFigure", fig_trajectory_orientation);
    legend(traj_orient_names_with_gt, "Location", "best");
    set(0, "CurrentFigure", fig_trajectory_velocity);
    legend(traj_vel_names_with_gt, "Location", "best");
end