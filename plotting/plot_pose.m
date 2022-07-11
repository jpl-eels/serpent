function [fig_position, fig_orientation] = plot_pose(pose_data, plot_opts)
    % Computation
    names_with_gt = ["Reference", pose_data.names];
    gt_format = 'k--';
    gt_rots_axang = pose_data.gt.rots_axang;
    gt_angles = pose_data.gt.angles;
    angle_limits = [-pi, pi];
    if plot_opts.angle == "deg"
        gt_rots_axang = rad2deg(gt_rots_axang);
        gt_angles = rad2deg(gt_angles);
        angle_limits = rad2deg(angle_limits);
    end

    % Reference Position
    fig_position = figure("Position", plot_opts.figure_dims, "Name", ...
        "Position");
    set(0, "CurrentFigure", fig_position);
    r = 4;
    c = 3;
    title("Position");
    subplot(r,c,1);
    plot(pose_data.gt.timestamps, pose_data.gt.positions(:, 1), gt_format);
    xlabel("t (s)");
    ylabel("x (m)");
    grid on;
    subplot(r,c,2);
    xlabel("t (s)");
    ylabel("APE x (m)");
    grid on;
    subplot(r,c,3);
    xlabel("t (s)");
    ylabel("RPE x (m)");
    grid on;
    subplot(r,c,4);
    plot(pose_data.gt.timestamps, pose_data.gt.positions(:, 2), gt_format);
    xlabel("t (s)");
    ylabel("y (m)");
    grid on;
    subplot(r,c,5);
    xlabel("t (s)");
    ylabel("APE y (m)");
    grid on;
    subplot(r,c,6);
    xlabel("t (s)");
    ylabel("RPE y (m)");
    grid on;
    subplot(r,c,7);
    plot(pose_data.gt.timestamps, pose_data.gt.positions(:, 3), gt_format);
    xlabel("t (s)");
    ylabel("z (m)");
    grid on;
    subplot(r,c,8);
    xlabel("t (s)");
    ylabel("APE z (m)");
    grid on;
    subplot(r,c,9);
    xlabel("t (s)");
    ylabel("RPE z (m)");
    grid on;
    subplot(r,c,10);
    plot(pose_data.gt.timestamps, pose_data.gt.distances, gt_format);
    xlabel("t (s)");
    ylabel("Distance (m)");
    grid on;
    subplot(r,c,11);
    xlabel("t (s)");
    ylabel("APE (m)");
    grid on;
    subplot(r,c,12);
    xlabel("t (s)");
    ylabel("RPE (m)");
    grid on;

    % Reference Orientation
    fig_orientation = figure("Position", plot_opts.figure_dims, "Name", ...
        "Orientation");
    set(0, "CurrentFigure", fig_orientation);
    r = 4;
    c = 3;
    title("Orientation");
    subplot(r,c,1);
    plot(pose_data.gt.timestamps, gt_rots_axang(:, 1), gt_format);
    xlabel("t (s)");
    ylabel(join(["r_x (", plot_opts.angle, ")"], ""));
    ylim(angle_limits);
    grid on;
    subplot(r,c,2);
    xlabel("t (s)");
    ylabel(join(["APE r_x (", plot_opts.angle, ")"], ""));
    grid on;
    subplot(r,c,3);
    xlabel("t (s)");
    ylabel(join(["RPE r_x (", plot_opts.angle, ")"], ""));
    grid on;
    subplot(r,c,4);
    plot(pose_data.gt.timestamps, gt_rots_axang(:, 2), gt_format);
    xlabel("t (s)");
    ylabel(join(["r_y (", plot_opts.angle, ")"], ""));
    ylim(angle_limits);
    grid on;
    subplot(r,c,5);
    xlabel("t (s)");
    ylabel(join(["APE r_y (", plot_opts.angle, ")"], ""));
    grid on;
    subplot(r,c,6);
    xlabel("t (s)");
    ylabel(join(["RPE r_y (", plot_opts.angle, ")"], ""));
    grid on;
    subplot(r,c,7);
    plot(pose_data.gt.timestamps, gt_rots_axang(:, 3), gt_format);
    xlabel("t (s)");
    ylabel(join(["r_z (", plot_opts.angle, ")"], ""));
    ylim(angle_limits);
    grid on;
    subplot(r,c,8);
    xlabel("t (s)");
    ylabel(join(["APE r_z (", plot_opts.angle, ")"], ""));
    grid on;
    subplot(r,c,9);
    xlabel("t (s)");
    ylabel(join(["RPE r_z (", plot_opts.angle, ")"], ""));
    grid on;
    subplot(r,c,10);
    plot(pose_data.gt.timestamps, gt_angles, gt_format);
    xlabel("t (s)");
    ylabel(join(["Angle (", plot_opts.angle, ")"], ""));
    ylim([0, angle_limits(2)]);
    grid on;
    subplot(r,c,11);
    xlabel("t (s)");
    ylabel(join(["APE (", plot_opts.angle, ")"], ""));
    grid on;
    subplot(r,c,12);
    xlabel("t (s)");
    ylabel(join(["RPE (", plot_opts.angle, ")"], ""));
    grid on;

    % Query Poses
    for i = 1:pose_data.num_odom
        colour = plot_opts.colours{i};
        entry = pose_data.entries{i};
        timestamps = entry.timestamps;
        ape_timestamps = entry.ape.timestamps;
        rpe_timestamps = entry.rpe.timestamps;
        positions = entry.positions;
        distances = entry.distances;
        position_apes = entry.ape.positions;
        position_apes_norm = entry.ape_norm.position;
        position_rpes = entry.rpe.positions;
        position_rpes_norm = entry.rpe_norm.position;
        rots_axang = entry.rots_axang;
        angles = entry.angles;
        rot_axang_apes = entry.ape.rots_axang;
        rot_axang_apes_norm = entry.ape_norm.rot_axang;
        rot_axang_rpes = entry.rpe.rots_axang;
        rot_axang_rpes_norm = entry.rpe_norm.rot_axang;
        if plot_opts.angle == "deg"
            rots_axang = rad2deg(rots_axang);
            angles = rad2deg(angles);
            rot_axang_apes = rad2deg(rot_axang_apes);
            rot_axang_apes_norm = rad2deg(rot_axang_apes_norm);
            rot_axang_rpes = rad2deg(rot_axang_rpes);
            rot_axang_rpes_norm = rad2deg(rot_axang_rpes_norm);
        end

        % Position
        set(0, "CurrentFigure", fig_position);
        subplot(r,c,1);
        hold on;
        plot(timestamps, positions(:, 1), 'color', colour);
        subplot(r,c,4);
        hold on;
        plot(timestamps, positions(:, 2), 'color', colour);
        subplot(r,c,7);
        hold on;
        plot(timestamps, positions(:, 3), 'color', colour);
        subplot(r,c,10);
        hold on;
        plot(timestamps, distances, 'color', colour);

        subplot(r,c,2);
        hold on;
        plot(ape_timestamps, position_apes(:, 1), 'color', colour);
        subplot(r,c,5);
        hold on;
        plot(ape_timestamps, position_apes(:, 2), 'color', colour);
        subplot(r,c,8);
        hold on;
        plot(ape_timestamps, position_apes(:, 3), 'color', colour);
        subplot(r,c,11);
        hold on;
        plot(ape_timestamps, position_apes_norm, 'color', colour);

        subplot(r,c,3);
        hold on;
        plot(rpe_timestamps, position_rpes(:, 1), 'color', colour);
        subplot(r,c,6);
        hold on;
        plot(rpe_timestamps, position_rpes(:, 2), 'color', colour);
        subplot(r,c,9);
        hold on;
        plot(rpe_timestamps, position_rpes(:, 3), 'color', colour);
        subplot(r,c,12);
        hold on;
        plot(rpe_timestamps, position_rpes_norm, 'color', colour);

        % Orientation
        set(0, "CurrentFigure", fig_orientation);
        subplot(r,c,1);
        hold on;
        plot(timestamps, rots_axang(:, 1), 'color', colour);
        subplot(r,c,4);
        hold on;
        plot(timestamps, rots_axang(:, 2), 'color', colour);
        subplot(r,c,7);
        hold on;
        plot(timestamps, rots_axang(:, 3), 'color', colour);
        subplot(r,c,10);
        hold on;
        plot(timestamps, angles, 'color', colour);

        subplot(r,c,2);
        hold on;
        plot(ape_timestamps, rot_axang_apes(:, 1), 'color', colour);
        subplot(r,c,5);
        hold on;
        plot(ape_timestamps, rot_axang_apes(:, 2), 'color', colour);
        subplot(r,c,8);
        hold on;
        plot(ape_timestamps, rot_axang_apes(:, 3), 'color', colour);
        subplot(r,c,11);
        hold on;
        plot(ape_timestamps, rot_axang_apes_norm, 'color', colour);

        subplot(r,c,3);
        hold on;
        plot(rpe_timestamps, rot_axang_rpes(:, 1), 'color', colour);
        subplot(r,c,6);
        hold on;
        plot(rpe_timestamps, rot_axang_rpes(:, 2), 'color', colour);
        subplot(r,c,9);
        hold on;
        plot(rpe_timestamps, rot_axang_rpes(:, 3), 'color', colour);
        subplot(r,c,12);
        hold on;
        plot(rpe_timestamps, rot_axang_rpes_norm, 'color', colour);
    end

    % Legend
    for fig = [fig_position, fig_orientation]
        set(0, "CurrentFigure", fig);
        for i = 1:3:10
            subplot(r,c,i);
            legend(names_with_gt, "Location", "northwest");
        end
        for i = [2:3:11, 3:3:12]
            subplot(r,c,i);
            legend(pose_data.names, "Location", "northwest");
        end
    end
end