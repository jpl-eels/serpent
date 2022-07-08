function [fig_lin_vel, fig_ang_vel] = plot_twist(twist_data, ...
        plot_opts)
    % Computation
    names_with_gt = ["Reference", twist_data.names];
    gt_angular_velocities = twist_data.gt.angular_velocities;
    if plot_opts.angle == "deg"
        gt_angular_velocities = rad2deg(gt_angular_velocities);
    end

    % Reference Linear
    fig_lin_vel = figure("Position", plot_opts.figure_dims, "Name", ...
        "Linear Velocity");
    set(0, "CurrentFigure", fig_lin_vel);
    r = 4;
    c = 3;
    title("Linear Velocity");
    subplot(r,c,1);
    plot(twist_data.gt.timestamps, twist_data.gt.linear_velocities(:, 1));
    xlabel("t (s)");
    ylabel("v_x (m/s)");
    grid on;
    subplot(r,c,2);
    xlabel("t (s)");
    ylabel("AE v_x (m/s)");
    grid on;
    subplot(r,c,3);
    xlabel("t (s)");
    ylabel("RE v_x (m/s)");
    grid on;
    subplot(r,c,4);
    plot(twist_data.gt.timestamps, twist_data.gt.linear_velocities(:, 2));
    xlabel("t (s)");
    ylabel("v_y (m/s)");
    grid on;
    subplot(r,c,5);
    xlabel("t (s)");
    ylabel("AE v_y (m/s)");
    grid on;
    subplot(r,c,6);
    xlabel("t (s)");
    ylabel("RE v_y (m/s)");
    grid on;
    subplot(r,c,7);
    plot(twist_data.gt.timestamps, twist_data.gt.linear_velocities(:, 3));
    xlabel("t (s)");
    ylabel("v_z (m/s)");
    grid on;
    subplot(r,c,8);
    xlabel("t (s)");
    ylabel("AE v_z (m/s)");
    grid on;
    subplot(r,c,9);
    xlabel("t (s)");
    ylabel("RE v_z (m/s)");
    grid on;
    subplot(r,c,10);
    plot(twist_data.gt.timestamps, twist_data.gt.linear_speeds);
    xlabel("t (s)");
    ylabel("Speed (m/s)");
    grid on;
    subplot(r,c,11);
    xlabel("t (s)");
    ylabel("AE (m/s)");
    grid on;
    subplot(r,c,12);
    xlabel("t (s)");
    ylabel("RE (m/s)");
    grid on;

    % Reference Angular
    fig_ang_vel = figure("Position", plot_opts.figure_dims, "Name", ...
        "Angular Velocity");
    set(0, "CurrentFigure", fig_ang_vel);
    r = 4;
    c = 3;
    title("Orientation");
    subplot(r,c,1);
    plot(twist_data.gt.timestamps, gt_angular_velocities(:, 1));
    xlabel("t (s)");
    ylabel(join(["\omega_x (", plot_opts.angle, "/s)"]));
    grid on;
    subplot(r,c,2);
    xlabel("t (s)");
    ylabel(join(["AE \omega_x (", plot_opts.angle, "/s)"]));
    grid on;
    subplot(r,c,3);
    xlabel("t (s)");
    ylabel(join(["RE \omega_x (", plot_opts.angle, "/s)"]));
    grid on;
    subplot(r,c,4);
    plot(twist_data.gt.timestamps, gt_angular_velocities(:, 2));
    xlabel("t (s)");
    ylabel(join(["\omega_y (", plot_opts.angle, "/s)"]));
    grid on;
    subplot(r,c,5);
    xlabel("t (s)");
    ylabel(join(["AE \omega_y (", plot_opts.angle, "/s)"]));
    grid on;
    subplot(r,c,6);
    xlabel("t (s)");
    ylabel(join(["RE \omega_y (", plot_opts.angle, "/s)"]));
    grid on;
    subplot(r,c,7);
    plot(twist_data.gt.timestamps, gt_angular_velocities(:, 3));
    xlabel("t (s)");
    ylabel(join(["\omega_z (", plot_opts.angle, "/s)"]));
    grid on;
    subplot(r,c,8);
    xlabel("t (s)");
    ylabel(join(["APE \omega_z (", plot_opts.angle, "/s)"]));
    grid on;
    subplot(r,c,9);
    xlabel("t (s)");
    ylabel(join(["RE \omega_z (", plot_opts.angle, "/s)"]));
    grid on;
    subplot(r,c,10);
    plot(twist_data.gt.timestamps, twist_data.gt.angular_speeds);
    xlabel("t (s)");
    ylabel(join(["Angular Speed (", plot_opts.angle, "/s)"]));
    grid on;
    subplot(r,c,11);
    xlabel("t (s)");
    ylabel(join(["APE (", plot_opts.angle, "/s)"]));
    grid on;
    subplot(r,c,12);
    xlabel("t (s)");
    ylabel(join(["RE (", plot_opts.angle, "/s)"]));
    grid on;

    % Query Twists
    for i = 1:twist_data.num_odom
        entry = twist_data.entries{i};
        timestamps = entry.timestamps;
        ae_timestamps = entry.ae.timestamps;
        re_timestamps = entry.re.timestamps;
        linear_velocities = entry.linear_velocities;
        linear_speeds = entry.linear_speeds;
        linear_velocity_aes = entry.ae.linear_velocities;
        linear_velocity_ae_norms = entry.ae_norm.linear_velocity;
        linear_velocity_res = entry.re.linear_velocities;
        linear_velocity_re_norms = entry.re_norm.linear_velocity;
        angular_velocities = entry.angular_velocities;
        angular_speeds = entry.angular_speeds;
        angular_velocity_aes = entry.ae.angular_velocities;
        angular_velocity_ae_norms = entry.ae_norm.angular_velocity;
        angular_velocity_res = entry.re.angular_velocities;
        angular_velocity_re_norms = entry.re_norm.angular_velocity;
        if plot_opts.angle == "deg"
            angular_velocities = rad2deg(angular_velocities);
            angular_velocity_aes = rad2deg(angular_velocity_aes);
            angular_velocity_ae_norms = rad2deg(angular_velocity_ae_norms);
        end

        % Linear Velocity
        set(0, "CurrentFigure", fig_lin_vel);
        subplot(r,c,1);
        hold on;
        plot(timestamps, linear_velocities(:, 1));
        subplot(r,c,4);
        hold on;
        plot(timestamps, linear_velocities(:, 2));
        subplot(r,c,7);
        hold on;
        plot(timestamps, linear_velocities(:, 3));
        subplot(r,c,10);
        hold on;
        plot(timestamps, linear_speeds);

        subplot(r,c,2);
        hold on;
        plot(ae_timestamps, linear_velocity_aes(:, 1));
        subplot(r,c,5);
        hold on;
        plot(ae_timestamps, linear_velocity_aes(:, 2));
        subplot(r,c,8);
        hold on;
        plot(ae_timestamps, linear_velocity_aes(:, 3));
        subplot(r,c,11);
        hold on;
        plot(ae_timestamps, linear_velocity_ae_norms);

        subplot(r,c,3);
        hold on;
        plot(re_timestamps, linear_velocity_res(:, 1));
        subplot(r,c,6);
        hold on;
        plot(re_timestamps, linear_velocity_res(:, 2));
        subplot(r,c,9);
        hold on;
        plot(re_timestamps, linear_velocity_res(:, 3));
        subplot(r,c,12);
        hold on;
        plot(re_timestamps, linear_velocity_re_norms);

        % Orientation
        set(0, "CurrentFigure", fig_ang_vel);
        subplot(r,c,1);
        hold on;
        plot(timestamps, angular_velocities(:, 1));
        subplot(r,c,4);
        hold on;
        plot(timestamps, angular_velocities(:, 2));
        subplot(r,c,7);
        hold on;
        plot(timestamps, angular_velocities(:, 3));
        subplot(r,c,10);
        hold on;
        plot(timestamps, angular_speeds);

        subplot(r,c,2);
        hold on;
        plot(ae_timestamps, angular_velocity_aes(:, 1));
        subplot(r,c,5);
        hold on;
        plot(ae_timestamps, angular_velocity_aes(:, 2));
        subplot(r,c,8);
        hold on;
        plot(ae_timestamps, angular_velocity_aes(:, 3));
        subplot(r,c,11);
        hold on;
        plot(ae_timestamps, angular_velocity_ae_norms);

        subplot(r,c,3);
        hold on;
        plot(re_timestamps, angular_velocity_res(:, 1));
        subplot(r,c,6);
        hold on;
        plot(re_timestamps, angular_velocity_res(:, 2));
        subplot(r,c,9);
        hold on;
        plot(re_timestamps, angular_velocity_res(:, 3));
        subplot(r,c,12);
        hold on;
        plot(re_timestamps, angular_velocity_re_norms);
    end

    % Legend
    for fig = [fig_lin_vel, fig_ang_vel]
        set(0, "CurrentFigure", fig);
        for i = 1:3:10
            subplot(r,c,i);
            legend(names_with_gt, "Location", "northwest");
        end
        for i = [2:3:11, 3:3:12]
            subplot(r,c,i);
            legend(twist_data.names, "Location", "northwest");
        end
    end
end