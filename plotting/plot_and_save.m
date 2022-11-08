function [pose_data, twist_data] = plot_and_save(data, plot_opts)
    pose_data = compute_pose_data(data, plot_opts.align_first_pose);
    if plot_opts.start_from_time_zero
        start_time = pose_data.gt.timestamps(1);
        pose_data.gt.timestamps = pose_data.gt.timestamps - start_time;
        for i = 1:length(pose_data.entries)
            pose_data.entries{i}.timestamps = ...
                pose_data.entries{i}.timestamps - start_time;
            pose_data.entries{i}.ape.timestamps = ...
                pose_data.entries{i}.ape.timestamps - start_time;
            pose_data.entries{i}.rpe.timestamps = ...
                pose_data.entries{i}.rpe.timestamps - start_time;

        end
    end
    fprintf("Finished computing pose data.\n");

    twist_data = compute_twist_data(data);
    if plot_opts.start_from_time_zero
        start_time = twist_data.gt.timestamps(1);
        twist_data.gt.timestamps = twist_data.gt.timestamps - start_time;
        for i = 1:length(twist_data.entries)
            twist_data.entries{i}.timestamps = ...
                twist_data.entries{i}.timestamps - start_time;
            twist_data.entries{i}.ae.timestamps = ...
                twist_data.entries{i}.ae.timestamps - start_time;
            twist_data.entries{i}.re.timestamps = ...
                twist_data.entries{i}.re.timestamps - start_time;

        end
    end
    fprintf("Finished computing twist data.\n");

    [fig_position, fig_orientation] = plot_pose(pose_data, plot_opts);
    fprintf("Finished plotting pose data.\n");

    [fig_lin_vel, fig_ang_vel] = plot_twist(twist_data, plot_opts);
    fprintf("Finished plotting twist data.\n");

    [fig_trajectory, fig_trajectory_orientation, ...
        fig_trajectory_velocity] = plot_trajectory(pose_data, ...
        twist_data, plot_opts);

    fids = 1;
    if plot_opts.save
        create_save_directory(plot_opts.save_dir);
        fids(2) = fopen(join([plot_opts.save_dir, "data.txt"], ""), 'w');

        for filetype = plot_opts.filetypes
            saveas(fig_position, join([plot_opts.save_dir, "position"], ...
                ""), filetype);
            saveas(fig_orientation, join([plot_opts.save_dir, ...
                "orientation"], ""), filetype);
            saveas(fig_lin_vel, join([plot_opts.save_dir, ...
                "linear_velocity"], ""), filetype);
            saveas(fig_ang_vel, join([plot_opts.save_dir, ...
                "angular_velocity"], ""), filetype);
            saveas(fig_trajectory, join([plot_opts.save_dir, ...
                "trajectory"], ""), filetype);
            saveas(fig_trajectory_orientation, join([plot_opts.save_dir, ...
                "trajectory orientation"], ""), filetype);
            saveas(fig_trajectory_velocity, join([plot_opts.save_dir, ...
                "trajectory velocity"], ""), filetype);
            fprintf("Finished saving figures as .%s\n", filetype);
        end
        if plot_opts.close_after_save
            close(fig_position);
            close(fig_orientation);
            close(fig_lin_vel);
            close(fig_ang_vel);
            close(fig_trajectory);
            close(fig_trajectory_orientation);
            close(fig_trajectory_velocity);
        end
    end

    fmt_str = ['%30s: %.', num2str(plot_opts.summary_decimal_places), 'f'];
    for fid = fids
        for i = 1:length(data.names)
            fprintf(fid, "%s Summary:\n", data.names(i));
            fprintf(fid, [fmt_str, ' m\n'], ...
                "Final Translation ||APE||", ...
                pose_data.entries{i}.ape_norm.position(end));
            fprintf(fid, [fmt_str, ' rad\n'], ...
                "Final Orientation ||APE||", ...
                pose_data.entries{i}.ape_norm.rot_axang(end));
            fprintf(fid, [fmt_str, ' m\n'], ...
                "Translation APE RMSE", ...
                pose_data.entries{i}.ape_norm.position_rmse);
            fprintf(fid, [fmt_str, ' rad\n'], ...
                "Orientation APE RMSE", ...
                pose_data.entries{i}.ape_norm.rot_axang_rmse);
            fprintf(fid, [fmt_str, ' m/s\n'], ...
                "Final Linear Velocity ||APE||", ...
                twist_data.entries{i}.ae_norm.linear_velocity(end));
            fprintf(fid, [fmt_str, ' rad/s\n'], ...
                "Final Angular Velocity ||APE||", ...
                twist_data.entries{i}.ae_norm.angular_velocity(end));
            fprintf(fid, [fmt_str, ' m/s\n'], ...
                "Linear Velocity APE RMSE", ...
                twist_data.entries{i}.ae_norm.linear_velocity_rmse);
            fprintf(fid, [fmt_str, ' rad/s\n'], ...
                "Angular Velocity APE RMSE", ...
                twist_data.entries{i}.ae_norm.angular_velocity_rmse);
        end
    end
end