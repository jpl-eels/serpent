function plot_and_save(data, plot_opts)
    pose_data = compute_pose_data(data);
    fprintf("Finished computing pose data.\n");
    [fig_position, fig_orientation] = plot_pose(pose_data, plot_opts);
    fprintf("Finished plotting pose data.\n");
    twist_data = compute_twist_data(data);
    fprintf("Finished computing twist data.\n");
    [fig_lin_vel, fig_ang_vel] = plot_twist(twist_data, plot_opts);
    fprintf("Finished plotting twist data.\n");
    if plot_opts.save
        [mkdir_status, msg, msgID] = mkdir(plot_opts.save_dir);
        if mkdir_status
            fprintf("Save directory: %s\n", plot_opts.save_dir);
        else
            fprintf('Failed to create directory [msgID: %s, msg: %s\n', ...
                msgID, msg);
        end
        for filetype = plot_opts.filetypes
            saveas(fig_position, join([plot_opts.save_dir, "position"], ...
                ""), filetype);
            saveas(fig_orientation, join([plot_opts.save_dir, ...
                "orientation"], ""), filetype);
            saveas(fig_lin_vel, join([plot_opts.save_dir, ...
                "linear_velocity"], ""), filetype);
            saveas(fig_ang_vel, join([plot_opts.save_dir, ...
                "angular_velocity"], ""), filetype);
            fprintf("Finished saving figures as .%s\n", filetype);
        end
        if plot_opts.close_after_save
            close(fig_position);
            close(fig_orientation);
            close(fig_lin_vel);
            close(fig_ang_vel);
        end
    end
end