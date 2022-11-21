function jacobians = plot_and_save_jacobians(jacobian_data, plot_opts)
    jacobians = compute_jacobians_data(jacobian_data);

    [fig_jacobian, fig_abs_jacobian, fig_squared_jacobian] = ...
        plot_jacobian(jacobians, plot_opts);

    if plot_opts.save
        create_save_directory(plot_opts.save_dir);

        for filetype = plot_opts.filetypes
            saveas(fig_jacobian, join([plot_opts.save_dir, ...
                "jacobian"], ""), filetype);
            saveas(fig_abs_jacobian, join([plot_opts.save_dir, ...
                "abs_jacobian"], ""), filetype);
            saveas(fig_squared_jacobian, join([plot_opts.save_dir, ...
                "squared_jacobian"], ""), filetype);
            fprintf("Finished saving figures as .%s\n", filetype);
        end
    end
    if plot_opts.close_figures
        close(fig_jacobian);
    end
end