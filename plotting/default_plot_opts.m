function plot_opts = default_plot_opts()
    plot_opts.align_first_pose = true;
    plot_opts.align_trajectories = false;
    plot_opts.align_opt_params = struct;
    plot_opts.align_opt_params.gamma = 0.95;
    plot_opts.align_opt_params.eta = 1.0e-4;
    plot_opts.align_opt_params.iterations = 1000;
    plot_opts.align_opt_params.max_r_change = deg2rad(5);
    plot_opts.align_opt_params.max_t_change = 0.1;
    plot_opts.start_from_time_zero = true;
    plot_opts.use_frame_numbers = false;
    plot_opts.summary_decimal_places = 3;
    plot_opts.angle = "deg";
    plot_opts.figure_dims = [0, 0, 2400, 1600];
    plot_opts.colours = {[0 0.4470 0.7410], [0.8500 0.3250 0.0980], ...
        [0.9290 0.6940 0.1250], [0.4940 0.1840 0.5560], ...
        [0.4660 0.6740 0.1880], [0.3010 0.7450 0.9330],[0.6350 0.0780 0.1840]};
    plot_opts.gt_linespec = 'k--';
    plot_opts.save = false;
    plot_opts.save_dir = "";
    plot_opts.filetypes = ["fig", "png", "svg"];
    plot_opts.close_figures = false;
end