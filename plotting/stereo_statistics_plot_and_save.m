function statistics_data = stereo_statistics_plot_and_save(data, plot_opts)
    % Convert struct data to arrays
    statistics_data = extract_statistics(data);
    fprintf("Finished computing statistics.\n");

    % Growth of features
    [fig_core_stats, fig_extra_stats, fig_match_id_stats] = ...
        plot_statistics(statistics_data, plot_opts);
    fprintf("Finished plotting statistics.\n")

    fids = 1;
    if plot_opts.save
        create_save_directory(plot_opts.save_dir);
        fids(2) = fopen(join([plot_opts.save_dir, "data.txt"], ""), 'w');

        for filetype = plot_opts.filetypes
            saveas(fig_core_stats, join([plot_opts.save_dir, ...
                "core_statistics"], ""), filetype);
            saveas(fig_extra_stats, join([plot_opts.save_dir, ...
                "extra_statistics"], ""), filetype);
            saveas(fig_match_id_stats, join([plot_opts.save_dir, ...
                "match_id_statistics"], ""), filetype);
            fprintf("Finished saving figures as .%s\n", filetype);
        end
    end
    if plot_opts.close_figures
        close(fig_core_stats);
        close(fig_extra_stats);
        close(fig_match_id_stats);
    end

    fmt_str = ['%30s: %.', num2str(plot_opts.summary_decimal_places), 'f'];
    for fid = fids
        for i = 1:length(statistics_data.names)
            entry = statistics_data.entries{i};
            fprintf(fid, "%s Summary:\n", statistics_data.names(i));
            fprintf(fid, [fmt_str, '\n'], "Mean Track Retention", ...
                entry.mean_track_retention_fraction);
            fprintf(fid, [fmt_str, ' frames\n'], "Max Track Duration", ...
                entry.max_track_duration);
        end
    end
end