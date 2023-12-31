function [fig_core_stats, fig_extra_stats, fig_match_id_stats] = ...
    plot_statistics(statistics_data, plot_opts)
    % Configuration
    x_axes = cell(statistics_data.num_stats, 1);
    if plot_opts.use_frame_numbers
        x_label = "frame";
        for i = 1:statistics_data.num_stats
            x_axes{i} = statistics_data.entries{i}.frame_numbers;
        end
    else
        x_label = "t (s)";
        for i = 1:statistics_data.num_stats
            x_axes{i} = statistics_data.entries{i}.timestamps;
        end
    end

    % Core figure
    core_y_labels = ["max match id", "total match count", ...
        "tracked match count", "new match count", ...
        "frame number", "track retention fraction"];
    fig_core_stats = figure("Position", plot_opts.figure_dims, "Name", ...
        "Statistics");
    set(0, "CurrentFigure", fig_core_stats);
    core_r = 3;
    core_c = 2;
    title("Core Statistics");
    for i = 1:core_r*core_c
        subplot(core_r,core_c,i);
        xlabel(x_label);
        ylabel(core_y_labels(i));
        grid on;
    end

    % Extra figure
    extra_y_labels = [...
        "tracked kp count left", "tracked kp count right", ...
        "extracted kp count left", "extracted kp count right", ...
        "filtered extracted kp count left", ...
        "filtered extracted kp count right"];
    fig_extra_stats = figure("Position", plot_opts.figure_dims, "Name", ...
        "Extra Statistics");
    set(0, "CurrentFigure", fig_extra_stats);
    extra_r = 3;
    extra_c = 2;
    title("Extra Statistics");
    for i = 1:extra_r*extra_c
        subplot(extra_r,extra_c,i);
        if mod(i, 2) == 1
            title("Left");
        else
            title("Right");
        end
        xlabel(x_label);
        ylabel(extra_y_labels(i));
        grid on;
    end

    % Match id figure
    fig_match_id_stats = figure("Position", plot_opts.figure_dims, ...
        "Name", "Match Id Statistics");
    set(0, "CurrentFigure", fig_match_id_stats);
    mi_r = 1;
    mi_c = 2;
    title("Match Id Statistics");
    subplot(mi_r,mi_c,1);
    xlabel(x_label);
    ylabel("frames tracked");
    zlabel("feature count");
    subplot(mi_r,mi_c,2);
    xlabel(x_label);
    ylabel("mean/min/max frames tracked");
    grid on;

    % Plot all statistics
    for i = 1:statistics_data.num_stats
        colour = plot_opts.colours{i};
        entry = statistics_data.entries{i};
        x_axis = x_axes{i};

        % Max match id
        set(0, "CurrentFigure", fig_core_stats);
        subplot(core_r,core_c,1);
        hold on;
        plot(x_axis, entry.max_match_ids, 'color', colour);
        % Total match count
        subplot(core_r,core_c,2);
        hold on;
        plot(x_axis, entry.total_match_counts, 'color', colour);
        % Tracked match count
        subplot(core_r,core_c,3);
        hold on;
        plot(x_axis, entry.tracked_match_counts, 'color', colour);
        % New match count
        subplot(core_r,core_c,4);
        hold on;
        plot(x_axis, entry.new_match_counts, 'color', colour);
        % Timestamps/Frame numbers
        subplot(core_r,core_c,5);
        hold on;
        if plot_opts.use_frame_numbers
            plot(x_axis, entry.timestamps, 'color', colour);
        else
            plot(x_axis, entry.frame_numbers, 'color', colour);
        end
        % Track Retention Fraction
        subplot(core_r,core_c,6);
        hold on;
        plot(x_axis, entry.track_retention_fractions, 'color', colour);

        set(0, "CurrentFigure", fig_extra_stats);
        % Tracked Kp Counts
        subplot(extra_r,extra_c,1);
        hold on;
        plot(x_axis, entry.tracked_kp_counts.left, 'color', colour);
        subplot(extra_r,extra_c,2);
        hold on;
        plot(x_axis, entry.tracked_kp_counts.right, 'color', colour);
        % Extracted Kp Counts
        subplot(extra_r,extra_c,3);
        hold on;
        plot(x_axis, entry.extracted_kp_counts.left, 'color', colour);
        subplot(extra_r,extra_c,4);
        hold on;
        plot(x_axis, entry.extracted_kp_counts.right, 'color', colour);
        % Filtered Extracted Kp Counts
        subplot(extra_r,extra_c,5);
        hold on;
        plot(x_axis, entry.filtered_extracted_kp_counts.left, 'color', ...
            colour);
        subplot(extra_r,extra_c,6);
        hold on;
        plot(x_axis, entry.filtered_extracted_kp_counts.right, 'color', ...
            colour);

        set(0, "CurrentFigure", fig_match_id_stats);
        % Tracked match histogram
        subplot(mi_r,mi_c,1);
        hold on;
        [Y, T] = meshgrid(1:entry.max_track_duration, entry.timestamps);
        surf(T, Y, entry.frames_tracked_bins, 'LineStyle', 'none', ...
            'EdgeColor', 'interp', 'FaceColor', 'interp');
        axis tight;
        subplot(mi_r,mi_c,2);
        hold on;
        plot(x_axis, entry.mean_frames_tracked, 'color', colour);
        plot(x_axis, entry.min_frames_tracked, 'color', colour, ...
            'LineStyle', '--');
        plot(x_axis, entry.max_frames_tracked, 'color', colour, ...
            'LineStyle', '--');
    end

    % Legend
    set(0, "CurrentFigure", fig_core_stats);
    for i = 1:core_r*core_c
        subplot(core_r,core_c,i);
        legend(statistics_data.names, "Location", "best");
    end
    set(0, "CurrentFigure", fig_extra_stats);
    for i = 1:extra_r*extra_c
        subplot(extra_r,extra_c,i);
        legend(statistics_data.names, "Location", "best");
    end
    set(0, "CurrentFigure", fig_match_id_stats);
    subplot(mi_r,mi_c,2);
    mmm_legend = cell(3*length(statistics_data.names), 1);
    for i = 1:length(statistics_data.names)
        name = statistics_data.names{i};
        mmm_legend{3*(i-1)+1} = join([name, "mean"], " ");
        mmm_legend{3*(i-1)+2} = join([name, "min"], " ");
        mmm_legend{3*(i-1)+3} = join([name, "max"], " ");
    end
    legend(mmm_legend, "Location", "best");
end