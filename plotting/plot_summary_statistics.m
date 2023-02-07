function fig = plot_summary_statistics(statistics, plot_opts)
    % Computation
    labels = arrayfun(@(s) string(s.Label), statistics{1}{1}.Statistics);
    stats_labels = {'Min', 'Mean', 'Count', 'Max', 'Variance'};

    % Figure
    figs = gobjects(length(labels), 1);
    for f = 1:length(labels)
        label = labels(f);
        fig_label = join([label, "Statistics"], " ");
        figs(f) = figure("Position", plot_opts.figure_dims, "Name", fig_label);
        set(0, "CurrentFigure", figs(f));
        r = 2;
        c = 3;
        title(fig_label);
        for i = 1:length(stats_labels)
            subplot(r,c,i);
            xlabel("t (s)");
            ylabel(stats_labels{i});
            grid on;
        end
            
        % Iterate over the datasets
        for i = 1:length(statistics)
            colour = plot_opts.colours{i};
            timestamps = extract_timestamps(statistics{i});
            if plot_opts.start_from_time_zero
                timestamps = timestamps - timestamps(1);
            end
            for j = 1:length(stats_labels)
                subplot(r,c,j);
                hold on;
                stats_data = zeros(length(statistics{i}), 1);
                for p = 1:length(statistics{i})
                    stats_data(p) = statistics{i}{p}.Statistics(f).(stats_labels{j});
                end
                plot(timestamps, stats_data, 'color', colour);
            end
        end
    end
end