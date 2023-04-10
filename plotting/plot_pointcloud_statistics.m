function fig = plot_pointcloud_statistics(raw_pointcloud_sets, plot_opts)
    % Compute
    pointcloud_sets = cell(length(raw_pointcloud_sets), 1);
    for i = 1:length(raw_pointcloud_sets)
        raw_pointclouds = raw_pointcloud_sets{i};
        pointclouds = cell(length(raw_pointclouds), 1);
        for j = 1:length(raw_pointclouds)
            pointclouds{j} = from_PointCloud2(raw_pointclouds{j});
        end
        pointcloud_sets{i} = pointclouds;
    end
    fields = fieldnames(pointcloud_sets{1}{1});

    % Create figure
    fig = figure("Position", plot_opts.figure_dims, "Name", ...
        "PointCloud Statistics");
    set(0, "CurrentFigure", fig);
    r = floor(sqrt(length(fields)));
    c = ceil(sqrt(length(fields)));
    title("PointCloud Statistics");
    for i = 1:length(fields)
        subplot(r,c,i);
        xlabel("t (s)");
        ylabel(fields{i});
        grid on;
    end

    % Iterate over pointcloud sets
    for i = 1:length(pointcloud_sets)
        timestamps = extract_timestamps(raw_pointcloud_sets{i});
        if plot_opts.start_from_time_zero
            timestamps = timestamps - timestamps(1);
        end
        pointclouds = pointcloud_sets{i};
        for f = 1:length(fields)
            field_data = zeros(0, 1);
            timestamp_groups = zeros(0, 1);
            for j = 1:length(pointclouds)
                field_data = vertcat(field_data, ...
                    pointclouds{j}.(fields{f}));
                timestamp_groups = vertcat(timestamp_groups, repmat( ...
                    timestamps(j), length(pointclouds{j}.(fields{f})), 1));
            end
            subplot(r,c,f);
            hold on;
            boxplot(field_data, timestamp_groups);
            ax = gca;
            ax.XTick = ax.XTick(1:round(length(pointclouds)/10):end);
        end
    end
end