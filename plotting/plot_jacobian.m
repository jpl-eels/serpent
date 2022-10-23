function [fig_jacobian, fig_abs_jacobian, fig_squared_jacobian] = plot_jacobian(...
    jacobians, plot_opts)

    fig_jacobian = figure("Position", plot_opts.figure_dims, "Name", ...
        "Jacobian");
    set(0, "CurrentFigure", fig_jacobian);
    r = 2;
    c = round(jacobians.entries{1}.jacobian_dim/2);
    title("Jacobian");
    for i = 1:jacobians.entries{1}.jacobian_dim
        subplot(r,c,i);
        xlabel("idx");
        ylabel(plot_opts.axis_labels(1, i));
        if isfield(plot_opts, "axis_limits")
            ylim(plot_opts.axis_limits(1, :));
        end
        grid on;
    end

    fig_abs_jacobian = figure("Position", plot_opts.figure_dims, "Name", ...
        "Absolute Value Jacobian");
    set(0, "CurrentFigure", fig_abs_jacobian);
    r = 2;
    c = round(jacobians.entries{1}.jacobian_dim/2);
    title("Absolute Value Jacobian");
    for i = 1:jacobians.entries{1}.jacobian_dim
        subplot(r,c,i);
        xlabel("idx");
        ylabel(plot_opts.axis_labels(2, i));
        if isfield(plot_opts, "axis_limits")
            ylim(plot_opts.axis_limits(2, :));
        end
        grid on;
    end

    fig_squared_jacobian = figure("Position", plot_opts.figure_dims, ...
        "Name", "Squared Jacobian");
    set(0, "CurrentFigure", fig_squared_jacobian);
    r = 2;
    c = round(jacobians.entries{1}.jacobian_dim/2);
    title("Squared Jacobian");
    for i = 1:jacobians.entries{1}.jacobian_dim
        subplot(r,c,i);
        xlabel("idx");
        ylabel(plot_opts.axis_labels(3, i));
        if isfield(plot_opts, "axis_limits")
            ylim(plot_opts.axis_limits(3, :));
        end
        grid on;
    end

    % Plot Jacobians
    for i = 1:jacobians.num_entries
        colour = plot_opts.colours{i};
        entry = jacobians.entries{i};
        for j = 1:entry.jacobian_dim
            set(0, "CurrentFigure", fig_jacobian);
            subplot(r,c,j);
            hold on;
            plot(entry.jacobians(:, j), 'color', colour);

            set(0, "CurrentFigure", fig_abs_jacobian);
            subplot(r,c,j);
            hold on;
            plot(entry.abs_jacobians(:, j), 'color', colour);

            set(0, "CurrentFigure", fig_squared_jacobian);
            subplot(r,c,j);
            hold on;
            plot(entry.squared_jacobians(:, j), 'color', colour);
        end
    end

    % Legend
    for i = 1:entry.jacobian_dim
        subplot(r,c,i);
        legend(jacobians.names, "Location", "best");
    end
end