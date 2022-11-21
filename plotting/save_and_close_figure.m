function save_and_close_figure(fig, name, plot_opts)
    if plot_opts.save
        create_save_directory(plot_opts.save_dir);
        for filetype = plot_opts.filetypes
            filename = join([plot_opts.save_dir, name, ".", filetype], "");
            saveas(fig, filename);
            fprintf(join(["Saved figure \'", filename, "\'\n"], ""));
        end
    end
    if plot_opts.close_figures
        close(fig);
    end
end