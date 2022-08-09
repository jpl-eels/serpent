function data = stereo_statistics_from_rosbags(config)
    data = struct;
    data.names = config.names;
    filepaths = config.data_dir + config.filenames;
    data.statistics = cell(length(filepaths), 1);
    for i = 1:length(filepaths)
        bag = rosbag(filepaths(i));
        bag_select = select(bag, "Topic", config.stats_topics(i), ...
            "MessageType", "serpent/StereoTrackerStatistics");
        if bag_select.NumMessages == 0
            error(join(["0 msgs found in bag ", filepaths(i), ...
                " for topic ", config.stats_topics(i)], ""));
        end
        data.statistics{i} = readMessages(bag_select, 'DataFormat', ...
            'struct');
    end
end