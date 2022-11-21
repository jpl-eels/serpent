function data = raw_data_from_rosbags(filepaths, topics, msg_type)
    data = cell(length(filepaths), 1);
    for i = 1:length(filepaths)
        bag = rosbag(filepaths(i));
        bag_select = select(bag, "Topic", topics(i), "MessageType", ...
            msg_type);
        if bag_select.NumMessages == 0
            error(join(["0 msgs found in bag ", filepaths(i), ...
                " for topic ", topics(i), " and MessageType ", ...
                msg_type], ""));
        end
        data{i} = readMessages(bag_select, 'DataFormat', ...
            'struct');
    end
end