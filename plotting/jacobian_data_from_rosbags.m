function data = jacobian_data_from_rosbags(config, topic_name)
    data = struct;
    data.names = config.names;
    filepaths = config.data_dir + config.filenames;
    
    data.jacobians = cell(length(filepaths), 1);
    for i = 1:length(filepaths)
        bag = rosbag(filepaths(i));
        bag_select = select(bag, "Topic", topic_name, "MessageType", ...
            "std_msgs/Float64MultiArray");
        if bag_select.NumMessages == 0
            error(join(["0 msgs found in bag ", filepaths(i), ...
                " for topic ", topic_name], ""));
        end
        data.jacobians{i} = readMessages(bag_select, 'DataFormat', ...
            'struct');
    end
end