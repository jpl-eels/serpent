function data = data_from_rosbags(config)
    data = struct;
    data.gt = struct;
    data.gt.accelerometer_bias = config.gt.accelerometer_bias;
    data.gt.gyroscope_bias = config.gt.gyroscope_bias;
    data.names = config.names;

    filepaths = config.data_dir + config.filenames;
    gt_filepath = config.gt.data_dir + config.gt.filename;
    
    gt_bag = rosbag(gt_filepath);
    gt_bag_select = select(gt_bag, "Topic", config.gt.topic, ...
        "MessageType", "nav_msgs/Odometry");
    data.gt.odom = readMessages(gt_bag_select, 'DataFormat', 'struct');
    
    data.odoms = cell(length(filepaths), 1);
    for i = 1:length(filepaths)
        bag = rosbag(filepaths(i));
        bag_select = select(bag, "Topic", config.odom_topics(i), ...
            "MessageType", "nav_msgs/Odometry");
        data.odoms{i} = readMessages(bag_select, 'DataFormat', 'struct');
    end
end