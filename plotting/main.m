clc;
clear;
close all;

% Configuration
config = struct;
config.gt = struct;
data_base_dir = join([getenv("HOME"), "/data/jpl/"], "");
data_source = "eels1/";
data_name = "athabasca_023";
% No GT yet, so use odometry
config.gt.data_dir = join([data_base_dir, "results/", data_source, ...
    data_name, "/"], "");
config.gt.filename = "sad_3x3_2022-11-21-23-27-28.bag";
config.gt.topic = "/serpent/optimisation/odometry";
config.gt.accelerometer_bias = [0.0, 0.0, 0.0];
config.gt.gyroscope_bias = [0.0, 0.0, 0.0];
config.data_dir = join([data_base_dir, "results/", data_source, ...
    data_name, "/"], "");
analysis_name = "track_points_statistics";
config.filenames = "sad_3x3_2022-11-21-23-27-28.bag";
config.names = "sad 3x3";
config.odom_topics = repmat("/serpent/optimisation/odometry", 1, ...
    length(config.names));
config.reg_tf_topics = repmat("/serpent/registration/transform", ...
    1, length(config.names));
% config.track_points_topics = repmat( ...
%     "/serpent/stereo/track_points_with_distances", 1, ...
%     length(config.names));
config.track_points_statistics_topics = repmat( ...
    "/serpent/stereo/track_points_with_distances/statistics", 1, ...
    length(config.names));
config.filepaths = config.data_dir + config.filenames;

% Plot Options
plot_opts = default_plot_opts();
plot_opts.save = true;
plot_opts.save_dir = join([data_base_dir, "analysis/", data_source, ...
    data_name, "/", analysis_name, "/"], "");
plot_opts.close_figures = true;

% Registration
try
    reg_transform_raw_data = raw_data_from_rosbags(config.filepaths, ...
        config.reg_tf_topics, "geometry_msgs/PoseWithCovarianceStamped");
    reg_transform_data = cell(length(config.filepaths), 1);
    for i = 1:length(config.filepaths)
        reg_transform_data{i} = extract_pose_with_covariance_stamped(...
            reg_transform_raw_data{i});
        fig_cov = plot_pose_with_cov(reg_transform_data{i}, plot_opts);
        filename = join(["registration_covariance_", config.names(i)], "");
        save_and_close_figure(fig_cov, filename, plot_opts);
    end
catch ex
    warning("Could not generate analysis for registration poses.");
end

% Stereo track points statistics
try
    track_points_statistics = raw_data_from_rosbags(config.filepaths, ...
        config.track_points_statistics_topics, ...
        "statistics_msgs/SummaryStatisticsArray");
    plot_summary_statistics(track_points_statistics, plot_opts);
%     track_points = raw_data_from_rosbags(config.filepaths, ...
%         config.track_points_topics, "sensor_msgs/PointCloud2");
%     plot_pointcloud_statistics(track_points, plot_opts);
catch ex
    warning(join([ ...
        "Could not generate analysis for track point statistics: ", ...
        ex.message], ""));
end

% Odometry
odom_data = odom_data_from_rosbags(config);
[pose_data, twist_data] = plot_and_save(odom_data, plot_opts);