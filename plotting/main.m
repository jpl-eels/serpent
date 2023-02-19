clc;
clear;
close all;

% Configuration
config = struct;
config.gt = struct;
data_base_dir = join([getenv("HOME"), "/data/jpl/"], "");
data_source = "eels1/";
data_name = "athabasca_005";
config.gt.data_dir = join([data_base_dir, "results/", data_source, ...
    data_name, "/"], "");
config.gt.filename = "locus_gt_odometry_bodyframe_20s_to_100s_2023-02-18-20-52-03.bag";
config.gt.topic = "/eels1/lo_frontend/odometry";
config.gt.accelerometer_bias = [0.0, 0.0, 0.0];
config.gt.gyroscope_bias = [0.0, 0.0, 0.0];
config.data_dir = join([data_base_dir, "results/", data_source, ...
    data_name, "/"], "");
analysis_name = "feb_19_analysis";
config.filenames = ["locus_2023-02-18-21-10-10.bag", ...
    "liosam_2023-02-18-22-53-59.bag", ...
    "serpent_constant_1deg1cm_2023-02-19-12-44-49.bag", ...
    "serpent_lls_constant_2023-02-19-12-37-18.bag", ...
    "serpent_censi_constant_2023-02-19-12-42-12.bag", ...
    "serpent_censi_range_2023-02-19-12-33-40.bag", ...
    "serpent_censi_range_postchange2_2023-02-19-23-48-35.bag"];
config.names = ["locus", "liosam", "serpent constant", ...
    "serpent i.i.d. LLS", "serpent i.i.d. Censi", "serpent Range Censi",...
    "serpent Range Censi (postchange)"];
config.odom_topics = ["/eels1/lo_frontend/odometry", "/odometry/imu", ...
    repmat("/serpent/optimisation/odometry", 1, 5)];
config.reg_tf_topics = ["", "", ...
    repmat("/serpent/registration/transform", 1, 5)];
% config.track_points_topics = repmat( ...
%     "/serpent/stereo/track_points_with_distances", 1, ...
%     length(config.names));
config.track_points_statistics_topics = ["", "", ...
    repmat("/serpent/stereo/track_points_with_distances/statistics", 1, 5)];
config.filepaths = config.data_dir + config.filenames;

% Plot Options
plot_opts = default_plot_opts();
plot_opts.save = true;
plot_opts.save_dir = join([data_base_dir, "analysis/", data_source, ...
    data_name, "/", analysis_name, "/"], "");
plot_opts.close_figures = false;      

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
