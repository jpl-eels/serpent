clc;
clear;
close all;

% Configuration
config = struct;
config.gt = struct;
data_base_dir = join([getenv("HOME"), "/data/jpl/"], "");
data_source = "eels1/";
data_name = "athabasca_023";
config.gt.data_dir = join([data_base_dir, "results/", data_source, ...
    data_name, "/"], "");
config.gt.filename = "locus_gt_odometry_bodyframe_2022-12-04-21-24-15.bag";
config.gt.topic = "/eels1/lo_frontend/odometry";
config.gt.accelerometer_bias = [0.0, 0.0, 0.0];
config.gt.gyroscope_bias = [0.0, 0.0, 0.0];
config.data_dir = join([data_base_dir, "results/", data_source, ...
    data_name, "/"], "");
analysis_name = "lidar_imu_fusion_s2m";
config.filenames = ["s2m_p2planelin_2022-12-05-00-37-28.bag", ...
    "s2m_p2planelin_defimunoise_2022-12-05-00-42-26.bag", ...
    "s2m_p2planelin_plus1magimunoise_2022-12-05-00-47-30.bag", ...
    "s2m_p2planelin_plus2magimunoise_2022-12-05-00-54-27.bag"];
config.names = ["s2m", ...
    "s2m-imu-p2plane-lin-default-imu-noise", ...
    "s2m-imu-p2plane-lin-plus1mag-imu-noise", ...
    "s2m-imu-p2plane-lin-plus2mag-imu-noise"];
config.odom_topics = repmat("/serpent/optimisation/odometry", 1, ...
    length(config.names));
config.reg_tf_topics = repmat("/serpent/registration/transform", ...
    1, length(config.names));
config.filepaths = config.data_dir + config.filenames;

% Plot Options
plot_opts = default_plot_opts();
plot_opts.save = true;
plot_opts.save_dir = join([data_base_dir, "analysis/", data_source, ...
    data_name, "/", analysis_name, "/"], "");
plot_opts.close_figures = false;      

% Registration
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

% Odometry
odom_data = odom_data_from_rosbags(config);
[pose_data, twist_data] = plot_and_save(odom_data, plot_opts);