clc;
clear;
close all;

% Configuration
config = struct;
config.gt = struct;
data_base_dir = join([getenv("HOME"), "/data/jpl/"], "");
data_source = "simulator_prcp_gazebo/";
data_name = "surface_ellipse_2";
config.gt.data_dir = join([data_base_dir, "datasets/", data_source], "");
config.gt.filename = join([data_name, ".bag"], "");
config.gt.topic = "/ground_truth";
config.gt.accelerometer_bias = [0.0, 0.0, 0.0];
config.gt.gyroscope_bias = [0.0, 0.0, 0.0];
config.data_dir = join([data_base_dir, "results/", data_source, data_name, ...
    "/"], "");
analysis_name = "linear_velocity_bugfix";
config.filenames = ["s2s_2022-10-06-16-01-16.bag", ...
    "s2s_imu_2022-10-06-12-16-18.bag"];
config.names = ["s2s", "s2s imu"];
config.odom_topics = repmat("/serpent/optimisation/odometry", 1, ...
    length(config.names));

% Plot Options
plot_opts = default_plot_opts();
plot_opts.save = true;
plot_opts.save_dir = join([data_base_dir, "analysis/", data_source, ...
    data_name, "/", analysis_name, "/"], "");

data = data_from_rosbags(config);

[pose_data, twist_data] = plot_and_save(data, plot_opts);
