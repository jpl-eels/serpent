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
config.gt.filename = "s2m_2022-10-23-15-44-50.bag";
config.gt.topic = "/serpent/optimisation/odometry";
config.gt.accelerometer_bias = [0.0, 0.0, 0.0];
config.gt.gyroscope_bias = [0.0, 0.0, 0.0];
config.data_dir = join([data_base_dir, "results/", data_source, ...
    data_name, "/"], "");
analysis_name = "point_to_plane_jacobians";
config.filenames = "s2m_2022-10-23-15-44-50.bag";
config.names = "s2m";
config.odom_topics = repmat("/serpent/optimisation/odometry", 1, ...
    length(config.names));

% Plot Options
plot_opts = default_plot_opts();
plot_opts.save = true;
plot_opts.save_dir = join([data_base_dir, "analysis/", data_source, ...
    data_name, "/", analysis_name, "/"], "");

% Odometry
data = odom_data_from_rosbags(config);
[pose_data, twist_data] = plot_and_save(data, plot_opts);
