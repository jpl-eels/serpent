clc;
clear;
close all;

% Configuration
config = struct;
config.gt = struct;
data_base_dir = join([getenv("HOME"), "/data/jpl/"], "");
data_source = "eels1/";
data_name = "athabasca_053";
% No GT yet, so use odometry
config.gt.data_dir = join([data_base_dir, "results/", data_source, ...
    data_name, "/"], "");
config.gt.filename = join(["fixed_cov_s2m_imu_2022-10-18-23-12-51", ...
    ".bag"], "");
config.gt.topic = "/serpent/optimisation/odometry";
config.gt.accelerometer_bias = [0.0, 0.0, 0.0];
config.gt.gyroscope_bias = [0.0, 0.0, 0.0];
config.data_dir = join([data_base_dir, "results/", data_source, ...
    data_name, "/"], "");
analysis_name = "point_to_plane_jacobians";
config.filenames = ["fixed_cov_s2m_imu_2022-10-18-23-12-51.bag"];
config.names = ["s2m imu"];
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

% Jacobians
jacobian_data = jacobian_data_from_rosbags(config, ...
    "/serpent/debug/registration_jacobian");
plot_opts.axis_labels = [["J r_x", "J r_y", "J r_z", "J t_x", "J t_y", ...
    "J t_z"]; ["J^2 r_x", "J^2 r_y", "J^2 r_z", "J^2 t_x", "J^2 t_y", ...
    "J^2 t_z"]];
plot_opts.axis_limits = [[-1000, 1000]; [-1000000, 1000000]];
jacobians = plot_and_save_jacobians(jacobian_data, plot_opts);