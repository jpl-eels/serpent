clc;
clear;
close all;

% Configuration
config = struct;
config.gt = struct;
config.gt.data_dir = "/home/william/data/simulator_prcp_gazebo/";
config.gt.filename = "surface_spiral_2.bag";
config.gt.topic = "/ground_truth";
config.gt.accelerometer_bias = [0.0, 0.0, 0.0];
config.gt.gyroscope_bias = [0.0, 0.0, 0.0];
config.data_dir = join([config.gt.data_dir, ...
    "surface_spiral_2_analysis/"], "");
config.filenames = ["serpent_imu_int_2/odometry.bag", "serpent_imu_opt_2/odometry.bag"];
config.names = ["imu int", "imu opt"];
config.odom_topics = ["/integrate_imu/odometry", ...
    "/serpent/optimisation/odometry"];

% Plot Options
plot_opts = default_plot_opts();
plot_opts.save = true;
plot_opts.save_dir = join([config.gt.data_dir, "matlab_analysis/", ...
    replace(config.gt.filename, ".bag", "/"), "serpent_imu_replot/"], "");

data = data_from_rosbags(config);

[pose_data, twist_data] = plot_and_save(data, plot_opts);
