% Read rosbags and generate all plots

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
config.data_dir = "/home/william/data/simulator_prcp_gazebo/surface_spiral_2_analysis/";
config.filenames = ["serpent_imu_opt_1/odometry.bag", "serpent_imu_int_1/odometry.bag"];
config.names = ["imu opt", "imu int"];
config.odom_topics = ["/serpent/optimisation/odometry", "/integrate_imu/odometry"];

plot_opts.angle = "deg";
plot_opts.figure_dims = [0, 0, 2400, 1600];
plot_opts.save = true;
plot_opts.filetypes = ["fig", "png"];
plot_opts.save_dir = join([config.gt.data_dir, "matlab_analyis/", ...
    replace(config.gt.filename, ".bag", "/")], "");
plot_opts.close_after_save = false;

data = data_from_rosbags(config);

plot_and_save(data, plot_opts);
