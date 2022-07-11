% Read rosbags and generate all plots

clc;
clear;
close all;

% Configuration
config = struct;
config.gt = struct;
% config.gt.data_dir = "/home/william/data/simulator_prcp_gazebo/";
config.gt.data_dir = "/home/william/data/jpl_snowboard/LIDAR-IMU_dataset_challenging_flat_snow_surface_2022-03-31/";
% config.gt.filename = "surface_spiral_2.bag";
config.gt.filename = "analysis/ground_truth/odometry.bag";
% config.gt.topic = "/ground_truth";
config.gt.topic = "/snowboard/lo_frontend/odometry";
config.gt.accelerometer_bias = [0.0, 0.0, 0.0];
config.gt.gyroscope_bias = [0.0, 0.0, 0.0];
% config.data_dir = join([config.gt.data_dir, ...
%     "surface_spiral_2_analysis/"], "");
config.data_dir = join([config.gt.data_dir, "analysis/"], "");
config.filenames = ["serpent_s2s_3/odometry.bag", ...
    "serpent_s2m_3/odometry.bag"];
config.names = ["s2s", "s2m"];
config.odom_topics = ["/serpent/optimisation/odometry", ...
    "/serpent/optimisation/odometry"];

plot_opts.align_first_pose = true;
plot_opts.start_from_time_zero = true;
plot_opts.angle = "deg";
plot_opts.figure_dims = [0, 0, 2400, 1600];
plot_opts.save = true;
plot_opts.filetypes = ["fig", "png"];
% plot_opts.save_dir = join([config.gt.data_dir, "matlab_analyis/", ...
%     replace(config.gt.filename, ".bag", "/"), "serpent_imu/"], "");
plot_opts.save_dir = join([config.gt.data_dir, "matlab_analyis/", ...
    "mammoth/", "serpent_s2s_s2m_v3_vs_body/"], "");
plot_opts.close_after_save = false;

data = data_from_rosbags(config);

plot_and_save(data, plot_opts);
