% Read rosbags and generate all plots

clc;
clear;
close all;

% Configuration
config = struct;
config.gt = struct;
config.gt.data_dir = join([getenv("HOME"), ...
    "/data/eels1/highbay_82_aug_31/analysis/"], "");
config.gt.filename = "serpent/s2m_unsimplified_thirdrate2_2022-09-12-21-25-28.bag";
config.gt.topic = "/serpent/optimisation/odometry";
config.gt.accelerometer_bias = [0.0, 0.0, 0.0];
config.gt.gyroscope_bias = [0.0, 0.0, 0.0];
config.data_dir = join([config.gt.data_dir, "serpent/"], "");
config.filenames = ["s2m_simplified_thirdrate2_2022-09-12-21-57-55.bag"];
config.names = ["S2M frame simplify 1"];
config.odom_topics = repmat("/serpent/optimisation/odometry", 1, ...
    length(config.names));

% Plot Options
plot_opts = default_plot_opts();
plot_opts.save = true;
plot_opts.save_dir = join([config.gt.data_dir, "matlab/", ...
    "frame_simplify/"], "");

data = data_from_rosbags(config);

[pose_data, twist_data] = plot_and_save(data, plot_opts);
