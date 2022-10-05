clc;
clear;
close all;

% Configuration
config = struct;
config.gt = struct;
data_dir = join([getenv("HOME"), "/data/"], "");
data_source = "simulator_prcp_gazebo/";
data_name = "surface_ellipse_2";
config.gt.data_dir = join([data_dir, data_source], "");
config.gt.filename = join([data_name, ".bag"], "");
config.gt.topic = "/ground_truth";
config.gt.accelerometer_bias = [0.0, 0.0, 0.0];
config.gt.gyroscope_bias = [0.0, 0.0, 0.0];
config.data_dir = join([data_dir, "results/", data_source, data_name, ...
    "/"], "");
config.filenames = ["s2s_2022-09-23-14-40-23.bag", ...
    "s2s_imu_2022-09-23-14-42-49.bag"];
config.names = ["s2s", "s2s imu"];
config.odom_topics = repmat("/serpent/optimisation/odometry", 1, ...
    length(config.names));

plot_opts.align_first_pose = true;
plot_opts.start_from_time_zero = true;
plot_opts.summary_decimal_places = 3;
plot_opts.angle = "deg";
plot_opts.figure_dims = [0, 0, 2400, 1600];
plot_opts.colours = {[0 0.4470 0.7410], [0.8500 0.3250 0.0980], ...
    [0.9290 0.6940 0.1250], [0.4940 0.1840 0.5560], ...
    [0.4660 0.6740 0.1880], [0.3010 0.7450 0.9330],[0.6350 0.0780 0.1840]};
plot_opts.save = true;
plot_opts.filetypes = ["fig", "png"];
plot_opts.save_dir = join([data_dir, "analysis/", data_source, ...
    data_name, "/"], "");
plot_opts.close_after_save = false;

data = data_from_rosbags(config);

[pose_data, twist_data] = plot_and_save(data, plot_opts);
