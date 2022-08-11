clc;
clear;
close all;

% Configuration
config = struct;
config.gt = struct;
config.gt.data_dir = "/home/william/data/ue4_pedro/";
config.gt.filename = "replaced_gt_replaced_topic_names_replaced_frame_ids_ue4_icy_surface_0.bag";
config.gt.topic = "/true_pose_imu_rate";
config.gt.accelerometer_bias = [0.0, 0.0, 0.0];
config.gt.gyroscope_bias = [0.0, 0.0, 0.0];
config.data_dir = join([config.gt.data_dir, "analysis/"], "");
config.filenames = ["serpent_s2s_1/analysis.bag", ...
    "serpent_s2m_1/analysis.bag"];
config.names = ["s2s", "s2m"];
config.odom_topics = ["/serpent/optimisation/odometry", ...
    "/serpent/optimisation/odometry"];

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
plot_opts.save_dir = join([config.gt.data_dir, "plots/", ...
    "serpent_pc_reg/"], "");
plot_opts.close_after_save = false;

data = data_from_rosbags(config);

[pose_data, twist_data] = plot_and_save(data, plot_opts);
