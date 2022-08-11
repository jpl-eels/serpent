% Read, plot and save serpent/StereoTrackerStatistics messages

clc;
clear;
close all;

% Configuration
config = struct;
config.data_dir = join([getenv("HOME"), "/data/ue4_pedro/", ...
    "analysis/"], "");
% config.filenames = ["fast_20_statistics_0/analysis.bag", ...
%     "gftt_100_statistics_0/analysis.bag", ...
%     "orb_100_statistics_0/analysis.bag", ...
%     "sift_100_statistics_0/analysis.bag"];
config.filenames = ["gftt_100_statistics_1/analysis.bag"];
% config.names = ["FAST thr=20", "GFTT 100", "ORB 100", "SIFT 100"];
config.names = ["GFTT 100"];
config.stats_topics = repmat("/serpent/stereo/statistics", 1, ...
    length(config.names));

% Plot Options
plot_opts.use_frame_numbers = false;
plot_opts.summary_decimal_places = 3;
plot_opts.figure_dims = [0, 0, 2400, 1600];
plot_opts.colours = {[0 0.4470 0.7410], [0.8500 0.3250 0.0980], ...
    [0.9290 0.6940 0.1250], [0.4940 0.1840 0.5560], ...
    [0.4660 0.6740 0.1880], [0.3010 0.7450 0.9330],[0.6350 0.0780 0.1840]};
plot_opts.save = true;
plot_opts.filetypes = ["fig", "png"];
plot_opts.save_dir = join([config.data_dir, "plots/", ...
    "feature_comparison_with_matches/"], "");
plot_opts.close_after_save = false;

% Processings
data = stereo_statistics_from_rosbags(config);
statistics_data = stereo_statistics_plot_and_save(data, plot_opts);