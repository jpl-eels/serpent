% Read, plot and save serpent/StereoTrackerStatistics messages

clc;
clear;
close all;

% Configuration
config = struct;
config.data_dir = join([getenv("HOME"), "/data/ue4_pedro/", ...
    "analysis/"], "");
config.filenames = ["fast_thr25_statistics_1/analysis.bag", ...
    "gftt_100_statistics_1/analysis.bag", ...
    "orb_500_statistics_1/analysis.bag", ...
    "sift_100_statistics_1/analysis.bag"];
config.names = ["FAST thr=25", "GFTT 100", "ORB 500", "SIFT 100"];
config.stats_topics = repmat("/serpent/stereo/statistics", 1, ...
    length(config.names));

% Plot Options
plot_opts = default_plot_opts();
plot_opts.save = true;
plot_opts.save_dir = join([config.data_dir, "plots/", ...
    "feature_comparison_with_matches/"], "");

% Processings
data = stereo_statistics_from_rosbags(config);
statistics_data = stereo_statistics_plot_and_save(data, plot_opts);