function statistics_data = extract_statistics(data)
    statistics_data = struct;
    statistics_data.names = data.names;
    statistics_data.num_stats = length(data.statistics);
    statistics_data.entries = cell(statistics_data.num_stats, 1);
    for i = 1:statistics_data.num_stats
        entry = struct;
        statistics = data.statistics{i};
        entry.frame_numbers = cellfun(@(m) double(m.FrameNumber), ...
            statistics);
        entry.timestamps = extract_timestamps(statistics);
        entry.max_match_ids = cellfun(@(m) double(m.MaxMatchId), ...
            statistics);
        entry.longest_tracked_match_ids = cellfun(@(m) ...
            double(m.LongestTrackedMatchId), statistics);
        entry.tracked_kp_counts.left = cellfun(@(m) ...
            double(m.TrackedKpCount(1)), statistics);
        entry.tracked_kp_counts.right = cellfun(@(m) ...
            double(m.TrackedKpCount(2)), statistics);
        entry.tracked_match_counts = cellfun(@(m) ...
            double(m.TrackedMatchCount), statistics);
        entry.extracted_kp_counts.left = cellfun(@(m) ...
            double(m.ExtractedKpCount(1)), statistics);
        entry.extracted_kp_counts.right = cellfun(@(m) ...
            double(m.ExtractedKpCount(2)), statistics);
        entry.filtered_extracted_kp_counts.left = ...
            cellfun(@(m) double(m.FilteredExtractedKpCount(1)), ...
            statistics);
        entry.filtered_extracted_kp_counts.right = ...
            cellfun(@(m) double(m.FilteredExtractedKpCount(2)), ...
            statistics);
        entry.new_match_counts = cellfun(@(m) double(m.NewMatchCount), ...
            statistics);
        entry.total_match_counts = cellfun(@(m) ...
            double(m.TotalMatchCount), statistics);
        entry.track_retention_fractions = compute_track_retention(...
            entry.tracked_match_counts, entry.total_match_counts);
        entry.mean_track_retention_fraction = mean( ...
            entry.track_retention_fractions( ...
            ~isnan(entry.track_retention_fractions)));
        statistics_data.entries{i} = entry;
    end
end