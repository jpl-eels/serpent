function track_retention_fractions = compute_track_retention(...
    tracked_match_counts, total_match_counts)
    track_retention_fractions = zeros(length(tracked_match_counts), 1);
    track_retention_fractions(1) = nan;
    track_retention_fractions(2:end) = tracked_match_counts(2:end) ./ ...
        total_match_counts(1:(end-1));
    track_retention_fractions(track_retention_fractions == Inf) = nan;
end