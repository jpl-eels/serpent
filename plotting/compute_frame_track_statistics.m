function [frames_tracked, frames_tracked_bins, mean_frames_tracked, ...
    min_frames_tracked, max_frames_tracked, max_track_duration] = ...
    compute_frame_track_statistics(match_ids)
    frames_tracked = cell(length(match_ids), 1);
    mean_frames_tracked = zeros(length(match_ids), 1);
    max_frames_tracked = zeros(length(match_ids), 1);
    min_frames_tracked = zeros(length(match_ids), 1);
    last_frame_data = containers.Map('KeyType', 'int32', 'ValueType', ...
        'int32');
    for i = 1:length(match_ids)
        frame_match_ids = match_ids{i};
        frames_tracked{i} = zeros(length(frame_match_ids), 1);
        present_in_last_frame = isKey(last_frame_data, ...
            num2cell(frame_match_ids'));
        new_frame_data = containers.Map('KeyType', 'int32', ...
            'ValueType', 'int32');
        for j = 1:length(frame_match_ids)
            id = frame_match_ids(j);
            if present_in_last_frame(j)
                new_frame_data(id) = last_frame_data(id) + 1;
            else
                new_frame_data(id) = 1;
            end
            frames_tracked{i}(j) = new_frame_data(id);
        end
        mean_frames_tracked(i) = mean(frames_tracked{i});
        max_frames_tracked(i) = max(frames_tracked{i});
        min_frames_tracked(i) = min(frames_tracked{i});
        last_frame_data = new_frame_data;
    end
    max_track_duration = max(max_frames_tracked);
    frames_tracked_bins = zeros(length(match_ids), max_track_duration);
    for i = 1:length(match_ids)
        for j = 1:length(frames_tracked{i})
            frames_tracked_bins(i, frames_tracked{i}(j)) = ...
                frames_tracked_bins(i, frames_tracked{i}(j)) + 1;
        end
    end
end